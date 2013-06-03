/*
 * Cryptographic API.
 *
 * Support for OMAP AES HW acceleration.
 *
 * Copyright (c) 2010 Nokia Corporation
 * Author: Dmitry Kasatkin <dmitry.kasatkin@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */
/*
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Herman Schuurman
 * Change: July 2011 - Adapted the omap-aes.c driver to support Netra
 *	implementation of DES hardware accelerator.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <crypto/scatterwalk.h>
#include <crypto/des.h>

#include <mach/hardware.h>
#include <plat/cpu.h>
#include "nss.h"
#include "nss-cdma.h"

#define DEFAULT_TIMEOUT		(5*HZ)

#define FLAGS_MODE_MASK		0x000f
#define FLAGS_ENCRYPT		BIT(0)
#define FLAGS_CBC		BIT(1)
#define FLAGS_GIV		BIT(3)

#define FLAGS_INIT		BIT(4)
#define FLAGS_FAST		BIT(5)
#define FLAGS_BUSY		BIT(6)

struct nss_des_ctx {
	struct nss_des_dev *dd;

	int		keylen;
	u32		key[DES3_EDE_KEY_SIZE / sizeof(u32)];
	unsigned long	flags;
};

struct nss_des_reqctx {
	unsigned long mode;
};

#define NSS_DES_QUEUE_LENGTH	1
#define NSS_DES_CACHE_SIZE	0

struct nss_des_dev {
	struct list_head		list;
	unsigned long			phys_base;
	void __iomem			*io_base;
//	struct clk			*iclk;
	struct nss_des_ctx		*ctx;
	struct device			*dev;
	unsigned long			flags;
	int				err;

	spinlock_t			lock;
	struct crypto_queue		queue;

	struct tasklet_struct		done_task;
	struct tasklet_struct		queue_task;

	struct ablkcipher_request	*req;
	size_t				total;
	struct scatterlist		*in_sg;
	size_t				in_offset;
	struct scatterlist		*out_sg;
	size_t				out_offset;

	size_t				buflen;
	void				*buf_in;
	size_t				dma_size;
	int				dma_in;
	int				dma_lch_in;
	dma_addr_t			dma_addr_in;
	void				*buf_out;
	int				dma_out;
	int				dma_lch_out;
	dma_addr_t			dma_addr_out;
};

/* keep registered devices data here */
static LIST_HEAD(dev_list);
static DEFINE_SPINLOCK(list_lock);

static inline u32 nss_des_read(struct nss_des_dev *dd, u32 offset)
{
	return __raw_readl(dd->io_base + offset);
}

static inline void nss_des_write(struct nss_des_dev *dd, u32 offset,
				  u32 value)
{
	__raw_writel(value, dd->io_base + offset);
}

static inline void nss_des_write_mask(struct nss_des_dev *dd, u32 offset,
				       u32 value, u32 mask)
{
	u32 val;

	val = nss_des_read(dd, offset);
	val &= ~mask;
	val |= value;
	nss_des_write(dd, offset, val);
}

static void nss_des_write_n(struct nss_des_dev *dd, u32 offset,
			     u32 *value, int count)
{
	for (; count--; value++, offset += 4)
		nss_des_write(dd, offset, *value);
}

static int nss_des_wait(struct nss_des_dev *dd, u32 offset, u32 bit)
{
	unsigned long timeout = jiffies + DEFAULT_TIMEOUT;

	while (!(nss_des_read(dd, offset) & bit)) {
		if (time_is_before_jiffies(timeout)) {
			dev_err(dd->dev, "nss-des timeout\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int nss_des_hw_init(struct nss_des_dev *dd)
{
	/*
	 * clocks are enabled when request starts and disabled when finished.
	 * It may be long delays between requests.
	 * Device might go to off mode to save power.
	 */
//	clk_enable(dd->iclk);

	if (!(dd->flags & FLAGS_INIT)) {
		/* is it necessary to reset before every operation? */
		nss_des_write_mask(dd, DES_REG_SYSCFG, DES_REG_SYSCFG_SOFTRESET,
				    DES_REG_SYSCFG_SOFTRESET);
		/*
		 * prevent OCP bus error (SRESP) in case an access to the module
		 * is performed while the module is coming out of soft reset
		 */
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");

		if (nss_des_wait(dd, DES_REG_SYSSTATUS,
				  DES_REG_SYSSTATUS_RESETDONE))
			return -ETIMEDOUT;

		nss_des_write(dd, DES_REG_SYSCFG, DES_REG_SYSCFG_SIDLE_SMARTIDLE | DES_REG_SYSCFG_AUTOIDLE);

		dd->flags |= FLAGS_INIT;
		dd->err = 0;
	}

	return 0;
}

static int nss_des_write_ctrl(struct nss_des_dev *dd)
{
	unsigned int key32;
	int i, err;
	u32 val, mask;

	err = nss_des_hw_init(dd);
	if (err)
		return err;

	val = 0;
	if (dd->dma_lch_out >= 0)
		val |= DES_REG_SYSCFG_DREQ_DATA_OUT_EN;
	if (dd->dma_lch_in >= 0)
		val |= DES_REG_SYSCFG_DREQ_DATA_IN_EN;

	nss_des_write_mask(dd, DES_REG_SYSCFG, val, DES_REG_SYSCFG_DREQ_MASK);

	pr_debug("Set key\n");
	key32 = dd->ctx->keylen / sizeof(u32);

	/* set a key */
	for (i = 0; i < key32; i++) {
		nss_des_write(dd, DES_REG_KEYS(i),
			       __le32_to_cpu(dd->ctx->key[i]));
	}

	if ((dd->flags & FLAGS_CBC) && dd->req->info)
		nss_des_write_n(dd, DES_REG_IV_L, dd->req->info, 2);

	val = DES_REG_CTRL_TDES;
	if (dd->flags & FLAGS_CBC)
		val |= DES_REG_CTRL_MODE_CBC;
	if (dd->flags & FLAGS_ENCRYPT)
		val |= DES_REG_CTRL_DIRECTION;

	mask = DES_REG_CTRL_MODE_MASK | DES_REG_CTRL_DIRECTION | DES_REG_CTRL_TDES;

	nss_des_write_mask(dd, DES_REG_CTRL, val, mask);

	/* CDMA IN */
	nss_set_cdma_dest_params(dd->dma_lch_in, 0, NSS_CDMA_AMODE_CONSTANT,
				 dd->phys_base + DES_REG_DATA_L, 0, 2);

	nss_set_cdma_dest_burst_mode(dd->dma_lch_in, NSS_CDMA_DATA_BURST_4);
	nss_set_cdma_src_burst_mode(dd->dma_lch_in, NSS_CDMA_DATA_BURST_4);

	/* CDMA OUT */
	nss_set_cdma_src_params(dd->dma_lch_out, 0, NSS_CDMA_AMODE_CONSTANT,
				dd->phys_base + DES_REG_DATA_L, 0, 2);

	nss_set_cdma_src_burst_mode(dd->dma_lch_out, NSS_CDMA_DATA_BURST_4);
	nss_set_cdma_dest_burst_mode(dd->dma_lch_out, NSS_CDMA_DATA_BURST_4);

	return 0;
}

static struct nss_des_dev *nss_des_find_dev(struct nss_des_ctx *ctx)
{
	struct nss_des_dev *dd = NULL;

	spin_lock_bh(&list_lock);
	if (!ctx->dd) {
		dd = list_first_entry(&dev_list, typeof(*dd), list);
		list_rotate_left(&dev_list);
		ctx->dd = dd;
	} else {
		/* already found before */
		dd = ctx->dd;
	}
	spin_unlock_bh(&list_lock);

	return dd;
}

static void nss_des_cdma_callback(int lch, u16 ch_status, void *data)
{
	struct nss_des_dev *dd = data;

	if (ch_status != NSS_CDMA_BLOCK_IRQ) {
		pr_err("nss-des DMA error status: 0x%hx\n", ch_status);
		dd->err = -EIO;
		dd->flags &= ~FLAGS_INIT; /* request to re-initialize */
	} else if (lch == dd->dma_lch_in) {
		return;
	}

	/* dma_lch_out - completed */
	tasklet_schedule(&dd->done_task);
}

static int nss_des_cdma_init(struct nss_des_dev *dd)
{
	int err = -ENOMEM;

	dd->dma_lch_out = -1;
	dd->dma_lch_in = -1;

	dd->buf_in = (void *)__get_free_pages(GFP_KERNEL, NSS_DES_CACHE_SIZE);
	dd->buf_out = (void *)__get_free_pages(GFP_KERNEL, NSS_DES_CACHE_SIZE);
	dd->buflen = PAGE_SIZE << NSS_DES_CACHE_SIZE;
	dd->buflen &= ~(DES3_EDE_BLOCK_SIZE - 1);

	if (!dd->buf_in || !dd->buf_out) {
		dev_err(dd->dev, "unable to alloc pages.\n");
		goto err_alloc;
	}

	/* MAP here */
	dd->dma_addr_in = dma_map_single(dd->dev, dd->buf_in, dd->buflen,
					 DMA_TO_DEVICE);
	if (dma_mapping_error(dd->dev, dd->dma_addr_in)) {
		dev_err(dd->dev, "dma %d bytes error\n", dd->buflen);
		err = -EINVAL;
		goto err_map_in;
	}

	dd->dma_addr_out = dma_map_single(dd->dev, dd->buf_out, dd->buflen,
					  DMA_FROM_DEVICE);
	if (dma_mapping_error(dd->dev, dd->dma_addr_out)) {
		dev_err(dd->dev, "dma %d bytes error\n", dd->buflen);
		err = -EINVAL;
		goto err_map_out;
	}

	err = nss_request_cdma(dd->dma_in, "nss-des-rx",
			       nss_des_cdma_callback, dd, &dd->dma_lch_in);
	if (err) {
		dev_err(dd->dev, "Unable to request DMA channel\n");
		goto err_dma_in;
	}
	err = nss_request_cdma(dd->dma_out, "nss-des-tx",
			       nss_des_cdma_callback, dd, &dd->dma_lch_out);
	if (err) {
		dev_err(dd->dev, "Unable to request DMA channel\n");
		goto err_dma_out;
	}

	return 0;

err_dma_out:
	nss_free_cdma(dd->dma_lch_in);
err_dma_in:
	dma_unmap_single(dd->dev, dd->dma_addr_out, dd->buflen,
			 DMA_FROM_DEVICE);
err_map_out:
	dma_unmap_single(dd->dev, dd->dma_addr_in, dd->buflen, DMA_TO_DEVICE);
err_map_in:
	free_pages((unsigned long)dd->buf_out, NSS_DES_CACHE_SIZE);
	free_pages((unsigned long)dd->buf_in, NSS_DES_CACHE_SIZE);
err_alloc:
	if (err)
		pr_err("error: %d\n", err);
	return err;
}

static void nss_des_cdma_cleanup(struct nss_des_dev *dd)
{
	nss_free_cdma(dd->dma_lch_out);
	nss_free_cdma(dd->dma_lch_in);
	dma_unmap_single(dd->dev, dd->dma_addr_out, dd->buflen,
			 DMA_FROM_DEVICE);
	dma_unmap_single(dd->dev, dd->dma_addr_in, dd->buflen, DMA_TO_DEVICE);
	free_pages((unsigned long)dd->buf_out, NSS_DES_CACHE_SIZE);
	free_pages((unsigned long)dd->buf_in, NSS_DES_CACHE_SIZE);
}

static void sg_copy_buf(void *buf, struct scatterlist *sg,
			unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;

	if (!nbytes)
		return;

	scatterwalk_start(&walk, sg);
	scatterwalk_advance(&walk, start);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}

static int sg_copy(struct scatterlist **sg, size_t *offset, void *buf,
		   size_t buflen, size_t total, int out)
{
	unsigned int count, off = 0;

	while (buflen && total) {
		count = min((*sg)->length - *offset, total);
		count = min(count, buflen);

		if (!count)
			return off;

		/*
		 * buflen and total are DES3_EDE_BLOCK_SIZE size aligned,
		 * so count should be also aligned
		 */

		sg_copy_buf(buf + off, *sg, *offset, count, out);

		off += count;
		buflen -= count;
		*offset += count;
		total -= count;

		if (*offset == (*sg)->length) {
			*sg = sg_next(*sg);
			if (*sg)
				*offset = 0;
			else
				total = 0;
		}
	}

	return off;
}

static int nss_des_crypt_cdma(struct crypto_tfm *tfm, dma_addr_t dma_addr_in,
			      dma_addr_t dma_addr_out, int length)
{
	struct nss_des_ctx *ctx = crypto_tfm_ctx(tfm);
	struct nss_des_dev *dd = ctx->dd;
	int len32;

	pr_debug("len: %d\n", length);

	dd->dma_size = length;

	if (!(dd->flags & FLAGS_FAST))
		dma_sync_single_for_device(dd->dev, dma_addr_in, length,
					   DMA_TO_DEVICE);

	len32 = DIV_ROUND_UP(length, sizeof(u32));

	/* IN */
	nss_set_cdma_transfer_params(dd->dma_lch_in, NSS_CDMA_DATA_TYPE_S32,
				     len32, 1, NSS_CDMA_SYNC_PACKET, dd->dma_in,
				     NSS_CDMA_DST_SYNC);

	nss_set_cdma_src_params(dd->dma_lch_in, 0, NSS_CDMA_AMODE_POST_INC,
				dma_addr_in, 0, 0);

	/* OUT */
	nss_set_cdma_transfer_params(dd->dma_lch_out, NSS_CDMA_DATA_TYPE_S32,
				     len32, 1, NSS_CDMA_SYNC_PACKET,
				     dd->dma_out, NSS_CDMA_SRC_SYNC);

	nss_set_cdma_dest_params(dd->dma_lch_out, 0, NSS_CDMA_AMODE_POST_INC,
				 dma_addr_out, 0, 0);

	nss_start_cdma(dd->dma_lch_in);
	nss_start_cdma(dd->dma_lch_out);

	/* start DMA or disable idle mode */
	nss_des_write_mask(dd, DES_REG_SYSCFG,
			   DES_REG_SYSCFG_DREQ_DATA_OUT_EN | DES_REG_SYSCFG_DREQ_DATA_IN_EN,
			   DES_REG_SYSCFG_DREQ_MASK);

	return 0;
}

static int nss_des_crypt_cdma_start(struct nss_des_dev *dd)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(
		crypto_ablkcipher_reqtfm(dd->req));
	int err, fast = 0, in, out;
	size_t count;
	dma_addr_t addr_in, addr_out;

	pr_debug("total: %d\n", dd->total);

	nss_des_write(dd, DES_REG_LENGTH, dd->total);

	if (sg_is_last(dd->in_sg) && sg_is_last(dd->out_sg)) {
		/* check for alignment */
		in = IS_ALIGNED((u32)dd->in_sg->offset, sizeof(u32));
		out = IS_ALIGNED((u32)dd->out_sg->offset, sizeof(u32));

		fast = in && out;
	}

	if (fast)  {
		count = min(dd->total, sg_dma_len(dd->in_sg));
		count = min(count, sg_dma_len(dd->out_sg));

		if (count != dd->total) {
			pr_err("request length != buffer length\n");
			return -EINVAL;
		}

		pr_debug("fast\n");

		err = dma_map_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			return -EINVAL;
		}

		err = dma_map_sg(dd->dev, dd->out_sg, 1, DMA_FROM_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
			return -EINVAL;
		}

		addr_in = sg_dma_address(dd->in_sg);
		addr_out = sg_dma_address(dd->out_sg);

		dd->flags |= FLAGS_FAST;

	} else {
		/* use cache buffers */
		count = sg_copy(&dd->in_sg, &dd->in_offset, dd->buf_in,
				dd->buflen, dd->total, 0);

		addr_in = dd->dma_addr_in;
		addr_out = dd->dma_addr_out;

		dd->flags &= ~FLAGS_FAST;

	}

	dd->total -= count;

	err = nss_des_crypt_cdma(tfm, addr_in, addr_out, count);
	if (err) {
		dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, dd->out_sg, 1, DMA_TO_DEVICE);
	}

	return err;
}

static void nss_des_finish_req(struct nss_des_dev *dd, int err)
{
	struct ablkcipher_request *req = dd->req;

	pr_debug("err: %d\n", err);

//	clk_disable(dd->iclk);
	dd->flags &= ~FLAGS_BUSY;

	req->base.complete(&req->base, err);
}

static int nss_des_crypt_cdma_stop(struct nss_des_dev *dd)
{
	int err = 0;
	size_t count;

	pr_debug("total: %d\n", dd->total);

	nss_des_write_mask(dd, DES_REG_SYSCFG, 0, DES_REG_SYSCFG_DREQ_MASK);

	nss_stop_cdma(dd->dma_lch_in);
	nss_stop_cdma(dd->dma_lch_out);

	if (dd->flags & FLAGS_FAST) {
		dma_unmap_sg(dd->dev, dd->out_sg, 1, DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
	} else {
		dma_sync_single_for_device(dd->dev, dd->dma_addr_out,
					   dd->dma_size, DMA_FROM_DEVICE);

		/* copy data */
		count = sg_copy(&dd->out_sg, &dd->out_offset, dd->buf_out,
				dd->buflen, dd->dma_size, 1);
		if (count != dd->dma_size) {
			err = -EINVAL;
			pr_err("not all data converted: %u\n", count);
		}
	}

	return err;
}

static int nss_des_handle_queue(struct nss_des_dev *dd,
				 struct ablkcipher_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct nss_des_ctx *ctx;
	struct nss_des_reqctx *rctx;
	unsigned long flags;
	int err, ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = ablkcipher_enqueue_request(&dd->queue, req);
	if (dd->flags & FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);

	/* assign new request to device */
	dd->req = req;
	dd->total = req->nbytes;
	dd->in_offset = 0;
	dd->in_sg = req->src;
	dd->out_offset = 0;
	dd->out_sg = req->dst;

	rctx = ablkcipher_request_ctx(req);
	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	rctx->mode &= FLAGS_MODE_MASK;
	dd->flags = (dd->flags & ~FLAGS_MODE_MASK) | rctx->mode;

	dd->ctx = ctx;
	ctx->dd = dd;

	err = nss_des_write_ctrl(dd);
	if (!err)
		err = nss_des_crypt_cdma_start(dd);
	if (err) {
		/* des_task will not finish it, so do it here */
		nss_des_finish_req(dd, err);
		tasklet_schedule(&dd->queue_task);
	}

	return ret; /* return ret, which is enqueue return value */
}

static void nss_des_done_task(unsigned long data)
{
	struct nss_des_dev *dd = (struct nss_des_dev *)data;
	int err;

	pr_debug("enter\n");

	err = nss_des_crypt_cdma_stop(dd);

	err = dd->err ? : err;

	if (dd->total && !err) {
		err = nss_des_crypt_cdma_start(dd);
		if (!err)
			return; /* CDMA started. Not finishing. */
	}

	nss_des_finish_req(dd, err);
	nss_des_handle_queue(dd, NULL);

	pr_debug("exit\n");
}

static void nss_des_queue_task(unsigned long data)
{
	struct nss_des_dev *dd = (struct nss_des_dev *)data;

	nss_des_handle_queue(dd, NULL);
}

static int nss_des_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct nss_des_ctx *ctx = crypto_ablkcipher_ctx(
		crypto_ablkcipher_reqtfm(req));
	struct nss_des_reqctx *rctx = ablkcipher_request_ctx(req);
	struct nss_des_dev *dd;

	pr_debug("nbytes: %d, enc: %d, cbc: %d\n", req->nbytes,
		 !!(mode & FLAGS_ENCRYPT),
		 !!(mode & FLAGS_CBC));

	if (!IS_ALIGNED(req->nbytes, DES3_EDE_BLOCK_SIZE)) {
		pr_err("request size is not exact amount of DES blocks\n");
		return -EINVAL;
	}

	dd = nss_des_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	rctx->mode = mode;

	return nss_des_handle_queue(dd, req);
}

/* ********************** ALG API ************************************ */

static int nss_des_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct nss_des_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	if (keylen != DES3_EDE_KEY_SIZE)
		return -EINVAL;

	pr_debug("enter, keylen: %d\n", keylen);

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int nss_des_ecb_encrypt(struct ablkcipher_request *req)
{
	return nss_des_crypt(req, FLAGS_ENCRYPT);
}

static int nss_des_ecb_decrypt(struct ablkcipher_request *req)
{
	return nss_des_crypt(req, 0);
}

static int nss_des_cbc_encrypt(struct ablkcipher_request *req)
{
	return nss_des_crypt(req, FLAGS_ENCRYPT | FLAGS_CBC);
}

static int nss_des_cbc_decrypt(struct ablkcipher_request *req)
{
	return nss_des_crypt(req, FLAGS_CBC);
}

static int nss_des_cra_init(struct crypto_tfm *tfm)
{
	pr_debug("enter\n");

	tfm->crt_ablkcipher.reqsize = sizeof(struct nss_des_reqctx);

	return 0;
}

static void nss_des_cra_exit(struct crypto_tfm *tfm)
{
	pr_debug("enter\n");
}

/* ********************** ALGS ************************************ */

static struct crypto_alg algs[] = {
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "ecb-des-nss",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nss_des_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= nss_des_cra_init,
		.cra_exit		= nss_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= nss_des_setkey,
			.encrypt	= nss_des_ecb_encrypt,
			.decrypt	= nss_des_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(des3_ede)",
		.cra_driver_name	= "cbc-des-nss",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nss_des_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= nss_des_cra_init,
		.cra_exit		= nss_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.geniv		= "eseqiv",
			.ivsize		= DES3_EDE_BLOCK_SIZE,
			.setkey		= nss_des_setkey,
			.encrypt	= nss_des_cbc_encrypt,
			.decrypt	= nss_des_cbc_decrypt,

		}
	},
};

struct crypto_resource {
	unsigned long phys_base;
	int dma_out;
	int dma_in;
};

static struct crypto_resource nss_crypto_des_resources[] = {
	{
		.phys_base = NSS_SEC_DES_1S_BASE,
		.dma_out   = NSS_CDMA_DES_S_DATA_OUT_REQ,
		.dma_in    = NSS_CDMA_DES_S_DATA_IN_REQ
	},
	{
		.phys_base = NSS_SEC_DES_1P_BASE,
		.dma_out   = NSS_CDMA_DES_P_DATA_OUT_REQ,
		.dma_in    = NSS_CDMA_DES_P_DATA_IN_REQ
	}
};

static int nss_des_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nss_des_dev *dd, *pdd, *tmp;
	int err = -ENOMEM, i, j, k;
	u32 reg;

	pdd = kzalloc(ARRAY_SIZE(nss_crypto_des_resources) *
		      sizeof(struct nss_des_dev), GFP_KERNEL);
	if (pdd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		goto err_data;
	}
	dd = pdd;
	platform_set_drvdata(pdev, dd);

	for (k = 0; k < ARRAY_SIZE(nss_crypto_des_resources); k++) {
		dd->dev = dev;

		spin_lock_init(&dd->lock);
		crypto_init_queue(&dd->queue, NSS_DES_QUEUE_LENGTH);

		/* Get the module base address and DMA resources */
		/* NOTE: this should be moved over the resources */
		dd->phys_base = nss_crypto_des_resources[k].phys_base;
		dd->dma_out   = nss_crypto_des_resources[k].dma_out;
		dd->dma_in    = nss_crypto_des_resources[k].dma_in;
		dd->io_base   = ioremap(dd->phys_base, SZ_4K);

		if (!dd->io_base) {
			dev_err(dev, "can't ioremap\n");
			err = -ENOMEM;
			goto err_io;
		}
		reg = nss_des_read(dd, DES_REG_REV);
		dev_info(dev, "NSS DES hw accel rev: %u.%u (context %d @0x%08lx)\n",
			 ((reg & DES_REG_REV_X_MAJOR_MASK) >> 8),
			 (reg & DES_REG_REV_Y_MINOR_MASK), k, dd->phys_base);

		tasklet_init(&dd->done_task, nss_des_done_task, (unsigned long)dd);
		tasklet_init(&dd->queue_task, nss_des_queue_task, (unsigned long)dd);

		err = nss_des_cdma_init(dd);
		if (err)
			goto err_dma;

		INIT_LIST_HEAD(&dd->list);
		spin_lock(&list_lock);
		list_add_tail(&dd->list, &dev_list);
		spin_unlock(&list_lock);
		dd++;
	}

	for (i = 0; i < ARRAY_SIZE(algs); i++) {
		pr_debug("i: %d\n", i);
		INIT_LIST_HEAD(&algs[i].cra_list);
		err = crypto_register_alg(&algs[i]);
		if (err)
			goto err_algs;
	}

	pr_info("probe() done\n");

	return 0;

err_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&algs[j]);
	nss_des_cdma_cleanup(dd);
err_dma:
	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
	iounmap(dd->io_base);
err_io:
	/* delete all contexts that already were registered */
	spin_lock_bh(&list_lock);
	list_for_each_entry(tmp, &dev_list, list) {
		dd = tmp;
		spin_lock(&list_lock);
		list_del(&dd->list);
		spin_unlock(&list_lock);

		tasklet_kill(&dd->done_task);
		tasklet_kill(&dd->queue_task);
		nss_des_cdma_cleanup(dd);
		iounmap(dd->io_base);
	}
	spin_unlock_bh(&list_lock);

	kfree(dd);
	dd = NULL;
err_data:
	dev_err(dev, "initialization failed.\n");
	return err;
}

static int nss_des_remove(struct platform_device *pdev)
{
	struct nss_des_dev *pdd = platform_get_drvdata(pdev);
	struct nss_des_dev *dd;
	int i;

	if (!pdd)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(algs); i++)
		crypto_unregister_alg(&algs[i]);

	dd = pdd;
	for (i = 0; i < ARRAY_SIZE(nss_crypto_des_resources); i++) {
		spin_lock(&list_lock);
		list_del(&dd->list);
		spin_unlock(&list_lock);

		tasklet_kill(&dd->done_task);
		tasklet_kill(&dd->queue_task);
		nss_des_cdma_cleanup(dd);
		iounmap(dd->io_base);
		dd++;
	}

	kfree(pdd);

	return 0;
}

static struct platform_driver nss_des_driver = {
	.probe	= nss_des_probe,
	.remove	= nss_des_remove,
	.driver	= {
		.name	= "nss-des",
		.owner	= THIS_MODULE,
	},
};

static int __init nss_des_mod_init(void)
{
	pr_info("loading NSS DES driver\n");

	/* This only works on a GP device */
	if (!cpu_is_ti81xx() || omap_type() != OMAP2_DEVICE_TYPE_GP) {
		pr_err("Unsupported cpu\n");
		return -ENODEV;
	}
	return  platform_driver_register(&nss_des_driver);
}

static void __exit nss_des_mod_exit(void)
{
	pr_info("unloading NSS DES driver\n");

	platform_driver_unregister(&nss_des_driver);
}

module_init(nss_des_mod_init);
module_exit(nss_des_mod_exit);

MODULE_DESCRIPTION("NSS DES/3DES acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Herman Schuurman");
