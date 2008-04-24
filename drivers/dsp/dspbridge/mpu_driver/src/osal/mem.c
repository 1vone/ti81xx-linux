/*
 * dspbridge/src/osal/linux/mem.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== mem.c ========
 *  Purpose:
 *      Implementation of platform specific memory services.
 *
 *  Public Functions:
 *      MEM_Alloc
 *      MEM_AllocObject
 *      MEM_AllocPhysMem
 *      MEM_Calloc
 *      MEM_Exit
 *      MEM_FlushCache
 *      MEM_Free
 *      MEM_FreePhysMem
 *      MEM_FreeObject
 *      MEM_GetNumPages
 *      MEM_Init
 *      MEM_IsValidHandle
 *      MEM_LinearAddress
 *      MEM_PageLock
 *      MEM_PageUnlock
 *      MEM_UnMapLinearAddress
 *      MEM_VirtualToPhysical
 *      MEM_ExtPhysPoolInit
 *
 *! Revision History:
 *! =================
 *! 18-Jan-2004 hp: Added support for External physical memory pool
 *! 19-Apr-2004 sb: Added Alloc/Free PhysMem, FlushCache, VirtualToPhysical
 *! 01-Sep-2001 ag: Code cleanup.
 *! 02-May-2001 ag: MEM_[UnMap]LinearAddress revamped to align Phys to Virt.
 *!		 Set PAGE_PHYSICAL if phy addr <= 512MB. Opposite uSoft doc!
 *! 29-Aug-2000 rr: MEM_LinearAddress does not check for 512MB for non-x86.
 *! 28-Mar-2000 rr: MEM_LinearAddress changed.Handles address larger than 512MB
 *! 03-Feb-2000 rr: Module init/exit is handled by OSAL Init/Exit.GT Changes.
 *! 22-Nov-1999 kc: Added changes from code review.
 *! 16-Aug-1999 kc: modified for WinCE.
 *! 20-Mar-1999 ag: SP 4 fix in MEM_UMBCalloc().
 *!		 Mdl offset now ORed not added to userBuf.
 *! 23-Dec-1997 cr: Code review changes.
 *! 08-Dec-1997 cr: Prepared for code review.
 *! 24-Jun-1997 cr: Created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg_zones.h>
#include <gt.h>

/*  ----------------------------------- This */
#include <mem.h>
#include <list.h>

/*  ----------------------------------- Defines */
#define MEM_512MB   0x1fffffff
#define memInfoSign 0x464E494D	/* "MINF" (in reverse). */

#ifdef DEBUG
#define MEM_CHECK		/* Use to detect source of memory leaks */
#endif

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask MEM_debugMask = { 0, 0 };	/* GT trace variable */
#endif

static ULONG cRefs = 0;		/* module reference count */

static BOOL extPhysMemPoolEnabled = FALSE;

struct extPhysMemPool {
	DWORD physMemBase;
	DWORD physMemSize;
	DWORD virtMemBase;
	DWORD nextPhysAllocPtr;
};

struct extPhysMemPool extMemPool;

/*  Information about each element allocated on heap */
struct memInfo {
	struct LST_ELEM link;		/* Must be first */
	size_t size;
	PVOID caller;
	DWORD dwSignature;	/* Should be last */
};

#ifdef MEM_CHECK

/*
 *  This structure holds a linked list to all memory elements allocated on
 *  heap by DSP/BIOS Bridge. This is used to report memory leaks and free
 *  such elements while removing the DSP/BIOS Bridge driver
 */
struct memMan {
	struct LST_LIST lst;
	spinlock_t lock;
};

struct memMan mMan;

/*
 *  These functions are similar to LST_PutTail and LST_RemoveElem and are
 *  duplicated here to make MEM independent of LST
 */
static inline VOID MLST_PutTail(struct LST_LIST *pList, struct LST_ELEM *pElem)
{
	pElem->prev = pList->head.prev;
	pElem->next = &pList->head;
	pList->head.prev = pElem;
	pElem->prev->next = pElem;
	pElem->self = pElem;
}

static inline VOID MLST_RemoveElem(struct LST_LIST *pList,
				   struct LST_ELEM *pCurElem)
{
	pCurElem->prev->next = pCurElem->next;
	pCurElem->next->prev = pCurElem->prev;
	pCurElem->next = NULL;
	pCurElem->prev = NULL;
}

VOID MEM_Check()
{
	struct memInfo *pMem;
	struct LST_ELEM *last = &mMan.lst.head;
	struct LST_ELEM *curr = mMan.lst.head.next;

	if (!LST_IsEmpty(&mMan.lst)) {
		GT_0trace(MEM_debugMask, GT_7CLASS, "*** MEMORY LEAK ***\n");
		GT_0trace(MEM_debugMask, GT_7CLASS,
			  "Addr      Size      Caller\n");
		while (curr != last) {
			pMem = (struct memInfo *)curr;
			curr = curr->next;
			if ((ULONG)pMem > PAGE_OFFSET &&
			    MEM_IsValidHandle(pMem, memInfoSign)) {
				GT_3trace(MEM_debugMask, GT_7CLASS,
					"%lx  %d\t [<%p>]\n",
					(ULONG) pMem + sizeof(struct memInfo),
					pMem->size, pMem->caller);
				MLST_RemoveElem(&mMan.lst,
						(struct LST_ELEM *) pMem);
				kfree(pMem);
			} else {
				GT_1trace(MEM_debugMask, GT_7CLASS,
					  "Invalid allocation or "
					  "Buffer underflow at %x\n",
					  (ULONG)pMem +	sizeof(struct memInfo));
				break;
			}
		}
	}
	DBC_Ensure(LST_IsEmpty(&mMan.lst));
}

#endif

VOID MEM_ExtPhysPoolInit(DWORD poolPhysBase, DWORD poolSize)
{
	DWORD poolVirtBase;

	/* get the virtual address for the physical memory pool passed */
	poolVirtBase = (DWORD)ioremap(poolPhysBase, poolSize);

	if ((PVOID *)poolVirtBase == NULL) {
		GT_0trace(MEM_debugMask, GT_7CLASS,
			  "[PHYS_POOL]Mapping External "
			  "physical memory to virt failed \n");
		extPhysMemPoolEnabled = FALSE;
	} else {
		extMemPool.physMemBase = poolPhysBase;
		extMemPool.physMemSize = poolSize;
		extMemPool.virtMemBase = poolVirtBase;
		extMemPool.nextPhysAllocPtr = poolPhysBase;
		extPhysMemPoolEnabled = TRUE;
		GT_3trace(MEM_debugMask, GT_1CLASS,
			  "ExtMemory Pool details " "Pool"
			  "Physical mem base = %0x " "Pool Physical mem size "
			  "= %0x" "Pool Virtual mem base = %0x \n",
			  poolPhysBase, poolSize, poolVirtBase);
	}
}

VOID MEM_ExtPhysPoolRelease(VOID)
{
	GT_0trace(MEM_debugMask, GT_1CLASS,
		  "Releasing External memory pool \n");
	if (extPhysMemPoolEnabled) {
		iounmap((void *)(extMemPool.virtMemBase));
		extPhysMemPoolEnabled = FALSE;
	}
}

/*
 *  ======== MEM_ExtPhysMemAlloc ========
 *  Purpose:
 *     Allocate physically contiguous, uncached memory from external memory pool
 */

PVOID MEM_ExtPhysMemAlloc(ULONG bytes, ULONG align, OUT ULONG *pPhysAddr)
{
	DWORD newAllocPtr;
	DWORD offset;
	DWORD virtAddr;

	GT_2trace(MEM_debugMask, GT_1CLASS,
		  "Ext Memory Allocation" "bytes=0x%x , "
		  "align=0x%x \n", bytes, align);
	if (align == 0) {
		GT_0trace(MEM_debugMask, GT_7CLASS,
			  "ExtPhysical Memory Allocation "
			  "No alignment request in allocation call !! \n");
		align = 1;
	}
	if (bytes > ((extMemPool.physMemBase + extMemPool.physMemSize)
	    - extMemPool.nextPhysAllocPtr)) {
		GT_1trace(MEM_debugMask, GT_7CLASS,
			  "ExtPhysical Memory Allocation "
			  "unable to allocate memory for bytes = 0x%x \n",
			  bytes);
		pPhysAddr = 0;
		return NULL;
	} else {
		offset = (extMemPool.nextPhysAllocPtr & (align - 1));
		if (offset == 0)
			newAllocPtr = extMemPool.nextPhysAllocPtr;
		else
			newAllocPtr = (extMemPool.nextPhysAllocPtr) +
				      (align - offset);
		if ((newAllocPtr + bytes) <=
		    (extMemPool.physMemBase + extMemPool.physMemSize)) {
			/* we can allocate */
			*pPhysAddr = newAllocPtr;
			extMemPool.nextPhysAllocPtr = newAllocPtr + bytes;
			virtAddr = extMemPool.virtMemBase + (newAllocPtr -
				   extMemPool.physMemBase);
			GT_2trace(MEM_debugMask, GT_1CLASS,
				  "Ext Memory Allocation succedded "
				  "phys address=0x%x , virtaddress=0x%x \n",
				  newAllocPtr, virtAddr);
			return ((PVOID)(virtAddr));
		} else {
			*pPhysAddr = 0;
			return NULL;
		}
	}
}

/*
 *  ======== MEM_Alloc ========
 *  Purpose:
 *      Allocate memory from the paged or non-paged pools.
 */
PVOID MEM_Alloc(ULONG cBytes, MEM_POOLATTRS type)
{
	struct memInfo *pMem = NULL;

	GT_2trace(MEM_debugMask, GT_ENTER,
		  "MEM_Alloc: cBytes 0x%x\ttype 0x%x\n", cBytes, type);
	if (cBytes > 0) {
		switch (type) {
		case MEM_NONPAGED:
		/* If non-paged memory required, see note at top of file. */
		case MEM_PAGED:
#ifndef MEM_CHECK
			pMem = kmalloc(cBytes, GFP_ATOMIC);
#else
			pMem = kmalloc(cBytes + sizeof(struct memInfo),
			       GFP_ATOMIC);
			if (pMem) {
				pMem->size = cBytes;
				pMem->caller = __builtin_return_address(0);
				pMem->dwSignature = memInfoSign;

				spin_lock(&mMan.lock);
				MLST_PutTail(&mMan.lst,
					    (struct LST_ELEM *)pMem);
				spin_unlock(&mMan.lock);

				pMem = (PVOID)((ULONG)pMem +
					sizeof(struct memInfo));
			}
#endif
			break;
		case MEM_LARGEVIRTMEM:
#ifndef MEM_CHECK
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
			/* FIXME - Replace with 'vmalloc' after BP fix */
			pMem = __vmalloc(cBytes, GFP_ATOMIC, PAGE_KERNEL);
#else
			pMem = vmalloc(cBytes);
#endif
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
			/* FIXME - Replace with 'vmalloc' after BP fix */
			pMem = __vmalloc((cBytes + sizeof(struct memInfo)),
				GFP_ATOMIC, PAGE_KERNEL);
#else
			pMem = vmalloc(cBytes + sizeof(struct memInfo));
#endif
			if (pMem) {
				pMem->size = cBytes;
				pMem->caller = __builtin_return_address(0);
				pMem->dwSignature = memInfoSign;

				spin_lock(&mMan.lock);
				MLST_PutTail(&mMan.lst,
					    (struct LST_ELEM *) pMem);
				spin_unlock(&mMan.lock);

				pMem = (PVOID)((ULONG)pMem +
					sizeof(struct memInfo));
			}
#endif
			break;

		default:
			GT_0trace(MEM_debugMask, GT_6CLASS,
				  "MEM_Alloc: unexpected "
				  "MEM_POOLATTRS value\n");
			break;
		}
	}

	return (pMem);
}

/*
 *  ======== MEM_AllocPhysMem ========
 *  Purpose:
 *      Allocate physically contiguous, uncached memory
 */
PVOID MEM_AllocPhysMem(ULONG cBytes, ULONG ulAlign, OUT ULONG *pPhysicalAddress)
{
	PVOID pVaMem = NULL;
	dma_addr_t paMem;

	DBC_Require(cRefs > 0);

	GT_2trace(MEM_debugMask, GT_ENTER,
		  "MEM_AllocPhysMem: cBytes 0x%x\tulAlign"
		  "0x%x\n", cBytes, ulAlign);

	if (cBytes > 0) {
		if (extPhysMemPoolEnabled) {
			pVaMem = MEM_ExtPhysMemAlloc(cBytes, ulAlign,
						    (ULONG *)&paMem);
		} else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
			pVaMem = dma_alloc_coherent(NULL, cBytes, &paMem,
						   GFP_ATOMIC);
#else
			pVaMem = pci_alloc_consistent(NULL, cBytes, &paMem);
#endif

		if (pVaMem == NULL) {
			*pPhysicalAddress = 0;
			GT_1trace(MEM_debugMask, GT_6CLASS,
				  "MEM_AllocPhysMem failed: "
				  "0x%x\n", pVaMem);
		} else {
			*pPhysicalAddress = paMem;
		}
	}
	return pVaMem;
}

/*
 *  ======== MEM_Calloc ========
 *  Purpose:
 *      Allocate zero-initialized memory from the paged or non-paged pools.
 */
PVOID MEM_Calloc(ULONG cBytes, MEM_POOLATTRS type)
{
	struct memInfo *pMem = NULL;

	GT_2trace(MEM_debugMask, GT_ENTER,
		  "MEM_Calloc: cBytes 0x%x\ttype 0x%x\n",
		  cBytes, type);

	if (cBytes > 0) {
		switch (type) {
		case MEM_NONPAGED:
		/* If non-paged memory required, see note at top of file. */
		case MEM_PAGED:
#ifndef MEM_CHECK
			pMem = kmalloc(cBytes, GFP_ATOMIC);
			if (pMem)
				memset(pMem, 0, cBytes);

#else
			pMem = kmalloc(cBytes + sizeof(struct memInfo),
				      GFP_ATOMIC);
			if (pMem) {
				memset((PVOID)((ULONG)pMem +
					sizeof(struct memInfo)), 0, cBytes);
				pMem->size = cBytes;
				pMem->caller = __builtin_return_address(0);
				pMem->dwSignature = memInfoSign;
				spin_lock(&mMan.lock);
				MLST_PutTail(&mMan.lst,
					(struct LST_ELEM *) pMem);
				spin_unlock(&mMan.lock);
				pMem = (PVOID)((ULONG)pMem +
					sizeof(struct memInfo));
			}
#endif
			break;
		case MEM_LARGEVIRTMEM:
#ifndef MEM_CHECK
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
			/* FIXME - Replace with 'vmalloc' after BP fix */
			pMem = __vmalloc(cBytes, GFP_ATOMIC, PAGE_KERNEL);
#else
			/* Need to get fix for vmalloc */
			pMem = vmalloc(cBytes);
#endif
			if (pMem)
				memset(pMem, 0, cBytes);

#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
			/* FIXME - Replace with 'vmalloc' after BP fix */
			pMem = __vmalloc(cBytes + sizeof(struct memInfo),
				GFP_ATOMIC, PAGE_KERNEL);
#else
			pMem = vmalloc(cBytes + sizeof(struct memInfo));
			/* Need to get fix for vmalloc */
#endif
			if (pMem) {
				memset((PVOID)((ULONG)pMem +
					sizeof(struct memInfo)), 0, cBytes);
				pMem->size = cBytes;
				pMem->caller = __builtin_return_address(0);
				pMem->dwSignature = memInfoSign;
				spin_lock(&mMan.lock);
				MLST_PutTail(&mMan.lst, (struct LST_ELEM *)
					pMem);
				spin_unlock(&mMan.lock);
				pMem = (PVOID)((ULONG)pMem +
					sizeof(struct memInfo));
			}
#endif
			break;
		default:
			GT_1trace(MEM_debugMask, GT_6CLASS,
				  "MEM_Calloc: unexpected "
				  "MEM_POOLATTRS value 0x%x\n", type);
			break;
		}
	}

	return (pMem);
}

/*
 *  ======== MEM_Exit ========
 *  Purpose:
 *      Discontinue usage of the MEM module.
 */
VOID MEM_Exit()
{
	DBC_Require(cRefs > 0);

	GT_1trace(MEM_debugMask, GT_5CLASS, "MEM_Exit: cRefs 0x%x\n", cRefs);

	cRefs--;
#ifdef MEM_CHECK
	if (cRefs == 0)
		MEM_Check();

#endif
	MEM_ExtPhysPoolRelease();
	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== MEM_FlushCache ========
 *  Purpose:
 *      Flush cache
 */
VOID MEM_FlushCache(PVOID pMemBuf, ULONG cBytes, INT FlushType)
{

INT FlushMemDirection;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;
#endif
#endif

	DBC_Require(cRefs > 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
		switch (FlushType) {
		case PROC_INVALIDATE_MEM:		/* invalidate only */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
		FlushMemDirection = 	DMA_FROM_DEVICE;
#else
		FlushMemDirection = 	PCI_DMA_FROMDEVICE;
#endif
		break;
		case PROC_WRITEBACK_MEM:		/* writeback only */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
		FlushMemDirection = DMA_TO_DEVICE;
#else
		FlushMemDirection = PCI_DMA_TODEVICE;

#endif
		break;
		/* writeback and invalidate */
		case PROC_WRITEBACK_INVALIDATE_MEM:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
		FlushMemDirection = DMA_BIDIRECTIONAL;
#else
		FlushMemDirection = PCI_DMA_BIDIRECTIONAL;
#endif
		break;
		default:
		GT_1trace(MEM_debugMask, GT_6CLASS, "MEM_FlushCache: invalid "
			  "FlushMemType 0x%x\n", FlushType);

		break;
	}
	GT_0trace(MEM_debugMask, GT_ENTER, "MEM_FlushCache: Entered\n");

	/* Call OEM specific cache synchronization function */
	/* As such conistent_sync is recommeneded onloy for physically
	 * contiguous memory, but on ARM V6 the cache is virtually indexed, so
	 * consistent_sync, should work in vitual memory flush as well
	 */

	consistent_sync(pMemBuf, cBytes, FlushMemDirection);
#else
	switch (FlushType) {
	/* invalidate only */
	case PROC_INVALIDATE_MEM:
		dmac_inv_range((ULONG)pMemBuf, (ULONG)pMemBuf + cBytes - 1);
		outer_inv_range(__pa((ULONG)pMemBuf), __pa((ULONG)pMemBuf +
				cBytes - 1));
	break;
	/* writeback only */
	case PROC_WRITEBACK_MEM:
		dmac_clean_range((ULONG)pMemBuf, (ULONG) pMemBuf + cBytes - 1);
		outer_clean_range(__pa((ULONG)pMemBuf), __pa((ULONG)pMemBuf +
				  cBytes - 1));
	break;
	/* writeback and invalidate */
	case PROC_WRITEBACK_INVALIDATE_MEM:
		dmac_flush_range((ULONG)pMemBuf, (ULONG) pMemBuf + cBytes - 1);
		outer_flush_range(__pa((ULONG)pMemBuf), __pa((ULONG)pMemBuf +
				  cBytes - 1));
	break;
	default:
		GT_1trace(MEM_debugMask, GT_6CLASS, "MEM_FlushCache: invalid "
			  "FlushMemType 0x%x\n", FlushType);
	break;
	}
#if 0
	down_read(&mm->mmap_sem);
	vma = find_vma(mm, pMemBuf);
	up_read(&mm->mmap_sem);
	flush_cache_user_range(vma, (ULONG) pMemBuf, (ULONG) pMemBuf +
			       cBytes - 1);
#endif
#endif
}


/*
 *  ======== MEM_Free ========
 *  Purpose:
 *      Free the given block of system memory.
 */
VOID MEM_Free(IN PVOID pMemBuf)
{
#ifdef MEM_CHECK
	struct memInfo *pMem = (PVOID)((ULONG)pMemBuf - sizeof(struct memInfo));
#endif

	DBC_Require(pMemBuf != NULL);

	GT_1trace(MEM_debugMask, GT_ENTER, "MEM_Free: pMemBufs 0x%x\n",
		  pMemBuf);

	if (pMemBuf) {
#ifndef MEM_CHECK
		kfree(pMemBuf);
#else
		if (pMem) {
			if (pMem->dwSignature == memInfoSign) {
				spin_lock(&mMan.lock);
				MLST_RemoveElem(&mMan.lst,
						(struct LST_ELEM *) pMem);
				spin_unlock(&mMan.lock);
				pMem->dwSignature = 0;
				kfree(pMem);
			} else {
				GT_1trace(MEM_debugMask, GT_7CLASS,
					"Invalid allocation or "
					"Buffer underflow at %x\n",
					(ULONG) pMem + sizeof(struct memInfo));
				/*Do not try to free an invalid address*/
				/*  kfree(pMemBuf);*/
			}
		}
#endif
	}
}

/*
 *  ======== MEM_VFree ========
 *  Purpose:
 *      Free the given block of system memory.
 */
VOID MEM_VFree(IN PVOID pMemBuf)
{
#ifdef MEM_CHECK
	struct memInfo *pMem = (PVOID)((ULONG)pMemBuf - sizeof(struct memInfo));
#endif

	DBC_Require(pMemBuf != NULL);

	GT_1trace(MEM_debugMask, GT_ENTER, "MEM_VFree: pMemBufs 0x%x\n",
		  pMemBuf);

	if (pMemBuf) {
#ifndef MEM_CHECK
		vfree(pMemBuf);
#else
		if (pMem) {
			if (pMem->dwSignature == memInfoSign) {
				spin_lock(&mMan.lock);
				MLST_RemoveElem(&mMan.lst,
						(struct LST_ELEM *) pMem);
				spin_unlock(&mMan.lock);

				pMem->dwSignature = 0;
				vfree(pMem);
			} else {
				GT_1trace(MEM_debugMask, GT_7CLASS,
					  "Invalid allocation or "
					  "Buffer underflow at %x\n",
					  (ULONG) pMem +
					  sizeof(struct memInfo));
				/*Do not try to free an invalid address*/
				/*  kfree(pMemBuf); */
			}
		}
#endif
	}
}

/*
 *  ======== MEM_FreePhysMem ========
 *  Purpose:
 *      Free the given block of physically contiguous memory.
 */
VOID MEM_FreePhysMem(PVOID pVirtualAddress, DWORD pPhysicalAddress,
		     ULONG cBytes)
{
	DBC_Require(cRefs > 0);
	DBC_Require(pVirtualAddress != NULL);

	GT_1trace(MEM_debugMask, GT_ENTER, "MEM_FreePhysMem: pVirtualAddress "
		  "0x%x\n", pVirtualAddress);

	if (!extPhysMemPoolEnabled)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
		dma_free_coherent(NULL, cBytes, pVirtualAddress,
				 pPhysicalAddress);
#else
		consistent_free(pVirtualAddress, cBytes, pPhysicalAddress);
#endif
}

/*
 *  ======== MEM_Init ========
 *  Purpose:
 *      Initialize MEM module private state.
 */
BOOL MEM_Init()
{
	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		GT_create(&MEM_debugMask, "MM");	/* MM for MeM module */

#ifdef MEM_CHECK
		mMan.lst.head.next = &mMan.lst.head;
		mMan.lst.head.prev = &mMan.lst.head;
		mMan.lst.head.self = NULL;
#endif

	}

	cRefs++;

	GT_1trace(MEM_debugMask, GT_5CLASS, "MEM_Init: cRefs 0x%x\n", cRefs);

	DBC_Ensure(cRefs > 0);

	return (TRUE);
}

/*
 *  ======== MEM_VirtualToPhysical ========
 *  Purpose:
 *      Given a user allocated virtual address, return the corresponding
 *      physical address based on the page frame address.
 */
DWORD MEM_VirtualToPhysical(DWORD dwVirtAddr)
{

	DWORD dwPhysAddr;
	DWORD numUsrPgs;
	struct page *aPFNTab[1];
	struct task_struct *curr_task = current;
	struct mm_struct *mm = curr_task->mm;

	DBC_Require(cRefs > 0);
	DBC_Require((PVOID) dwVirtAddr != NULL);

	GT_1trace(MEM_debugMask, GT_ENTER, "MEM_VirtualToPhysical: dwVirtAddr "
		  "0x%x\n", dwVirtAddr);

	down_read(&mm->mmap_sem);
	numUsrPgs = get_user_pages(curr_task, mm, dwVirtAddr, 1, TRUE, 0,
				   aPFNTab, NULL);
	up_read(&mm->mmap_sem);

	if (numUsrPgs == 1) {
		dwPhysAddr = page_to_phys(aPFNTab[0]);
		/* Release the page lock, else this results in a physical
		 * memory leak */
		page_cache_release(aPFNTab[0]);
	} else {
		GT_0trace(MEM_debugMask, GT_4CLASS,
			  "MEM_VirtualToPhysical: attempting "
			  "page table walk !!!!!!: \n");
	struct mm_struct *mm = current->mm;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *ptep, pte;
	struct page *page;

	pgd = pgd_offset(mm, dwVirtAddr);
	if (!(pgd_none(*pgd) || pgd_bad(*pgd))) {
			pmd = pmd_offset(pgd, dwVirtAddr);
			if (!(pmd_none(*pmd) || pmd_bad(*pmd))) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
				ptep = pte_offset_map(pmd, dwVirtAddr);
#else
				ptep = pte_offset(pmd, dwVirtAddr);
#endif
	       if (ptep) {
				   pte = *ptep;
				   if (pte_present(pte)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
					   return(pte & PAGE_MASK);
#else
					   page = pte_page(pte);
					   return(page_to_phys(page));
#endif
				   }
			   }
			}
		}
		GT_1trace(MEM_debugMask, GT_6CLASS,
			  "MEM_VirtualToPhysical: LockPages()"
			  " failed: 0x%x\n", numUsrPgs);

		dwPhysAddr = 0;
	}

	return dwPhysAddr;
}

