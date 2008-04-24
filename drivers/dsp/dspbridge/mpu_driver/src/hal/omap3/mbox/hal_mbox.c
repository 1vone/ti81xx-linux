/*
 * dspbridge/src/hal/common/mbox/hal_mbox.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
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
 *  ======== hal_mbox.c ========
 *  Description:
 *      Mailbox messaging & configuration API definitions
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

/* ============================================================================
* STANDARD INCLUDE FILES
* =============================================================================
*/

/* ============================================================================
* PROJECT SPECIFIC INCLUDE FILES
* =============================================================================
*/
#include <GlobalTypes.h>
#include "MLBRegAcM.h"
#include <hal_defs.h>
#include <hal_mbox.h>
/* ============================================================================
* GLOBAL VARIABLES DECLARATIONS
* =============================================================================
*/

/* ============================================================================
* LOCAL TYPES AND DEFINITIONS
* =============================================================================
*/

/* width in bits of MBOX Id */
#define HAL_MBOX_ID_WIDTH	   2

/* ============================================================================
* LOCAL VARIABLES DECLARATIONS
* =============================================================================
*/

/* ============================================================================
* LOCAL FUNCTIONS PROTOTYPES
* =============================================================================
*/

/* ============================================================================
* FUNCTIONS
* =============================================================================
*/


struct MAILBOX_CONTEXT mboxsetting = {0, 0, 0};

/*
 *  ======== HAL_MBOX_saveSettings ========
 *  purpose:
 *  	Saves the mailbox context
 */
HAL_STATUS HAL_MBOX_saveSettings(UWORD32    baseAddress)
{
	HAL_STATUS status = RET_OK;

	mboxsetting.sysconfig = MLBMAILBOX_SYSCONFIGReadRegister32(baseAddress);
	/* Get current enable status */
	mboxsetting.irqEnable0 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				 (baseAddress, HAL_MBOX_U0_ARM);
	mboxsetting.irqEnable1 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				 (baseAddress, HAL_MBOX_U1_DSP1);
	return status;
}
/*==================== Function Separator =============================*/
/*
 *  ======== HAL_MBOX_restoreSettings ========
 *  purpose:
 *  	Restores the mailbox context
 */
HAL_STATUS HAL_MBOX_restoreSettings(UWORD32    baseAddress)
{
	 HAL_STATUS status = RET_OK;
	/* Restor IRQ enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, HAL_MBOX_U0_ARM,
						 mboxsetting.irqEnable0);
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, HAL_MBOX_U1_DSP1,
						 mboxsetting.irqEnable1);
	/* Restore Sysconfig register */
	MLBMAILBOX_SYSCONFIGWriteRegister32(baseAddress, mboxsetting.sysconfig);
	return status;
}
/*==================== Function Separator =============================*/
HAL_STATUS HAL_MBOX_MsgRead(
		      const UWORD32    baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      UWORD32 *const   pReadValue
		      )
{
    HAL_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pReadValue, NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HAL_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Read 32-bit message in mail box */
    *pReadValue = MLBMAILBOX_MESSAGE___0_15ReadRegister32(baseAddress,
							 (UWORD32)mailBoxId);

    return status;
}

/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_MsgWrite
(
		      const UWORD32   baseAddress,
		      const HAL_MBOX_Id_t  mailBoxId,
		      const UWORD32   writeValue
		      )
{
    HAL_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HAL_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Write 32-bit value to mailbox */
    MLBMAILBOX_MESSAGE___0_15WriteRegister32(baseAddress, (UWORD32)mailBoxId,
					    (UWORD32)writeValue);

    return status;
 }

/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_IsFull(
		      const UWORD32    baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      UWORD32  *const     pIsFull
		  )
{
    HAL_STATUS status = RET_OK;
    UWORD32 fullStatus;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pIsFull,  NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HAL_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* read the is full status parameter for Mailbox */
    fullStatus = MLBMAILBOX_FIFOSTATUS___0_15FifoFullMBmRead32(baseAddress,
							(UWORD32)mailBoxId);

    /* fill in return parameter */
    *pIsFull = (fullStatus & 0xFF);

    return status;
 }

/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_NumMsgGet(
		      const   UWORD32   baseAddress,
		      const   HAL_MBOX_Id_t  mailBoxId,
		      UWORD32 *const    pNumMsg
		  )
{
    HAL_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pNumMsg,  NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);

    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HAL_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Get number of messages available for MailBox */
    *pNumMsg = MLBMAILBOX_MSGSTATUS___0_15NbOfMsgMBmRead32(baseAddress,
							  (UWORD32)mailBoxId);

    return status;
 }

/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_EventEnable(
		      const UWORD32	     baseAddress,
		      const HAL_MBOX_Id_t       mailBoxId,
		  const HAL_MBOX_UserId_t   userId,
		  const UWORD32	     events
		      )
{
	HAL_STATUS status = RET_OK;
	UWORD32      irqEnableReg;

	/* Check input parameters */
	CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
			  RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 mailBoxId,
			 HAL_MBOX_ID_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 enableIrq,
			 HAL_MBOX_INT_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 userId,
			 HAL_MBOX_USER_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* Get current enable status */
	irqEnableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32(baseAddress,
							     (UWORD32)userId);

	/* update enable value */
	irqEnableReg |= ((UWORD32)(events)) << (((UWORD32)(mailBoxId)) *
			HAL_MBOX_ID_WIDTH);

	/* write new enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, (UWORD32)userId,
						 (UWORD32)irqEnableReg);

	mboxsetting.sysconfig = MLBMAILBOX_SYSCONFIGReadRegister32(baseAddress);
	/* Get current enable status */
	mboxsetting.irqEnable0 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				(baseAddress, HAL_MBOX_U0_ARM);
	mboxsetting.irqEnable1 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				(baseAddress, HAL_MBOX_U1_DSP1);
    return status;
 }


/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_EventDisable(
		      const UWORD32	     baseAddress,
		      const HAL_MBOX_Id_t       mailBoxId,
		  const HAL_MBOX_UserId_t   userId,
		  const UWORD32	     events
		      )
{
    HAL_STATUS status = RET_OK;
    UWORD32      irqDisableReg;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     mailBoxId,
		     HAL_MBOX_ID_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     disableIrq,
		     HAL_MBOX_INT_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     userId,
		     HAL_MBOX_USER_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Get current enable status */
    irqDisableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32(baseAddress,
		    (UWORD32)userId);

    /* update enable value */
    irqDisableReg &= ~((UWORD32)(events)) << (((UWORD32)(mailBoxId)) *
		     HAL_MBOX_ID_WIDTH);

    /* write new enable status */
    MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, (UWORD32)userId,
					     (UWORD32)irqDisableReg);

    return status;
 }


/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_EventStatus(
		      const UWORD32	      baseAddress,
		      const HAL_MBOX_Id_t	mailBoxId,
		  const HAL_MBOX_UserId_t    userId,
		  UWORD32 *const	     pEventStatus
		     )
{
    HAL_STATUS status = RET_OK;
    UWORD32      irqStatusReg;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress,   0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pIrqStatus, NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);

    CHECK_INPUT_RANGE_MIN0(
		     mailBoxId,
		     HAL_MBOX_ID_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     userId,
		     HAL_MBOX_USER_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Get Irq Status for specified mailbox/User Id */
    irqStatusReg = MLBMAILBOX_IRQSTATUS___0_3ReadRegister32(baseAddress,
		   (UWORD32)userId);

    /* update status value */
    *pEventStatus = (UWORD32)((((UWORD32)(irqStatusReg)) >>
		    (((UWORD32)(mailBoxId))*HAL_MBOX_ID_WIDTH)) &
		   ((UWORD32)(HAL_MBOX_INT_ALL)));

    return status;
 }

/*==================== Function Separator =============================*/

HAL_STATUS HAL_MBOX_EventAck(
		      const UWORD32	  baseAddress,
		      const HAL_MBOX_Id_t	mailBoxId,
		  const HAL_MBOX_UserId_t    userId,
		  const UWORD32	      event
		     )
{
    HAL_STATUS status = RET_OK;
    UWORD32      irqStatusReg;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress,   0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);

    CHECK_INPUT_RANGE_MIN0(
		     irqStatus,
		     HAL_MBOX_INT_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     mailBoxId,
		     HAL_MBOX_ID_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     userId,
		     HAL_MBOX_USER_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);


    /* calculate status to write */
    irqStatusReg = ((UWORD32)event) << (((UWORD32)(mailBoxId)) *
		   HAL_MBOX_ID_WIDTH);

    /* clear Irq Status for specified mailbox/User Id */
    MLBMAILBOX_IRQSTATUS___0_3WriteRegister32(baseAddress, (UWORD32)userId,
					     (UWORD32)irqStatusReg);

    return status;
 }

/* ============================================================================
* LOCAL FUNCTIONS
* =============================================================================
*/

/* EOF */
