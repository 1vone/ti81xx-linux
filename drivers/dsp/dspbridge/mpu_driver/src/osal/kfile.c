/*
 * dspbridge/src/osal/linux/kfile.c
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
 *  ======== kfilece.c ========
 *  Purpose:
 *      This module provides file i/o services.
 *
 *  Public Functions:
 *      KFILE_Close
 *      KFILE_Exit
 *      KFILE_Init
 *      KFILE_Open
 *      KFILE_Read
 *      KFILE_Seek
 *      KFILE_Tell
 *
 *! Revision History
 *! ================
 *! 03-Feb-2000 rr: Module init/exit is handled by OSAL Init/Exit.GT Changes.
 *! 22-Nov-1999 kc: Added changes from code review.
 *! 12-Nov-1999 kc: Enabled CSL for UNICODE/ANSI string conversions.
 *! 30-Sep-1999 ag: Changed KFILE_Read() GT level from _ENTER to _4CLASS.
 *!                 Removed GT_set().
 *! 25-Aug-1999 ag: Changed MEM_Calloc allocation type to MEM_PAGED.
 *! 13-Jul-1999 a0216266(ww - TID): Stubbed from kfilent.c.
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
#include <gp.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <mem.h>
#include <prcs.h>

/*  ----------------------------------- This */
#include <kfile.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE           0x4c49464b	/* hex code of KFIL (reversed) */
#define MAXFILENAMELENGTH   256
#define GENERAL_FAILURE     0xffffffff	/* SetFilePointer error */

/* The KFILE_FileObj abstracts the true file handle from a KFILE handle. */
struct KFILE_FileObj {
    DWORD dwSignature;
    __kernel_pid_t owner_pid;	/* PID of process that opened this file */
    char 	*fileName  ;
    Bool          isOpen    ;
    UINT        size      ;
    UINT        curPos    ;
    long 	  hInternal;		/* internal handle of file */
    struct file *fileDesc;

};

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask KFILE_debugMask = { 0, 0 };	/* Debug mask */
#endif

/*
 *  ======== KFILE_Close ========
 *  Purpose:
 *      This function closes a file's stream.
 */
INT KFILE_Close(struct KFILE_FileObj *hFile)
{
	BOOL fResult = FALSE;
	INT cRetVal = 0;	/* 0 indicates success */
	INT fRetVal = 0;
	__kernel_pid_t curr_pid;

	GT_1trace(KFILE_debugMask, GT_ENTER, "KFILE_Close: hFile 0x%x\n",
		  hFile);

	/* Check for valid handle */
	if (MEM_IsValidHandle(hFile, SIGNATURE)) {
		/* Close file only if opened by the same process (id). Otherwise
		 Linux closes all open file handles when process exits.*/
		PRCS_GetCurrentHandle((VOID **)&curr_pid);
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
		if (hFile->owner_pid == curr_pid) {
			fResult = sys_close(hFile->hInternal);
			if (fResult < 0) {
				cRetVal = E_KFILE_ERROR;
				GT_1trace(KFILE_debugMask, GT_6CLASS,
					  "KFILE_Close: sys_close "
					  "returned %d\n", fResult);
			}
		}
#else
		fRetVal = filp_close(hFile->fileDesc, NULL) ;
		if (fRetVal) {
			cRetVal = E_KFILE_ERROR;
			GT_1trace(KFILE_debugMask, GT_6CLASS,
				  "KFILE_Close: sys_close "
				  "returned %d\n", fRetVal);
		}
#endif
		MEM_FreeObject(hFile);
	} else {
		cRetVal = E_KFILE_INVALIDHANDLE;
		GT_0trace(KFILE_debugMask, GT_6CLASS, "KFILE_Close: "
			  "invalid file handle\n");
	}
	return (cRetVal);
}

/*
 *  ======== KFILE_Exit ========
 *  Purpose:
 *      Decrement reference count, and free resources when reference count
 *      is 0.
 */
VOID KFILE_Exit()
{
	GT_0trace(KFILE_debugMask, GT_5CLASS, "KFILE_Exit\n");
}

/*
 *  ======== KFILE_Init ========
 */
BOOL KFILE_Init()
{
	GT_create(&KFILE_debugMask, "KF");	/* "KF" for KFile */

	GT_0trace(KFILE_debugMask, GT_5CLASS, "KFILE_Init\n");

	return (TRUE);
}

/*
 *  ======== KFILE_Open ========
 *  Purpose:
 *      Open a file for reading ONLY
 */
struct KFILE_FileObj *KFILE_Open(CONST CHAR *pszFileName, CONST CHAR *pszMode)
{
	struct KFILE_FileObj *hFile;	/* file handle */
	long hTempFile;		/* tmp internal handle */
	DSP_STATUS status;
	mm_segment_t fs;
	UINT	length = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	long fileDesc;
#else
	struct file*fileDesc = NULL;

#endif

	DBC_Require(pszMode != NULL);
	DBC_Require(pszFileName != NULL);

	GT_2trace(KFILE_debugMask, GT_ENTER,
		  "KFILE_Open: pszFileName %s, pszMode "
		  "%s\n", pszFileName, pszMode);

	/* create a KFILE object */
	MEM_AllocObject(hFile, struct KFILE_FileObj, SIGNATURE);

	if (hFile) {
		fs = get_fs();
		set_fs(get_ds());
	/* Third argument is mode (permissions). Ignored unless creating file */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
		hTempFile = sys_open(pszFileName, O_RDONLY, 0);
		if (hTempFile < 0) {
#else
		fileDesc = filp_open(pszFileName, O_RDONLY, 0);
		if ((IS_ERR(fileDesc)) || (fileDesc == NULL) ||
		     (fileDesc->f_op == NULL) || (fileDesc->f_op->read == NULL)
		     || (fileDesc->f_op->llseek == NULL)) {
#endif
			status = DSP_EFILE;
		} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			hFile->hInternal = hTempFile;
#else
			hFile->fileDesc = fileDesc;
			hFile->fileName = pszFileName;
			hFile->isOpen	   = TRUE;
			hFile->curPos   = 0;
			hFile->size = fileDesc->f_op->llseek(fileDesc, 0,
							    SEEK_END);
			fileDesc->f_op->llseek(fileDesc, 0, SEEK_SET);
#endif
			PRCS_GetCurrentHandle((VOID **) &hFile->owner_pid);
			status = DSP_SOK;
		}
		set_fs(fs);
		if (!DSP_SUCCEEDED(status)) {
			/* free memory, and clear handle */
			MEM_FreeObject(hFile);
			hFile = NULL;
		}
	} else {
			GT_0trace(KFILE_debugMask, GT_6CLASS,
				  "KFILE_Open: MEM_AllocObject failed\n");
			status = DSP_EMEMORY;
	}
	return (hFile);
}

/*
 *  ======== KFILE_Read ========
 *  Purpose:
 *      Reads a specified number of bytes into a buffer.
 */
INT
KFILE_Read(VOID *pBuffer, INT cSize, INT cCount, struct KFILE_FileObj *hFile)
{
	DWORD dwBytesRead = 0;
	INT cRetVal = 0;
	mm_segment_t fs;

	DBC_Require(pBuffer != NULL);

	GT_4trace(KFILE_debugMask, GT_4CLASS,
		  "KFILE_Read: buffer 0x%x, cSize 0x%x,"
		  "cCount 0x%x, hFile 0x%x\n", pBuffer, cSize, cCount, hFile);

	/* check for valid file handle */
	if (MEM_IsValidHandle(hFile, SIGNATURE)) {
		if ((cSize > 0) && (cCount > 0) && pBuffer) {
			/* read from file */
			fs = get_fs();
			set_fs(get_ds());
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			dwBytesRead =  sys_read(hFile->hInternal, pBuffer,
				       cSize * cCount);
#else
			dwBytesRead = hFile->fileDesc->f_op->read(hFile->
				      fileDesc, pBuffer, cSize *cCount,
				      &(hFile->fileDesc->f_pos));
#endif
			set_fs(fs);
			if (dwBytesRead) {
				cRetVal = dwBytesRead / cSize;
				hFile->curPos += dwBytesRead;
				DBC_Assert((dwBytesRead / cSize) <= \
					  (UINT)cCount);
			} else {
				cRetVal = E_KFILE_ERROR;
				GT_0trace(KFILE_debugMask, GT_6CLASS,
					  "KFILE_Read: sys_read() failed\n");
			}
		} else {
			cRetVal = DSP_EINVALIDARG;
			GT_0trace(KFILE_debugMask, GT_6CLASS,
				  "KFILE_Read: Invalid argument(s)\n");
		}
	} else {
		cRetVal = E_KFILE_INVALIDHANDLE;
		GT_0trace(KFILE_debugMask, GT_6CLASS,
			  "KFILE_Read: invalid file handle\n");
	}

	return (cRetVal);
}

/*
 *  ======== KFILE_Seek ========
 *  Purpose:
 *      Sets the file position indicator. NOTE:  we don't support seeking
 *      beyond the boundaries of a file.
 */
INT KFILE_Seek(struct KFILE_FileObj *hFile, LONG lOffset, INT cOrigin)
{
	INT cRetVal = 0;	/* 0 for success */
	DWORD dwCurPos = 0;
	DSP_STATUS status = DSP_SOK;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
	struct file *fileDesc = NULL;
#endif


	GT_3trace(KFILE_debugMask, GT_ENTER, "KFILE_Seek: hFile 0x%x, "
		  "lOffset 0x%x, cOrigin 0x%x\n",
		  hFile, lOffset, cOrigin);

	/* check for valid file handle */
	if (MEM_IsValidHandle(hFile, SIGNATURE)) {
		/* based on the origin flag, move the internal pointer */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
	fileDesc = hFile->fileDesc;
#endif
		switch (cOrigin) {
		case KFILE_SEEK_SET:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			dwCurPos = sys_lseek(hFile->hInternal, lOffset,
					     SEEK_SET);
#else
			dwCurPos = hFile->fileDesc->f_op->llseek(hFile->
				   fileDesc, lOffset, SEEK_SET);
#endif
			cRetVal = ((dwCurPos >= 0) ? 0 : E_KFILE_ERROR);
			break;

		case KFILE_SEEK_CUR:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			dwCurPos = sys_lseek(hFile->hInternal, lOffset,
					     SEEK_CUR);
#else
			dwCurPos = hFile->fileDesc->f_op->llseek(hFile->
				   fileDesc, lOffset, SEEK_CUR);
#endif
			cRetVal = ((dwCurPos >= 0) ? 0 : E_KFILE_ERROR);
			break;
		case KFILE_SEEK_END:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			dwCurPos = sys_lseek(hFile->hInternal, lOffset,
					     SEEK_END);
#else
			dwCurPos = hFile->fileDesc->f_op->llseek(hFile->
				   fileDesc, lOffset, SEEK_END);
#endif
			cRetVal = ((dwCurPos >= 0) ? 0 : E_KFILE_ERROR);
			break;
		default:
			cRetVal = E_KFILE_BADORIGINFLAG;
			GT_0trace(KFILE_debugMask, GT_6CLASS,
				  "KFILE_Seek:bad origin flag\n");
			break;
		}
	} else {
		cRetVal = E_KFILE_INVALIDHANDLE;
		GT_0trace(KFILE_debugMask, GT_6CLASS,
			  "KFILE_Seek:invalid file handle\n");
	}
	return (cRetVal);
}

/*
 *  ======== KFILE_Tell ========
 *  Purpose:
 *      Reports the current value of the position indicator. We did not
 *	    consider 64 bit long file size, which implies a 4GB file limit
 *      (2 to 32 power).
 */
LONG KFILE_Tell(struct KFILE_FileObj *hFile)
{
	DWORD dwCurPos = 0;
	LONG lRetVal = E_KFILE_ERROR;

	GT_1trace(KFILE_debugMask, GT_ENTER, "KFILE_Tell: hFile 0x%x\n", hFile);

	if (MEM_IsValidHandle(hFile, SIGNATURE)) {

		/* Get current position. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
		dwCurPos = sys_lseek(hFile->hInternal, 0, SEEK_CUR);
#else
		dwCurPos = hFile->fileDesc->f_op->llseek(hFile->fileDesc, 0,
			   SEEK_CUR);
#endif
		if (dwCurPos >= 0) {
			lRetVal = dwCurPos;
		}
	} else {
		lRetVal = E_KFILE_INVALIDHANDLE;
		GT_0trace(KFILE_debugMask, GT_6CLASS,
			  "KFILE_Seek:invalid file handle\n");
	}
	return (lRetVal);
}

