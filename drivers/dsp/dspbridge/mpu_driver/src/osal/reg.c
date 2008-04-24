/*
 * dspbridge/src/osal/linux/reg.c
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
 *  ======== regce.c ========
 *  Purpose:
 *      Provide registry functions.
 *
 *  Public Functions:
 *      REG_DeleteValue
 *      REG_EnumKey
 *      REG_EnumValue
 *      REG_Exit
 *      REG_GetValue
 *      REG_Init
 *      REG_QueryInfoKey
 *      REG_SetValue
 *
 *! Revision History:
 *! ================
 *
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

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <mem.h>

/*  ----------------------------------- Others */
#include <dbreg.h>

/*  ----------------------------------- This */
#include <reg.h>
#include <regsup.h>

#if GT_TRACE
struct GT_Mask REG_debugMask = { 0, 0 };	/* GT trace var. */
#endif

/*
 *  ======== REG_DeleteValue ========
 *  Deletes a registry entry value.  NOTE:  A registry entry value is not the
 *  same as *  a registry key.
 */
DSP_STATUS REG_DeleteValue(OPTIONAL IN HANDLE *phKey, IN CONST PSTR pstrSubkey,
			   IN CONST PSTR pstrValue)
{
	DSP_STATUS status;
	DBC_Require(pstrSubkey && pstrValue);
	DBC_Require(phKey == NULL);
	DBC_Require(CSL_Strlen(pstrSubkey) < REG_MAXREGPATHLENGTH);
	DBC_Require(CSL_Strlen(pstrValue) < REG_MAXREGPATHLENGTH);

	GT_0trace(REG_debugMask, GT_ENTER, "REG_DeleteValue: entered\n");

	/*  Note that we don't use phKey */
	if (regsupDeleteValue(pstrSubkey, pstrValue) == DSP_SOK)
		status = DSP_SOK;
	else
		status = DSP_EFAIL;

	return (status);
}

#ifndef LINUX
/*
 *  ======== REG_EnumKey ========
 *  Enumerates subkeys of a specified registry key.
 */
DSP_STATUS REG_EnumKey(OPTIONAL IN HANDLE *phKey, IN DWORD dwIndex,
		       IN CONST PSTR pstrKey, IN OUT PSTR pstrSubkey,
		       IN OUT DWORD *pdwValueSize)
{
	DBC_Require(pstrKey && pstrSubkey && pdwValueSize);
	DBC_Require(phKey == NULL);
	DBC_Require(CSL_Strlen(pstrKey) < REG_MAXREGPATHLENGTH);

	GT_0trace(REG_debugMask, GT_ENTER, "REG_EnumKey: entered\n");

	return (DSP_ENOTIMPL);
}
#endif

/*
 *  ======== REG_EnumValue ========
 *  Enumerates a registry key and retrieve values stored under the key.
 *  We will assume the input pdwValueSize is smaller than
 *  REG_MAXREGPATHLENGTH for implementation purposes.
 */
DSP_STATUS REG_EnumValue(IN HANDLE *phKey, IN DWORD dwIndex,
			 IN CONST PSTR pstrKey, IN OUT PSTR pstrValue,
			 IN OUT DWORD *pdwValueSize, IN OUT PSTR pstrData,
			 IN OUT DWORD *pdwDataSize)
{
	DSP_STATUS status;

	DBC_Require(pstrKey && pstrValue && pdwValueSize && pstrData &&
		    pdwDataSize);
	DBC_Require(*pdwValueSize <= REG_MAXREGPATHLENGTH);
	DBC_Require(phKey == NULL);
	DBC_Require(CSL_Strlen(pstrKey) < REG_MAXREGPATHLENGTH);

	GT_0trace(REG_debugMask, GT_ENTER, "REG_EnumValue: entered\n");

	status = regsupEnumValue(dwIndex, pstrKey, pstrValue, pdwValueSize,
				 pstrData, pdwDataSize);

	return (status);
}

/*
 *  ======== REG_Exit ========
 *  Discontinue usage of the REG module.
 */
void REG_Exit()
{
	GT_0trace(REG_debugMask, GT_5CLASS, "REG_Exit\n");

	regsupExit();
}

/*
 *  ======== REG_GetValue ========
 *  Retrieve a value from the registry.
 */
DSP_STATUS REG_GetValue(OPTIONAL IN HANDLE *phKey, IN CONST PSTR pstrSubkey,
			IN CONST PSTR pstrValue, OUT BYTE *pbData,
			IN OUT DWORD *pdwDataSize)
{
	DSP_STATUS status;

	DBC_Require(pstrSubkey && pstrValue && pbData);
	DBC_Require(phKey == NULL);
	DBC_Require(CSL_Strlen(pstrSubkey) < REG_MAXREGPATHLENGTH);
	DBC_Require(CSL_Strlen(pstrValue) < REG_MAXREGPATHLENGTH);

	GT_0trace(REG_debugMask, GT_ENTER, "REG_GetValue: entered\n");

	/*  We need to use regsup calls...  */
	/*  ...for now we don't need the key handle or  */
	/*  the subkey, all we need is the value to lookup.  */
	if (regsupGetValue(pstrValue, pbData, pdwDataSize) == DSP_SOK)
		status = DSP_SOK;
	else
		status = DSP_EFAIL;

	return (status);
}

/*
 *  ======== REG_Init ========
 *  Initialize the REG module's private state.
 */
BOOL REG_Init()
{
	BOOL fInit;

	GT_create(&REG_debugMask, "RG");	/* RG for ReG */

	fInit = regsupInit();

	GT_0trace(REG_debugMask, GT_5CLASS, "REG_Init\n");

	return (fInit);
}

/*
 *  ======== REG_SetValue ========
 *  Set a value in the registry.
 */
DSP_STATUS REG_SetValue(OPTIONAL IN HANDLE *phKey, IN CONST PSTR pstrSubkey,
			IN CONST PSTR pstrValue, IN CONST DWORD dwType,
			IN BYTE *pbData, IN DWORD dwDataSize)
{
	DSP_STATUS status;

	DBC_Require(pstrValue && pbData);
	DBC_Require(phKey == NULL);
	DBC_Require(dwDataSize > 0);
	DBC_Require(CSL_Strlen(pstrValue) < REG_MAXREGPATHLENGTH);

	/*  We need to use regsup calls...  */
	/*  ...for now we don't need the key handle or  */
	/*  the subkey, all we need is the value to lookup.  */
	if (regsupSetValue(pstrValue, pbData, dwDataSize) == DSP_SOK)
		status = DSP_SOK;
	else
		status = DSP_EFAIL;

	return (status);
}

