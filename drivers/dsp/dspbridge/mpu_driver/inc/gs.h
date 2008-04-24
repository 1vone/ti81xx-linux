/*
 * dspbridge/inc/gs.h
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
 *  ======== gs.h ========
 *  Memory allocation/release wrappers.  This module allows clients to
 *  avoid OS spacific issues related to memory allocation.  It also provides
 *  simple diagnostic capabilities to assist in the detection of memory
 *  leaks.
 *! Revision History
 *! ================
 */

#ifndef GS_
#define GS_

/*
 *  ======== GS_alloc ========
 *  Alloc size bytes of space.  Returns pointer to space
 *  allocated, otherwise NULL.
 */
extern Ptr GS_alloc(Uns size);

/*
 *  ======== GS_calloc ========
 *  Alloc size bytes of space and initialize the space to 0.
 *  Returns pointer to space allocated, otherwise NULL.
 */
#define GS_calloc(s)    GS_alloc(s)

/*
 *  ======== GS_clearerr ========
 *  Clear error latches in GS module
 */
extern Void GS_clearerr();

/*
 *  ======== GS_err ========
 *  Return TRUE if an error occured; i.e., is any allocation failed
 *  or GS_frees() mismatch occured.
 */
extern Bool GS_err();

/*
 *  ======== GS_exit ========
 *  Module exit.  Do not change to "#define GS_init()"; in
 *  some environments this operation must actually do some work!
 */
extern Void GS_exit(Void);

/*
 *  ======== GS_free ========
 *  Free space allocated by GS_alloc() or GS_calloc().
 */
extern Void GS_free(Ptr ptr);

/*
 *  ======== GS_frees ========
 *  Free space allocated by GS_alloc() or GS_calloc() and assert that
 *  the size of the allocation is size bytes.
 */
extern Void GS_frees(Ptr ptr, Uns size);

/*
 *  ======== GS_init ========
 *  Module initialization.  Do not change to "#define GS_init()"; in
 *  some environments this operation must actually do some work!
 */
extern Void GS_init(Void);

/*
 *  ======== GS_mark ========
 */
extern Ptr GS_mark(Void);

/*
 *  ======== GS_realloc ========
 *  Resize a currently allocated buffer and preserve the
 *  current buffers contents.  Returns NULL of reallocation is not
 *  possible, otherwise a pointer to the new buffer is returned.
 */
extern Ptr GS_realloc(Ptr ptr, Uns size);

/*
 *  ======== GS_release ========
 */
extern Void GS_release(Ptr mark);

/*
 *  ======== GS_size ========
 *  Return the total size (in bytes) of all outstanding
 *  allocations.
 */
extern Uns GS_size(Void);

#endif				/*GS_ */
