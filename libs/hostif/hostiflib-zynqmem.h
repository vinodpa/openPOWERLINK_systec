/**
********************************************************************************
\file       /libs/hostif/hostiflib-zynqmem

\brief      Hostif interface memory offsets and size 

This module contains of platform specific definitions.

Copyright © 2011 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
All rights reserved. All use of this software and documentation is
subject to the License Agreement located at the end of this file below.

*******************************************************************************/

/*******************************************************************************/


#ifndef __HOSTIF_MEM_H__
#define __HOSTIF_MEM_H__

/* VERSION */
#define HOSTIF_VERSION_MAJOR 0
#define HOSTIF_VERSION_MINOR 0
#define HOSTIF_VERSION_REVISION 1
#define HOSTIF_VERSION_COUNT 44

/* BASE */
#define HOSTIF_BASE_DYNBUF0 2048
#define HOSTIF_BASE_DYNBUF1 4096
#define HOSTIF_BASE_ERRORCOUNTER 6144
#define HOSTIF_BASE_TXNMTQ 9252
#define HOSTIF_BASE_TXGENQ 11316
#define HOSTIF_BASE_TXSYNCQ 13380
#define HOSTIF_BASE_TXVETHQ 15444
#define HOSTIF_BASE_RXVETHQ 17508
#define HOSTIF_BASE_K2UQ 18548
#define HOSTIF_BASE_U2KQ 26756
#define HOSTIF_BASE_TPDO 34964
#define HOSTIF_BASE_RPDO 35988

/* SIZE */
#define HOSTIF_SIZE_DYNBUF0 2048
#define HOSTIF_SIZE_DYNBUF1 2048
#define HOSTIF_SIZE_ERRORCOUNTER 3108
#define HOSTIF_SIZE_TXNMTQ 2064
#define HOSTIF_SIZE_TXGENQ 2064
#define HOSTIF_SIZE_TXSYNCQ 2064
#define HOSTIF_SIZE_TXVETHQ 2064
#define HOSTIF_SIZE_RXVETHQ 1040
#define HOSTIF_SIZE_K2UQ 8208
#define HOSTIF_SIZE_U2KQ 8208
#define HOSTIF_SIZE_TPDO 1024
#define HOSTIF_SIZE_RPDO 1024

#endif
