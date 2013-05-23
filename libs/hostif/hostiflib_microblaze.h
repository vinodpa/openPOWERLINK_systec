/**
********************************************************************************
\file   hostiflib_microblaze.h

\brief  Host Interface Library - For Microblaze target

This header file provides specific macros for Xilinx Microblaze CPU.

*******************************************************************************/
#ifndef _INC_HOST_IF_MICROBLAZE_H_
#define _INC_HOST_IF_MICROBLAZE_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <unistd.h>
#include <stddef.h>
#include <xil_types.h>
#include <xil_cache.h>
#include "xintc_l.h"
#include <xilinx_usleep.h>
#include <xil_io.h>
#include <xparameters.h>
// include section header file for special functions in
// tightly-coupled memory
#include <section-microblaze.h>

// include generated header file for memory structure and version filed
#include "hostiflib-zynqmem.h"

#if defined(XPAR_HOSTINTERFACE_0_BASE)

#define HOSTIF_PCP_BASE
#define HOSTIF_HOST_BASE

#define HOSTIF_IRQ_IC_ID
#define HOSTIF_IRQ

#elif (defined(PCP_0_HOSTINTERFACE_0_PCP_BASE) && \
       defined(PCP_0_HOSTINTERFACE_0_HOST_BASE))

#define HOSTIF_PCP_BASE
#define HOSTIF_HOST_BASE

#else

#warning "Host Interface base is assumed! Set the correct address!"

#define HOSTIF_PCP_BASE             0x10000000
#define HOSTIF_HOST_BASE            0x10000000
#define HOSTIF_IRQ_IC_ID            0
#define HOSTIF_IRQ                  0

#endif


#define HOSTIF_MAKE_NONCACHEABLE(ptr) ptr

#define HOSTIF_UNCACHED_MALLOC(size) 	    malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)		    free(ptr)

/// sleep
#define HOSTIF_USLEEP(x)					usleep(x)

/// hw access
#define HOSTIF_RD32(base, offset) 			Xil_In32(base+offset)
#define HOSTIF_RD16(base, offset)			Xil_In16(base+offset)
#define HOSTIF_RD8(base, offset)			Xil_In8(base+offset)

#define HOSTIF_WR32(base, offset, dword)	Xil_Out32((base+offset),dword)
#define HOSTIF_WR16(base, offset, word)		Xil_Out16((base+offset),word)
#define HOSTIF_WR8(base, offset, byte)		Xil_Out8((base+offset),byte)

#define HOSTIF_IRQ_REG(cb, arg)  hostiflib_RegisterHandler(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ,cb,arg)
#define HOSTIF_IRQ_ENABLE()		 XIntc_EnableIntr(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ) //FIXME: Change the base address and mask as required

#define HOSTIF_IRQ_DISABLE()	XIntc_DisableIntr(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ) //FIXME: Change the base address and mask as required


#endif /* _INC_HOST_IF_MICROBLAZE_H_ */
