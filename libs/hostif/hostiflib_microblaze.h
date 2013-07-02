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

//TODO: Review
int hostiflib_RegisterHandler (u32 BaseAddress, int InterruptId,
	   XInterruptHandler Handler, void *CallBackRef);
void hostif_FlushDCacheRange(u32 dwAddr_p,u16 span_p);
void hostif_InvalidateDCacheRange(u32 dwAddr_p,u16 span_p);

#if defined(HOSTINTERFACE_0_BASE)

//TODO: Review
#define HOSTIF_PCP_BASE		HOSTINTERFACE_0_BASE
#define HOSTIF_HOST_BASE	HOSTINTERFACE_0_BASE

//TODO: Get IRQ ID from Interrupt Controller
//TODO: Any Intterupt is given to Microblaze from Host Interface ?
#define HOSTIF_IRQ_IC_ID	-1 //FIXME: @Vinod On Zynq Host is ARM
#define HOSTIF_IRQ			-1 //FIXME: @Vinod On Zynq Host is ARM

#elif (defined(PCP_0_HOSTINTERFACE_0_PCP_BASE) && \
       defined(PCP_0_HOSTINTERFACE_0_HOST_BASE))

//TODO: Review
#define HOSTIF_PCP_BASE		PCP_0_HOSTINTERFACE_0_PCP_BASE
#define HOSTIF_HOST_BASE	PCP_0_HOSTINTERFACE_0_HOST_BASE

#else

#warning "Host Interface base is assumed! Set the correct address!"

#define HOSTIF_PCP_BASE             0x10000000
#define HOSTIF_HOST_BASE            0x10000000
#define HOSTIF_IRQ_IC_ID            0
#define HOSTIF_IRQ                  0

#endif
#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define HOSTIF_USE_DCACHE			TRUE
#else
#define HOSTIF_USE_DCACHE			FALSE
#endif

#define HOSTIF_MAKE_NONCACHEABLE(ptr) 		(void *) ptr

#define HOSTIF_UNCACHED_MALLOC(size) 	    malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)		    free(ptr)

/// sleep
#define HOSTIF_USLEEP(x)					usleep(x)

/// hw access
#define HOSTIF_RD32(base, offset) 			MB_READ32((u32)(base), offset);
#define HOSTIF_RD16(base, offset)			MB_READ16((u32)(base), offset);
#define HOSTIF_RD8(base, offset)			MB_READ8((u32)(base), offset);

#define HOSTIF_WR32(base, offset, dword)	MB_WRITE32((u32)(base),offset,dword);
#define HOSTIF_WR16(base, offset, word)		MB_WRITE16((u32)(base),offset,word);
#define HOSTIF_WR8(base, offset, byte)		MB_WRITE8((u32)(base),offset,byte);



#define HOSTIF_IRQ_REG(cb, arg)  hostiflib_RegisterHandler(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ,cb,arg)
#define HOSTIF_IRQ_ENABLE()		 XIntc_EnableIntr(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ) //FIXME: Change the base address and mask as required

#define HOSTIF_IRQ_DISABLE()	XIntc_DisableIntr(HOSTIF_IRQ_IC_ID,HOSTIF_IRQ) //FIXME: Change the base address and mask as required




#endif /* _INC_HOST_IF_MICROBLAZE_H_ */
