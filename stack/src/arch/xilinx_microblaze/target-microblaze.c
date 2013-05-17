/*
 * target-microblaze.c
 *
 *  Created on: May 17, 2013
 *      Author: Gaurav Kumar Singh
 */

#include "global.h"
#include "Benchmark.h"

#include "xparameters.h"
#include "xilinx_irq.h"
#include "xilinx_usleep.h"
#include <Epl.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x*1000U)
//------------------------------------------------------------------------------
/**
\brief    returns current system tick

This function returns the currect system tick determined by the system timer.

\return system tick
\retval DWORD

\ingroup module_target
*/
//------------------------------------------------------------------------------
DWORD PUBLIC EplTgtGetTickCountMs (void)
{
	DWORD dwTicks;

    //FIXME: Find another way to generate a system tick...
    dwTicks = getMSCount();

    return dwTicks;
}
//------------------------------------------------------------------------------
/**
\brief    enables global interrupt

This function enabels/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void PUBLIC EplTgtEnableGlobalInterrupt (BYTE fEnable_p)
{
	static int              iLockCount = 0;

	    if (fEnable_p != FALSE)
	    {   // restore interrupts

	        if (--iLockCount == 0)
	        {
	            enableInterruptMaster();
	        }
	    }
	    else
	    {   // disable interrupts

	        if (iLockCount == 0)
	        {
	            disableInterruptMaster();
	        }
	        iLockCount++;
	    }

}
//------------------------------------------------------------------------------
/**
\brief    checks if CPU is in interrupt context

This function obtains if the CPU is in interrupt context.

\return CPU in interrupt context
\retval TRUE                    CPU is in interrupt context
\retval FALSE                   CPU is NOT in interrupt context

\ingroup module_target
*/
//------------------------------------------------------------------------------
BYTE PUBLIC EplTgtIsInterruptContext (void)
{
	// No real interrupt context check is performed.
	    // This would be possible with a flag in the ISR, only.
	    // For now, simply return ME.

	    DWORD dwGIE;

	    dwGIE = Xil_In32(XPAR_PCP_INTC_BASEADDR + XIN_MER_OFFSET) & \
	            XIN_INT_MASTER_ENABLE_MASK;

	    if(dwGIE == 0)
	    {
	        //master enable is off
	        return TRUE;
	    }
	    else
	    {
	        //master enable is on
	        return FALSE;
	    }
}
//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_init(void)
{
    return kEplSuccessful;
}
//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel target_cleanup(void)
{
    return kEplSuccessful;
}
//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  mseconds_p              Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep (unsigned int milliSecond_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSecond_p));
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

