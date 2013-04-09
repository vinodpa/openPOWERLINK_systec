/**
********************************************************************************
\file   target-nios2.c

\brief  target specific functions for Nios II without OS

This target depending module provides several functions that are necessary for
systems without shared buffer and any OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <global.h>
#include <unistd.h>
#include <sys/alt_irq.h>
#include <sys/alt_alarm.h>
#include <Epl.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x*1000U)

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

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

    dwTicks = alt_nticks();

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
static alt_irq_context  irq_context = 0;
static int              iLockCount = 0;

    if (fEnable_p != FALSE)
    {   // restore interrupts
        if (--iLockCount == 0)
        {
            alt_irq_enable_all(irq_context);
        }
    }
    else
    {   // disable interrupts
        if (iLockCount == 0)
        {
            irq_context = alt_irq_disable_all();
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
    // For now, the global interrupt enable flag is checked.

    if (alt_irq_enabled() == 0)
    {
        return TRUE;
    }
    else
    {
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
