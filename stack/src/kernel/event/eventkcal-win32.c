/**
********************************************************************************
\file   eventkcal-win32.c

\brief  Kernel event CAL module using circular buffers on Windows

This kernel event CAL module implementation uses circular buffers on Windows.

\ingroup module_eventkcal
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
#include <EplInc.h>
#include <Epl.h>
#include <eventcal.h>
#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>
#include <circbuffer.h>

#include <Windows.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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
/**
\brief Kernel event CAL instance type

The structure contains all necessary information needed by the kernel event
CAL module.
*/
typedef struct
{
    HANDLE                  threadHandle;
    HANDLE                  semUserData;
    HANDLE                  semKernelData;
    BOOL                    fInitialized;
    BOOL                    fStopThread;
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg);
static void signalKernelEvent(void);
static void signalUserEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module on Windows.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_init (void)
{
    tEplKernel      ret = kEplSuccessful;

    EPL_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    if ((instance_l.semUserData = CreateSemaphore(NULL, 0, 100, "Local\\semUserEvent")) == NULL)
        goto Exit;

    if ((instance_l.semKernelData = CreateSemaphore(NULL, 0, 100, "Local\\semKernelEvent")) == NULL)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kEplSuccessful)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kEplSuccessful)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kEplSuccessful)
        goto Exit;

    eventkcal_setSignalingCircbuf(kEventQueueK2U, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueKInt, signalKernelEvent);

    instance_l.fStopThread = FALSE;
    if ((instance_l.threadHandle = CreateThread (NULL, 0, eventThread, (LPVOID)&instance_l,
                                                 0, NULL)) == NULL)
        goto Exit;

    SetThreadPriority(instance_l.threadHandle, THREAD_PRIORITY_ABOVE_NORMAL);

    instance_l.fInitialized = TRUE;
    return kEplSuccessful;

Exit:

    if (instance_l.semKernelData != NULL)
        CloseHandle(instance_l.semKernelData);

    if (instance_l.semUserData != NULL)
        CloseHandle(instance_l.semUserData);

    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);

    return kEplNoResource;

}


//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_exit (void)
{
    if (instance_l.fInitialized == TRUE)
    {
        instance_l.fStopThread = TRUE;

        // jba wait for thread to exit!

        eventkcal_exitQueueCircbuf(kEventQueueK2U);
        eventkcal_exitQueueCircbuf(kEventQueueU2K);
        eventkcal_exitQueueCircbuf(kEventQueueUInt);

        CloseHandle(instance_l.semKernelData);
        CloseHandle(instance_l.semUserData);
    }
    instance_l.fInitialized = FALSE;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to the kernel queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postKernelEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to the user queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postUserEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel CAL module

This function will be called by the systems process function.
*/
//------------------------------------------------------------------------------
void eventkcal_process(void)
{
    // Nothing to do, because we use threads
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Event handler thread function

This function contains the main function for the event handler thread.

\param  arg                     Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg)
{
    tEventkCalInstance*     pInstance = (tEventkCalInstance*)arg;
    DWORD                    waitResult;

    TRACE("Kernel event thread %d waiting for events...\n", GetCurrentThreadId());
    while (!pInstance->fStopThread)
    {
        waitResult = WaitForSingleObject (pInstance->semKernelData, 100);
        switch (waitResult)
        {
            case WAIT_OBJECT_0:
                /* first handle kernel internal events --> higher priority! */
                if (eventkcal_getEventCountCircbuf(kEventQueueKInt) > 0)
                {
                    eventkcal_processEventCircbuf(kEventQueueKInt);
                }
                else
                {
                    if (eventkcal_getEventCountCircbuf(kEventQueueU2K) > 0)
                    {
                        eventkcal_processEventCircbuf(kEventQueueU2K);
                    }
                }
                break;

            case WAIT_TIMEOUT:
                //TRACE("kernel event timeout!\n");
                break;

            default:
                TRACE("%s() Semaphore wait unknown error! Error:%ld\n", __func__, GetLastError());
                break;
        }
    }
    TRACE("Kernel Event Thread is exiting!\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
void signalUserEvent(void)
{
    ReleaseSemaphore(instance_l.semUserData, 1, NULL);
}

//------------------------------------------------------------------------------
/**
\brief  Signal a kernel event

This function signals that a kernel event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
void signalKernelEvent(void)
{
    ReleaseSemaphore(instance_l.semKernelData, 1, NULL);
}

/// \}
