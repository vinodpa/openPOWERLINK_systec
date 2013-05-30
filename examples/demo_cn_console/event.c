/**
********************************************************************************
\file   event.c

\brief  CN Application event handler

This file contains a demo CN application event handler.

\ingroup module_demo
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <Epl.h>
#include <console/console.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static BOOL*    pfGsOff_l;

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
static tEplKernel processStateChangeEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p);

static tEplKernel processErrorWarningEvent(tEplApiEventType EventType_p,
                                           tEplApiEventArg* pEventArg_p,
                                           void GENERIC* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param  pfGsOff_p               Pointer to GsOff flag (determines that stack is down)

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void initEvents (BOOL* pfGsOff_p)
{
    pfGsOff_l = pfGsOff_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the applications stack event handler.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC processEvents(tEplApiEventType EventType_p,
                                tEplApiEventArg* pEventArg_p,
                                void GENERIC* pUserArg_p)
{
    tEplKernel          ret = kEplSuccessful;

    UNUSED_PARAMETER(pUserArg_p);

    switch (EventType_p)
    {
        case kEplApiEventNmtStateChange:
            ret = processStateChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventCriticalError:
        case kEplApiEventWarning:
            ret = processErrorWarningEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        default:
            break;
    }
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process state change events

The function processes state change events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateChangeEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tEplEventNmtStateChange*    pNmtStateChange = &pEventArg_p->m_NmtStateChange;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        return kEplGeneralError;
    }

    switch (pNmtStateChange->m_NewNmtState)
    {
        case kEplNmtGsOff:
            // NMT state machine was shut down,
            ret = kEplShutdown;

            console_printlog("StateChangeEvent:kEplNmtGsOff originating event = 0x%X (%s)\n",
                     pNmtStateChange->m_NmtEvent,
                     EplGetNmtEventStr(pNmtStateChange->m_NmtEvent));

            // signal that stack is off
            *pfGsOff_l = FALSE;
            break;

        case kEplNmtGsInitialising:
        case kEplNmtGsResetApplication:
        case kEplNmtGsResetConfiguration:
        case kEplNmtGsResetCommunication:
        case kEplNmtCsPreOperational1:
        case kEplNmtCsPreOperational2:
        case kEplNmtCsOperational:
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->m_NewNmtState,
                   pNmtStateChange->m_NmtEvent,
                   EplGetNmtEventStr(pNmtStateChange->m_NmtEvent));
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processErrorWarningEvent(tEplApiEventType EventType_p,
                                           tEplApiEventArg* pEventArg_p,
                                           void GENERIC* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    tEplEventError*         pInternalError = &pEventArg_p->m_InternalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("Err/Warn: Source = %s (%02X) EplError = %s (0x%03X)\n",
                EplGetEventSourceStr(pInternalError->m_EventSource),
                pInternalError->m_EventSource,
                EplGetEplKernelStr(pInternalError->m_EplError),
                pInternalError->m_EplError);

    FTRACE_MARKER("Err/Warn: Source = %s (%02X) EplError = %s (0x%03X)\n",
                EplGetEventSourceStr(pInternalError->m_EventSource),
                pInternalError->m_EventSource,
                EplGetEplKernelStr(pInternalError->m_EplError),
                pInternalError->m_EplError);

    // check additional argument
    switch (pInternalError->m_EventSource)
    {
        case kEplEventSourceEventk:
        case kEplEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            console_printlog(" OrgSource = %s %02X\n",
                     EplGetEventSourceStr(pInternalError->m_Arg.m_EventSource),
                     pInternalError->m_Arg.m_EventSource);

            FTRACE_MARKER(" OrgSource = %s %02X\n",
                     EplGetEventSourceStr(pInternalError->m_Arg.m_EventSource),
                     pInternalError->m_Arg.m_EventSource);
            break;

        case kEplEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            console_printlog(" val = %X\n", pInternalError->m_Arg.m_dwArg);
            FTRACE_MARKER(" val = %X\n", pInternalError->m_Arg.m_dwArg);
            break;

        default:
            console_printlog("\n");
            break;
    }
    return kEplSuccessful;
}

///\}




