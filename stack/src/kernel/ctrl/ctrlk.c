/**
********************************************************************************
\file   ctrlk.c

\brief  Implementation of kernel control module

This file contains the implementation of the kernel control module. The kernel
control module is responsible to execute commands from the user part of the
stack. Additionally it provides status information to the user part.

\ingroup module_ctrlk
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
#include <Epl.h>
#include <kernel/EplDllk.h>
#include <kernel/errhndk.h>
#include <kernel/eventk.h>
#include <kernel/EplNmtk.h>
#include <kernel/EplObdk.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <kernel/pdok.h>

#include <ctrl.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>

#if defined(CONFIG_INCLUDE_VETH)
#include <kernel/VirtualEthernet.h>
#endif

#if EPL_USE_SHAREDBUFF != FALSE
#include <SharedBuff.h>
#endif

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
\brief  Instance of kernel control module

The structure specifies the instance variable of the kernel control module.
*/
typedef struct
{
    tCtrlInitParam      initParam;           ///< initialization parameters
    UINT16              heartbeat;           ///< The heartbeat counter.
} tCtrlkInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlkInstance   instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel initStack(void);
static tEplKernel shutdownStack(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the kernel control module

The function initializes the kernel control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
tEplKernel ctrlk_init(void)
{
    tEplKernel      ret = kEplSuccessful;
#if EPL_USE_SHAREDBUFF != FALSE
    tShbError       shbError;

    shbError = ShbInit();
    if (shbError != kShbOk)
    {
        return kEplNoResource;
    }
#endif

    if ((ret = ctrlkcal_init()) != kEplSuccessful)
    {
        goto ExitCleanup;
    }

    // initialize heartbeat counter
    instance_l.heartbeat = 0;

    return kEplSuccessful;

ExitCleanup:
    ctrlkcal_exit();
#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel control module

The function cleans up the kernel control module.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
void ctrlk_exit(void)
{
    ctrlkcal_exit();
#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel control module

This function implements the main function of the control module. It processes
commands from the user part of the stack and executes them.

\return Returns the exit flag
\retval TRUE    If kernel stack should exit.
\retval FALSE   If kernel stack should continue running.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
BOOL ctrlk_process(void)
{
    tEplKernel          ret = kEplSuccessful;
    tEplKernel          fRet;
    UINT16              status;
    tCtrlCmdType        cmd = kCtrlNone;
    BOOL                fExit = FALSE;

    if (ctrlkcal_getCmd(&cmd) != kEplSuccessful)
    {
        TRACE ("%s: error getting command!\n", __func__);
        return FALSE;
    }

    if (cmd != kCtrlNone)
    {
        ret = ctrlk_executeCmd(cmd, &fRet, &status, &fExit);
        if (ret == kEplSuccessful)
        {
            ctrlkcal_sendReturn(fRet);
            ctrlkcal_setStatus(status);
        }
    }

    ret = ctrlkcal_process();
    if(ret != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: CAL process returned with 0x%X\n", __func__, ret);
        fExit = TRUE;
    }

    return fExit;
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command

The function executes a control command from the user part of the stack.
The return value of the executed function will be stored a \p pRet_p. If the
pointer to the status and exit flag is not NULL the appropriate data is stored.

\param  cmd_p               The command to be executed.
\param  pRet_p              Pointer to store the return value.
\param  pStatus_p           Pointer to store the kernel stack status. (if not NULL)
\param  pfExit_p            Pointer to store the exit flag. (if not NULL)

\return The function returns a tEplKernel error code.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
tEplKernel ctrlk_executeCmd(tCtrlCmdType cmd_p, tEplKernel* pRet_p, UINT16* pStatus_p,
                            BOOL* pfExit_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT16              status;
    BOOL                fExit;

    switch(cmd_p)
    {
        case kCtrlInitStack:
            TRACE ("Initialize kernel modules...\n");
            *pRet_p = initStack();
            status = kCtrlStatusRunning;
            fExit = FALSE;
            break;

        case kCtrlCleanupStack:
            TRACE ("Shutdown kernel modules...\n");
            *pRet_p = shutdownStack();
            status = kCtrlStatusReady;
            fExit = FALSE;
            break;

        case kCtrlShutdown:
            TRACE ("Shutdown kernel stack...\n");
            *pRet_p = shutdownStack();
            status = kCtrlStatusUnavailable;
            fExit = TRUE;
            break;

        default:
            TRACE ("Unknown command\n");
            ret = kEplGeneralError;
            status = kCtrlStatusUnavailable;
            fExit = TRUE;
            break;
    }

    if (pStatus_p != NULL)
    {
        *pStatus_p = status;
    }

    if (pfExit_p != NULL)
    {
        *pfExit_p = fExit;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update heartbeat counter

The function updates the heartbeat counter of the kernel stack which could be
used by the user stack to detect if the kernel stack is still running. It has
to be called periodically, e.g. from a timer routine.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
void ctrlk_updateHeartbeat(void)
{
    UINT16      heartbeat;

    heartbeat = instance_l.heartbeat++;
    ctrlkcal_updateHeartbeat(heartbeat);
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Initialize the kernel stack modules

The function initializes the kernel stack modules.

\param  pInitParam_p        Pointer to stack initialization parameters.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initStack(void)
{
    tEplKernel          ret;
    tEplDllkInitParam   dllkInitParam;

    ctrlkcal_readInitParam(&instance_l.initParam);

    if ((ret = eventk_init()) != kEplSuccessful)
        return ret;

#if defined(CONFIG_INCLUDE_NMTK)
    if ((ret = EplNmtkInit()) != kEplSuccessful)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_DLLK)
    EPL_MEMCPY(dllkInitParam.m_be_abLocalMac, instance_l.initParam.aMacAddress, 6);
    dllkInitParam.m_HwParam.m_pszDevName = instance_l.initParam.szEthDevName;
    dllkInitParam.m_HwParam.m_uiDevNumber = instance_l.initParam.ethDevNumber;

    ret = EplDllkAddInstance(&dllkInitParam);
    if (ret != kEplSuccessful)
        return ret;

    // copy MAC address back to instance structure
    EPL_MEMCPY(instance_l.initParam.aMacAddress, dllkInitParam.m_be_abLocalMac, 6);
    ctrlkcal_storeInitParam(&instance_l.initParam);

    EplDllkRegSyncHandler(pdok_sendSyncEvent);

    // initialize EplDllkCal module
    if ((ret = dllkcal_init()) != kEplSuccessful)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_PDOK)
    if ((ret = pdok_init()) != kEplSuccessful)
        return ret;
#endif

    ret = errhndk_init();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Cleanup the kernel stack modules

The function cleans up the kernel stack modules

\return Returns always kEplSuccessful
*/
//------------------------------------------------------------------------------
static tEplKernel shutdownStack(void)
{
#if defined(CONFIG_INCLUDE_PDOK)
    pdok_exit();
#endif

#if defined(CONFIG_INCLUDE_NMTK)
    EplNmtkDelInstance();
#endif

#if defined(CONFIG_INCLUDE_DLLK)
    EplDllkDelInstance();

    dllkcal_exit();
#endif

    eventk_exit();
    errhndk_exit();

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/// \}
//------------------------------------------------------------------------------

