/**
********************************************************************************
\file   dllucal.c

\brief  User DLL CAL module

This file contains the user DLL CAL mdodule

Copyright (c) 2012, SYSTEC electronik GmbH
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
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <dllcal.h>
#include <user/dllucal.h>
#include <user/eventu.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (EPL_DLL_PRES_CHAINING_MN != FALSE) \
    && (EPL_DLLCAL_TX_SYNC_QUEUE != EPL_QUEUE_SHB) \
    && (EPL_DLLCAL_TX_SYNC_QUEUE != EPL_QUEUE_HOSTINTERFACE)
#error "DLLCal module does not support direct calls with PRC MN"
#endif

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
typedef struct
{
    tEplDlluCbAsnd           apfnDlluCbAsnd[EPL_DLL_MAX_ASND_SERVICE_ID];
    tDllCalQueueInstance     dllCalQueueTxNmt;          ///< Dll Cal Queue instance for NMT priority
    tDllCalQueueInstance     dllCalQueueTxGen;          ///< Dll Cal Queue instance for Generic priority
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0) \
    && (EPL_DLL_PRES_CHAINING_MN != FALSE)
    tDllCalQueueInstance     dllCalQueueTxSync;         ///< Dll Cal Queue instance for Sync Request
    tDllCalFuncIntf*         pTxSyncFuncs;
#endif
    tDllCalFuncIntf*         pTxNmtFuncs;
    tDllCalFuncIntf*         pTxGenFuncs;
} tDlluCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// if no dynamic memory allocation shall be used
// define structures statically
static tDlluCalInstance     instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tEplKernel SetAsndServiceIdFilter(tEplDllAsndServiceId ServiceId_p,
                                         tEplDllAsndFilter Filter_p);


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize User DLL CAL module

This function initializes the user DLL CAL module.

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_init(void)
{
    tEplKernel      ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    instance_l.pTxNmtFuncs = GET_TX_NMT_INTERFACE();
    instance_l.pTxGenFuncs = GET_TX_GEN_INTERFACE();
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    instance_l.pTxSyncFuncs = GET_TX_SYNC_INTERFACE();
#endif

    ret = instance_l.pTxNmtFuncs->pfnAddInstance(&instance_l.dllCalQueueTxNmt,
                                                 kDllCalQueueTxNmt);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnAddInstance(&instance_l.dllCalQueueTxGen,
                                                 kDllCalQueueTxGen);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    ret = instance_l.pTxSyncFuncs->pfnAddInstance(&instance_l.dllCalQueueTxSync,
                                                  kDllCalQueueTxSync);

    if(ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Cleanup User DLL CAL module

This function cleans up the user DLL CAL module

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_exit(void)
{
    tEplKernel      ret = kEplSuccessful;

    instance_l.pTxNmtFuncs->pfnDelInstance(instance_l.dllCalQueueTxNmt);
    instance_l.pTxGenFuncs->pfnDelInstance(instance_l.dllCalQueueTxGen);
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    instance_l.pTxSyncFuncs->pfnDelInstance(instance_l.dllCalQueueTxSync);
#endif
    // reset instance structure
    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Process asynchronous frame event

The function processes an asynchronous frame event

\param  pEvent_p				Event to process

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_process(tEplEvent * pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplMsgType     msgType;
    UINT            asndServiceId;
    tEplFrameInfo   frameInfo;

    if (pEvent_p->m_EventType == kEplEventTypeAsndRx)
    {
        frameInfo.m_pFrame = (tEplFrame*) pEvent_p->m_pArg;
        frameInfo.m_uiFrameSize = pEvent_p->m_uiSize;

        msgType = (tEplMsgType)AmiGetByteFromLe(&frameInfo.m_pFrame->m_le_bMessageType);
        if (msgType != kEplMsgTypeAsnd)
        {
            ret = kEplInvalidOperation;
            goto Exit;
        }

        asndServiceId = (UINT) AmiGetByteFromLe(&frameInfo.m_pFrame->m_Data.m_Asnd.m_le_bServiceId);
        if (asndServiceId < EPL_DLL_MAX_ASND_SERVICE_ID)
        {   // ASnd service ID is valid
            if (instance_l.apfnDlluCbAsnd[asndServiceId] != NULL)
            {   // handler was registered
                ret = instance_l.apfnDlluCbAsnd[asndServiceId](&frameInfo);
            }
        }
    }
    else
    {
        ret = kEplInvalidEvent;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Configure DLL parameters

This function posts a DLL configuration event to the kernel DLL CAL module

\param  pDllConfigParam_p       Pointer to the DLL configuration parameters

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_config(tEplDllConfigParam * pDllConfigParam_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkConfig;
    event.m_pArg = pDllConfigParam_p;
    event.m_uiSize = sizeof (*pDllConfigParam_p);
    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Configure identity of local node

This function posts a dll identity event to the kernel DLL CAL module to
configure the identity of a local node for IdentResponse.

\param  pDllIdentParam_p        Pointer to ident parameters

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_setIdentity(tEplDllIdentParam * pDllIdentParam_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkIdentity;
    event.m_pArg = pDllIdentParam_p;
    event.m_uiSize = sizeof (*pDllIdentParam_p);
    ret = eventu_postEvent(&event);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Register ASnd handler

This function register the specified handler for the specified ASnd service
ID with the specified node ID filter.

\param  serviceId_p             ASnd service ID to register handler for.
\param  pfnDlluCbAsnd_p         Pointer to callback function.
\param  filter_p                Node filter ID.

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_regAsndService(tEplDllAsndServiceId serviceId_p,
                                    tEplDlluCbAsnd pfnDlluCbAsnd_p,
                                    tEplDllAsndFilter filter_p)
{
    tEplKernel  ret = kEplSuccessful;

    if (serviceId_p < tabentries (instance_l.apfnDlluCbAsnd))
    {
        // memorize function pointer
        instance_l.apfnDlluCbAsnd[serviceId_p] = pfnDlluCbAsnd_p;

        if (pfnDlluCbAsnd_p == NULL)
        {   // close filter
            filter_p = kEplDllAsndFilterNone;
        }

        // set filter in DLL module in kernel part
        ret = SetAsndServiceIdFilter(serviceId_p, filter_p);
    }
    else
    {
        ret = kEplDllInvalidAsndServiceId;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Send asynchronous frame

This function sends an asynchronous fram with the specified priority.

\param  pFrameInfo_p            Pointer to asynchronous frame. The frame size
                                includes the ethernet header (14 bytes).
\param  priority_p              Priority for sending this frame.

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_sendAsyncFrame(tEplFrameInfo * pFrameInfo_p,
                                  tEplDllAsyncReqPriority priority_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    switch (priority_p)
    {
        case kEplDllAsyncReqPrioNmt:
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxNmt,
                                        (BYTE*)pFrameInfo_p->m_pFrame,
                                        &(pFrameInfo_p->m_uiFrameSize));
            break;

        default:
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxGen,
                                        (BYTE*)pFrameInfo_p->m_pFrame,
                                        &(pFrameInfo_p->m_uiFrameSize));
            break;
    }

    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    // post event to DLL
    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &priority_p;
    event.m_uiSize = sizeof(priority_p);
    ret = eventu_postEvent(&event);
Exit:
    return ret;
}


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief	Issue a StatusRequest or IdentRequest

This function issues a StatusRequest or an IdentRequest to the specified node.

\param  service_p               Request service ID
\param  nodeId_p                The node to send the request.
\param  soaFlag1_p              Flag1 for this node (transmit in SoA and PReq).
                                If 0xff this flag is ignored.

\return Returns an error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_issueRequest(tEplDllReqServiceId service_p, UINT nodeId_p,
                                BYTE soaFlag1_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplEvent           event;
    tDllCalIssueRequest issueReq;

    // add node to appropriate request queue
    switch (service_p)
    {
        case kEplDllReqServiceIdent:
        case kEplDllReqServiceStatus:
            event.m_EventSink = kEplEventSinkDllkCal;
            event.m_EventType = kEplEventTypeDllkIssueReq;
            issueReq.service = service_p;
            issueReq.nodeId = nodeId_p;
            issueReq.soaFlag1 = soaFlag1_p;
            event.m_pArg = &issueReq;
            event.m_uiSize = sizeof (issueReq);
            ret = eventu_postEvent(&event);
            break;

        default:
            ret = kEplDllInvalidParam;
            goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issue a SyncRequest

This function issues a SyncRequest or an IdentRequest to the specified node.

\param  pSyncRequest_p          Pointer to sync request structure.
\param  size_p                  Size of sync request structure.

\return Returns an error code.
*/
//------------------------------------------------------------------------------
#if EPL_DLL_PRES_CHAINING_MN != FALSE
tEplKernel dllucal_issueSyncRequest(tEplDllSyncRequest* pSyncRequest_p, UINT size_p)
{
    tEplKernel  ret = kEplSuccessful;

    ret = instance_l.pTxSyncFuncs->pfnInsertDataBlock(instance_l.dllCalQueueTxSync,
                                                      (BYTE*)pSyncRequest_p, &size_p);
    return ret;
}
#endif
#endif


#if EPL_NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief	Configure the specified node

The function configures the specified node by sending a
kEplEventTypeDllkConfigNode event to the kernel DLL CAL module.

\param  pNodeInfo_p             Pointer to node info structure.

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_configNode(tEplDllNodeInfo* pNodeInfo_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkConfigNode;
    event.m_pArg = pNodeInfo_p;
    event.m_uiSize = sizeof (*pNodeInfo_p);

    ret = eventu_postEvent(&event);

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief	Add a node to the isochronous phase

The function adds a node to the isonchronous phase by sending a
kEplEventTypeDllkAddNode event to the kernel DLL CAL module.

\param  pNodeOpParam_p          Pointer to node info structure

\return Returns an error code
*/
//------------------------------------------------------------------------------
tEplKernel dllucal_addNode(tEplDllNodeOpParam* pNodeOpParam_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkAddNode;
    event.m_pArg = pNodeOpParam_p;
    event.m_uiSize = sizeof (*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Remove a node from the isochronous phase

The function removes the specified node from the isochronous phase by sending
a kEplEventTypeDllkDelNode event to the kernel DLL CAL module.

\param  pNodeOpParam_p          Pointer to node	info structure

\return Returns an error code

*/
//------------------------------------------------------------------------------
tEplKernel dllucal_deleteNode(tEplDllNodeOpParam* pNodeOpParam_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkDelNode;
    event.m_pArg = pNodeOpParam_p;
    event.m_uiSize = sizeof (*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Forward filter event to kernel part

The function forwards a filter event to the kernel DLL CAL module.

\param  serviceId_p             ASnd Service ID to forward.
\param  filter_p                Node ID filter to forward.

\return Returns an error code
*/
//------------------------------------------------------------------------------
static tEplKernel SetAsndServiceIdFilter(tEplDllAsndServiceId serviceId_p,
                                         tEplDllAsndFilter filter_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tEplEvent                   event;
    tDllCalAsndServiceIdFilter  servFilter;

    event.m_EventSink = kEplEventSinkDllkCal;
    event.m_EventType = kEplEventTypeDllkServFilter;
    servFilter.serviceId = serviceId_p;
    servFilter.filter = filter_p;
    event.m_pArg = &servFilter;
    event.m_uiSize = sizeof (servFilter);
    ret = eventu_postEvent(&event);

    return ret;
}

