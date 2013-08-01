/**
********************************************************************************
\file   dllk-internal.h

\brief  Internal definitions for DLL kernel module files

This file contains internal definitions used by the DLL kernel implementation
files.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_dllk_internal_H_
#define _INC_dllk_internal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <kernel/dllk.h>
#include <kernel/dllkcal.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <nmt.h>
#include <edrv.h>
#include <Benchmark.h>

#if EPL_TIMER_USE_HIGHRES != FALSE
#include "kernel/EplTimerHighResk.h"
#endif

#include "kernel/dllktgt.h"

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
#include "kernel/EplTimerSynck.h"
#endif


//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------
#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) && (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
#error "DLLK: select only one of EPL_DLL_PRES_READY_AFTER_SOA and EPL_DLL_PRES_READY_AFTER_SOC."
#endif

#if ((EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)) &&  defined(CONFIG_INCLUDE_NMT_MN)
#error "DLLK: currently, EPL_DLL_PRES_READY_AFTER_* is not supported if EPL_MODULE_NMT_MN is enabled."
#endif

#if (EDRV_FAST_TXFRAMES == FALSE) && ((EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE))
#error "DLLK: EPL_DLL_PRES_READY_AFTER_* is enabled, but not EDRV_FAST_TXFRAMES."
#endif

#if defined(CONFIG_INCLUDE_NMT_MN) && (EPL_DLL_PRES_FILTER_COUNT == 0)
#error "MN support needs EPL_DLL_PRES_FILTER_COUNT != 0"
#endif

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) && (EDRV_AUTO_RESPONSE_DELAY == FALSE)
#error "Ethernet driver support for auto-response delay is required for PRes Chaining."
#endif

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) && (EPL_DLL_PROCESS_SYNC != EPL_DLL_PROCESS_SYNC_ON_TIMER)
#error "PRes Chaining CN support requires EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER."
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
void  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#define TGT_DBG_POST_TRACE_VALUE(v)
#endif

#define DLLK_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkDllk << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)

// defines for indexes of tEplDllInstance.m_pTxFrameInfo
#define DLLK_TXFRAME_IDENTRES       0   // IdentResponse on CN / MN
#define DLLK_TXFRAME_STATUSRES      2   // StatusResponse on CN / MN
#define DLLK_TXFRAME_NMTREQ         4   // NMT Request from FIFO on CN / MN

#if EPL_DLL_PRES_CHAINING_CN != FALSE
#define DLLK_TXFRAME_SYNCRES      6   // SyncResponse on CN
#define DLLK_TXFRAME_NONEPL       8   // non-EPL frame from FIFO on CN / MN
#else
#define DLLK_TXFRAME_NONEPL       6   // non-EPL frame from FIFO on CN / MN
#endif

#define DLLK_TXFRAME_PRES           (DLLK_TXFRAME_NONEPL + 2) // PRes on CN / MN

#if defined(CONFIG_INCLUDE_NMT_MN)
  #define DLLK_TXFRAME_SOC          (DLLK_TXFRAME_PRES + 2)   // SoC on MN
  #define DLLK_TXFRAME_SOA          (DLLK_TXFRAME_SOC + 2)    // SoA on MN
  #define DLLK_TXFRAME_PREQ         (DLLK_TXFRAME_SOA + 2)    // PReq on MN
  #define DLLK_TXFRAME_COUNT        (DLLK_TXFRAME_PREQ + (2 * (EPL_D_NMT_MaxCNNumber_U8 + 2)))
                                    // on MN: 7 + MaxPReq of regular CNs + 1 Diag + 1 Router
#else
  #define DLLK_TXFRAME_COUNT        (DLLK_TXFRAME_PRES + 2)
#endif


#define DLLK_FILTER_PREQ                0
#define DLLK_FILTER_SOA_IDREQ           1
#define DLLK_FILTER_SOA_STATREQ         2
#define DLLK_FILTER_SOA_NMTREQ          3
#if EPL_DLL_PRES_CHAINING_CN != FALSE
  #define DLLK_FILTER_SOA_SYNCREQ       4
  #define DLLK_FILTER_SOA_NONEPL        5
#else
  #define DLLK_FILTER_SOA_NONEPL        4
#endif

#define DLLK_FILTER_SOA                 (DLLK_FILTER_SOA_NONEPL + 1)
#define DLLK_FILTER_SOC                 (DLLK_FILTER_SOA + 1)
#define DLLK_FILTER_ASND                (DLLK_FILTER_SOC + 1)
#define DLLK_FILTER_PRES                (DLLK_FILTER_ASND + 1)

#if EPL_DLL_PRES_FILTER_COUNT < 0
  #define DLLK_FILTER_COUNT             (DLLK_FILTER_PRES + 1)
#else
  #define DLLK_FILTER_COUNT             (DLLK_FILTER_PRES + EPL_DLL_PRES_FILTER_COUNT)
#endif

#define DLLK_SOAREQ_COUNT               3

// defines for tEdrvTxBuffer.m_uiTxBufLen
#define DLLK_BUFLEN_EMPTY               0   // buffer is empty
#define DLLK_BUFLEN_FILLING             1   // just the buffer is being filled
#define DLLK_BUFLEN_MIN                 60  // minimum ethernet frame length

// defines for tEplDllkInstance.m_bUpdateTxFrame
#define DLLK_UPDATE_NONE                0   // no update necessary
#define DLLK_UPDATE_STATUS              1   // StatusRes needs update
#define DLLK_UPDATE_BOTH                2   // IdentRes and StatusRes need update

// defines for tEplDllNodeInfo.m_bPresFilterFlags
#define DLLK_FILTER_FLAG_PDO            0x01    // PRes needed for RPDO
#define DLLK_FILTER_FLAG_HB             0x02    // PRes needed for Heartbeat Consumer

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tNmtState               nmtState;
    UINT64                  relativeTime;
    UINT8                   aLocalMac[6];
    tEdrvTxBuffer*          pTxBuffer;                      // Buffers for Tx-Frames
    UINT                    maxTxFrames;
    UINT8                   flag1;                          // Flag 1 with EN, EC for PRes, StatusRes
    UINT8                   mnFlag1;                        // Flag 1 with MS, EA, ER from PReq, SoA of MN
    UINT8                   flag2;                          // Flag 2 with PR and RS for PRes, StatusRes, IdentRes
    UINT8                   updateTxFrame;
    UINT                    usedPresFilterCount;
    tEplDllConfigParam      dllConfigParam;
    tEplDllIdentParam       dllIdentParam;
    tDllState               dllState;
    tDllkCbProcessRpdo      pfnCbProcessRpdo;
    tDllkCbProcessTpdo      pfnCbProcessTpdo;
    tEplDllkCbAsync         pfnCbAsync;
    tEplSyncCb              pfnCbSync;
    tEplDllAsndFilter       aAsndFilter[EPL_DLL_MAX_ASND_SERVICE_ID];
    tEdrvFilter             aFilter[DLLK_FILTER_COUNT];
#if EPL_NMT_MAX_NODE_ID > 0
    tDllkNodeInfo           aNodeInfo[EPL_NMT_MAX_NODE_ID];
#endif
    UINT8                   curTxBufferOffsetIdentRes;
    UINT8                   curTxBufferOffsetStatusRes;
    UINT8                   curTxBufferOffsetNmtReq;
    UINT8                   curTxBufferOffsetNonEpl;
    UINT8                   curTxBufferOffsetCycle;         // PRes, SoC, SoA, PReq
#if EPL_DLL_PRES_CHAINING_CN != FALSE
    UINT8                   curTxBufferOffsetSyncRes;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllkNodeInfo*          pFirstNodeInfo;
    UINT8                   aCnNodeIdList[2][EPL_NMT_MAX_NODE_ID];
    UINT8                   curNodeIndex;
    tEdrvTxBuffer**         ppTxBufferList;
    UINT8                   syncLastSoaReq;
    tEplDllReqServiceId     aLastReqServiceId[DLLK_SOAREQ_COUNT];
    UINT                    aLastTargetNodeId[DLLK_SOAREQ_COUNT];
    UINT8                   curLastSoaReq;
    BOOL                    fSyncProcessed;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    BOOL                    fPrcSlotFinished;
    tDllkNodeInfo*          pFirstPrcNodeInfo;
#endif
#endif

#if EPL_TIMER_USE_HIGHRES != FALSE
    tEplTimerHdl            timerHdlCycle;                  // used for EPL cycle monitoring on CN and generation on MN
#if defined(CONFIG_INCLUDE_NMT_MN)
    tEplTimerHdl            timerHdlResponse;               // used for CN response monitoring
#endif
#endif

    UINT                    cycleCount;                     // cycle counter (needed for multiplexed cycle support)
    UINT64                  frameTimeout;                   // frame timeout (cycle length + loss of frame tolerance)

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    UINT                    syncReqPrevNodeId;
    tEplTgtTimeStamp*       pSyncReqPrevTimeStamp;
    BOOL                    fPrcEnabled;
    UINT32                  prcPResTimeFirst;
    UINT32                  prcPResFallBackTimeout;
#endif
} tDllkInstance;

//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------
extern tDllkInstance        dllkInstance_g;
TGT_DLLK_DECLARE_CRITICAL_SECTION

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
/* Frame processing functions (dllkframe.c) */
tEdrvReleaseRxBuffer dllk_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p) SECTION_DLLK_FRAME_RCVD_CB;
void       dllk_processTransmittedNmtReq(tEdrvTxBuffer * pTxBuffer_p);
void       dllk_processTransmittedNonEpl(tEdrvTxBuffer * pTxBuffer_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
void       dllk_processTransmittedSoc(tEdrvTxBuffer * pTxBuffer_p);
void       dllk_processTransmittedSoa(tEdrvTxBuffer * pTxBuffer_p);
#endif
tEplKernel dllk_updateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tEplKernel dllk_updateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState NmtState_p);
tEplKernel dllk_updateFramePres(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tEplKernel dllk_checkFrame(tEplFrame * pFrame_p, UINT frameSize_p);
tEplKernel dllk_createTxFrame(UINT* pHandle_p, UINT* pFrameSize_p,
                              tEplMsgType msgType_p, tEplDllAsndServiceId serviceId_p);
tEplKernel dllk_deleteTxFrame(UINT handle_p);
tEplKernel dllk_processTpdo(tEplFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
tEplKernel dllk_mnSendSoa(tNmtState nmtState_p, tDllState* pDllStateProposed_p,
                          BOOL fEnableInvitation_p);
tEplKernel dllk_updateFrameSoa(tEdrvTxBuffer* pTxBuffer_p, tNmtState NmtState_p,
                               BOOL fEnableInvitation_p, BYTE curReq_p);
tEplKernel dllk_asyncFrameNotReceived(tEplDllReqServiceId reqServiceId_p, UINT nodeId_p);
#endif

//------------------------------------------------------------------------------
/* event functions (dllkevent.c) */
tEplKernel dllk_postEvent(tEplEventType EventType_p);
tEplKernel controlPdokcalSync(BOOL fEnable_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
tEplKernel dllk_issueLossOfPres(UINT nodeId_p);
#endif

//------------------------------------------------------------------------------
/* DLL filter functions (dllkfilter.c) */
void       dllk_setupAsndFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSocFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSoaFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSoaIdentReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupSoaStatusReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupSoaNmtReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
#if EPL_DLL_PRES_CHAINING_CN != FALSE
void       dllk_setupSoaSyncReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
#endif
void       dllk_setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p);
void       dllk_setupPreqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p, UINT8* pMacAdrs_p);
#if EPL_NMT_MAX_NODE_ID > 0
tEplKernel dllk_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p);
tEplKernel dllk_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType nodeOpType_p, BOOL fUpdateEdrv_p);
#endif

//------------------------------------------------------------------------------
/* DLL state machine functions (dllkstatemachine.c) */
tEplKernel dllk_changeState(tNmtEvent nmtEvent_p, tNmtState nmtState_p);

//------------------------------------------------------------------------------
/* node functions */
tEplKernel dllk_cleanupLocalNode(tNmtState oldNmtState_p);
tEplKernel dllk_setupLocalNode(tNmtState nmtState_p);
tEplKernel dllk_setupLocalNodeCn(void);
#if defined(CONFIG_INCLUDE_NMT_MN)
tEplKernel dllk_setupLocalNodeMn(void);
tEplKernel dllk_addNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p);
tEplKernel dllk_deleteNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p);
tEplKernel dllk_setupAsyncPhase(tNmtState nmtState_p, UINT nextTxBufferOffset_p,
                                UINT32 nextTimeOffsetNs_p, UINT* pIndex_p) SECTION_DLLK_PROCESS_SYNC;
tEplKernel dllk_setupSyncPhase(tNmtState nmtState_p, BOOL fReadyFlag_p, UINT nextTxBufferOffset_p,
                               UINT32* pNextTimeOffsetNs_p, UINT* pIndex_p) SECTION_DLLK_PROCESS_SYNC;
#endif
#if EPL_NMT_MAX_NODE_ID > 0
tDllkNodeInfo* dllk_getNodeInfo(UINT uiNodeId_p);
#endif

//------------------------------------------------------------------------------
/* Cycle/Sync Callback functions */
#if defined(CONFIG_INCLUDE_NMT_MN)
tEplKernel dllk_cbCyclicError(tEplKernel errorCode_p, tEdrvTxBuffer * pTxBuffer_p);
tEplKernel dllk_cbMnSyncHandler(void);
tEplKernel dllk_cbMnTimerCycle(tEplTimerEventArg* pEventArg_p);
#endif
#if EPL_TIMER_USE_HIGHRES != FALSE
tEplKernel dllk_cbCnTimer(tEplTimerEventArg* pEventArg_p);
#endif
#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
tEplKernel dllk_cbCnTimerSync(void);
tEplKernel dllk_cbCnLossOfSync(void);
#endif

//------------------------------------------------------------------------------
/* PRes Chaining functions */
#if EPL_DLL_PRES_CHAINING_CN == TRUE
tEplKernel dllk_presChainingEnable (void);
tEplKernel dllk_presChainingDisable (void);
#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
tEplKernel dllk_cbCnPresFallbackTimeout(void);
#endif
#endif

#ifdef __cplusplus
}
#endif


#endif /* _INC_dllk_internal_H_ */
