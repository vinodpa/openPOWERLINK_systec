/**
********************************************************************************
\file   dllucal-ioctl.c

\brief  Source file for user DLL CAL module

This file contains an implementation of the user dll CAL module which uses
Linux ioctl for communication with the kernel layer.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012-2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <dllcal.h>
#include <user/ctrlucal.h>
#include <powerlink-module.h>

#include <sys/ioctl.h>

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
\brief DLL CAL ioctl instance type

This struct is used for every event queue using the Linux ioctl for
event posting.
*/
typedef struct
{
    tDllCalQueue                dllCalQueue;        ///< DLL CAL queue
    tEplDllAsyncReqPriority     priority;           ///< Request priority
    int                         fd;                 ///< File descriptor of openPOWERLINK driver
} tDllCalIoctlInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel addInstance(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
static tEplKernel delInstance(tDllCalQueueInstance pDllCalQueue_p);
static tEplKernel insertDataBlock(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);

/* define external function interface */
static tDllCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    insertDataBlock,
    NULL,
    NULL,
    NULL
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return pointer to function interface

This function returns a pointer to the function interface structure which
is used to access the dllcal functions of the shared buffer implementation.

\return Returns a pointer to the local function interface
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllcalioctl_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Add DLL CAL instance

Add an instance for TX packet forwarding in DLL CAL.

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  dllCalQueue_p           parameter that determines the queue

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel addInstance(tDllCalQueueInstance *ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tDllCalIoctlInstance*       pInstance;

    pInstance = (tDllCalIoctlInstance *) EPL_MALLOC(sizeof(tDllCalIoctlInstance));
    if(pInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pInstance->dllCalQueue = dllCalQueue_p;
    pInstance->fd = ctrlucal_getFd();

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete DLL CAL instance

Delete the DLL CAL instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalIoctlInstance*     pInstance = (tDllCalIoctlInstance*)pDllCalQueue_p;

    EPL_FREE(pInstance);
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Insert data block into queue

Inserts a data block into the DLL CAL queue.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to the data block to be insert
\param  pDataSize_p             Pointer to the size of the data block to be
                                insert

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel insertDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                   BYTE *pData_p, UINT *pDataSize_p)
{
    tEplKernel                      ret = kEplSuccessful;
    tDllCalIoctlInstance*           pInstance =
                                            (tDllCalIoctlInstance*)pDllCalQueue_p;
    tIoctlDllCalAsync               ioctlAsyncFrame;
    int                             ioctlRet;

    if(pInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    ioctlAsyncFrame.size = *pDataSize_p;
    ioctlAsyncFrame.queue = pInstance->dllCalQueue;
    ioctlAsyncFrame.pData = pData_p;
    //TRACE ("%s() send async frame: size:%d\n", __func__, pFrameInfo_p->m_uiFrameSize);
    ioctlRet = ioctl(pInstance->fd, PLK_CMD_DLLCAL_ASYNCSEND, (ULONG)&ioctlAsyncFrame);
    if (ioctlRet < 0)
        return kEplDllAsyncTxBufferFull;
    return kEplSuccessful;

Exit:
    return ret;
}




