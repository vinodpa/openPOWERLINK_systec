/**
********************************************************************************
\file   circbuffer.h

\brief  Definitions for circular buffer library

This file contains the definitions for the circular buffer library.
*******************************************************************************/
/*------------------------------------------------------------------------------
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

#ifndef _INC_circbuffer_H_
#define _INC_circbuffer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <Epl.h>
#include <global.h>
#include <EplTarget.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NR_OF_CIRC_BUFFERS              20
#define CIRCBUF_BLOCK_ALIGNMENT         4

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef enum
{
    kCircBufOk                          =  0,
    kCircBufNoReadableData              =  1,
    kCircBufReadsizeTooSmall            =  2,
    kCircBufBufferFull                  =  3,
    kCircBufOpenMismatch                =  7,
    kCircBufInvalidArg                  =  9,
    kCircBufOutOfMem                    = 11,
    kCircBufAlreadySignaling            = 13,
    kCircBufExceedDataSizeLimit         = 14,
    kCircBufNoResource                  = 20
} tCircBufError;

/**
*  \brief Header for circular buffer
*
*  The struct defines the header of a circular buffer which contains the
*  necessary control information to be shared between all accessing
*  threads.
*/
typedef struct
{
    size_t              bufferSize;         ///< Total size of circular buffer
    UINT32              writeOffset;        ///< The write offset
    UINT32              readOffset;         ///< The read offset
    size_t              freeSize;           ///< Available space in buffer
    UINT32              dataCount;          ///< The entry count
} tCircBufHeader;

/**
*  \brief Circular buffer instance
*
*  The struct defines the circular buffer instance which contains the private
*  information for a thread accessing a circular buffer.
*/
typedef struct
{
    tCircBufHeader*     pCircBufHeader;             ///< Pointer to the circular buffer header
    BYTE*               pCircBuf;                   ///< Pointer to the circular buffer
    void*               pCircBufArchInstance;       ///< Pointer to architecture specific stuff
    UINT8               bufferId;                   ///< The id of the circular buffer
    VOIDFUNCPTR         pfnSigCb;                   ///< Pointer to the signaling callback function
} tCircBufInstance;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tCircBufError circbuf_alloc (UINT8 id_p, size_t size_p, tCircBufInstance** ppInstance_p);
tCircBufError circbuf_free (tCircBufInstance* pInstance_p);
tCircBufError circbuf_connect (UINT8 id_p, tCircBufInstance** ppInstance_p);
tCircBufError circbuf_disconnect (tCircBufInstance* pInstance_p);
void          circbuf_reset (tCircBufInstance* pInstance_p);
tCircBufError circbuf_writeData (tCircBufInstance* pInstance_p, const void* pData_p, size_t size_p);
tCircBufError circbuf_writeMultipleData(tCircBufInstance* pInstance_p, const void* pData_p, size_t size_p,
                                        const void * pData2_p, size_t size2_p);
tCircBufError circbuf_readData(tCircBufInstance* pInstance_p, void* pData_p,
                               size_t size_p, size_t* pDataBlockSize_p);
UINT32        circbuf_getDataCount(tCircBufInstance* pInstance_p);
tCircBufError circBuf_setSignaling(tCircBufInstance* pInstance_p, VOIDFUNCPTR pfnSigCb_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_circbuffer_H_ */
