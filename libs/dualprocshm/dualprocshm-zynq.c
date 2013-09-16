/**
********************************************************************************
\file   dualprocshm-zynq.c

\brief  Dual Processor Library Support File - For Zynq target

This file provides specific function definition for Zynq to support shared memory
interface using dual processor library

\ingroup module_dualprocshm
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2012 Kalycito Infotech Private Limited
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

#include "dualprocshm-target.h"
#include <string.h>
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


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Target specific to routine to retrieve the base address of
        common memory between dual processors

\param  pSize_p      minimum size of the common memory, returns the
                     actual size of common memory
\return pointer to base address of common memory
 */
//------------------------------------------------------------------------------
UINT8* dualprocshm_getCommonMemAddr(UINT16* pSize_p)
{
    UINT8* pAddr;

    if(*pSize_p > MAX_COMMON_MEM_SIZE )
    {
        TRACE("Common memory size exceeds Size: %d Avail: %d\n",*pSize_p,MAX_COMMON_MEM_SIZE);
        return NULL;
    }

    pAddr = (UINT8 *) COMMON_MEM_BASE;

    memset(pAddr,0,MAX_COMMON_MEM_SIZE);

    *pSize_p = MAX_COMMON_MEM_SIZE;

    return pAddr;
}
//------------------------------------------------------------------------------
/**
\brief  Target specific to routine to release the base address of
        common memory between dual processors

\param  pSize_p      size of the common memory


 */
//------------------------------------------------------------------------------
void dualprocshm_releaseCommonMemAddr(UINT16 pSize_p)
{

}
//------------------------------------------------------------------------------
/**
\brief  Target specific to routine to retrieve the base address for storing
        dynamic mapping table

\return pointer to base address of dynamic mapping table
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_getDynMapTableAddr(void)
{
    UINT8* pAddr;
    pAddr = (UINT8 *) MEM_ADDR_TABLE_BASE;

    memset(pAddr,0,(MAX_DYNAMIC_BUFF_COUNT * 4));
    return pAddr;
}
//------------------------------------------------------------------------------
/**
\brief  Target specific to routine to retrieve the base address for storing
        dynamic mapping table
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseDynMapTableAddr()
{

}
//------------------------------------------------------------------------------
/**
\brief  Target specific memory read routine

\param  base      base address to be read
\param  Size_p    No of bytes to be read
\param  pData_p   Pointer to receive the read data

 */
//------------------------------------------------------------------------------
void dualprocshm_targetReadData(UINT8* pBase_p, UINT16 Size_p, UINT8* pData_p)
{
    if(pBase_p == NULL || pData_p == NULL)
    {
        return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE((UINT32)pBase_p,Size_p);

    memcpy(pData_p,pBase_p,Size_p);

}
//------------------------------------------------------------------------------
/**
\brief  Target specific memory write routine

\param  base      base address to be written
\param  Size_p    No of bytes to write
\param  pData_p   Pointer to memory containing data to written

 */
//------------------------------------------------------------------------------
void dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 Size_p, UINT8* pData_p)
{
    if(pBase_p == NULL || pData_p == NULL)
    {
        return;
    }

    memcpy(pBase_p,pData_p,Size_p);

    DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32)pBase_p,Size_p);

}

/**
 * EOF
 */
