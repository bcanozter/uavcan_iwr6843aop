/*
* Copyright (C) 2024 Texas Instruments Incorporated
*
* All rights reserved not granted herein.
* Limited License.  
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, 
* non-exclusive license under copyrights and patents it now or hereafter 
* owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
* this software subject to the terms herein.  With respect to the foregoing patent 
* license, such license is granted  solely to the extent that any such patent is necessary 
* to Utilize the software alone.  The patent license shall not apply to any combinations which 
* include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  
* No hardware patent is licensed hereunder.
*
* Redistributions must preserve existing copyright notices and reproduce this license (including the 
* above copyright notice and the disclaimer and (if applicable) source code license limitations below) 
* in the documentation and/or other materials provided with the distribution
*
* Redistribution and use in binary form, without modification, are permitted provided that the following
* conditions are met:
*
*	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any 
*     software provided in binary form.
*	* any redistribution and use are licensed by TI for use only with TI Devices.
*	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
*
* If software source code is provided to you, modification and redistribution of the source code are permitted 
* provided that the following conditions are met:
*
*   * any redistribution and use of the source code, including any resulting derivative works, are licensed by 
*     TI for use only with TI Devices.
*   * any redistribution and use of any object code compiled from the source code and any resulting derivative 
*     works, are licensed by TI for use only with TI Devices.
*
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or 
* promote products derived from this software without specific prior written permission.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
* IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !(defined(CCS) || defined(_WIN32) || (defined(SOC_XWR68XX)) || defined(__TI_ARM_V7R4__))
#include <utils_mem.h>
#endif
/*#include <modules/utilities/radarOsal_malloc.h>*/
#include "radarOsal_malloc.h"

/* Osal functions for DSP*/
#if (defined(_TMS320C6X) || defined(_WIN32))
#ifndef SOC_XWR68XX // not XWR16x MMW_SDK

radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*!
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.

   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memInit(radarOsal_heapConfig *config, uint8_t numHeap)
{
    int32_t i;
    if (numHeap > RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
    {
        printf("Maximum supported heap is %d, cannot initialize %d heaps, exit!\n", RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS, numHeap);
        exit(1);
    }
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));

    for (i = 0; i < numHeap; i++)
    {
        if (strstr(config[i].heapName, "DDR") != NULL)
        {
            memcpy(gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapName, config[i].heapName, sizeof(config[i].heapName));
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr        = NULL; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize        = 0; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset = 0; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr     = (int8_t *)config[i].scratchAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize     = config[i].scratchSize;
        }
        else
        {
            memcpy(gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapName, config[i].heapName, sizeof(config[i].heapName));
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr        = (int8_t *)config[i].heapAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize        = config[i].heapSize;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset = 0;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr     = (int8_t *)config[i].scratchAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize     = config[i].scratchSize;
        }
    }
    return (RADARMEMOSAL_PASS);
}

/*!
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.

   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memDeInit(void)
{
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));
    return (RADARMEMOSAL_PASS);
}


/*!
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal_HeapType.

   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.

   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. 0 for no alignment requirement. Alignment has to be number being power of 2.

   \return    NULL if malloc failed, void pointer if malloc passed.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void *radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment)
{
    void    *pointer;
    uint32_t addrOffset;

    switch (memoryType)
    {
        case RADARMEMOSAL_HEAPTYPE_DDR_CACHED:
            if (scratchFlag == 0)
            {
#if defined(_WIN32)
                pointer = (void *)malloc(size);
#elif defined(CCS)
                pointer = (void *)memalign(alignment, size);
#else

                pointer = Utils_memAlloc(UTILS_HEAPID_DDR_CACHED_SR, size, alignment);
#endif
            }
            else
            {
                addrOffset = 0;
                if (alignment == 1)
                    pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
                else
                {
                    addrOffset = alignment - ((uint32_t)gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
                    pointer    = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
                }
                if (size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
                {
                    pointer = NULL;
                    printf("out of DDR memory!\n");
                }
            }
            return (pointer);

        case RADARMEMOSAL_HEAPTYPE_LL2:
            if (scratchFlag == 0)
            {
                if (alignment == 0)
                {
                    pointer = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset];
                    gRadarOsal_heapObj[memoryType].heapAllocOffset += size;
                }
                else
                {
                    addrOffset = alignment - ((uint32_t)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset]) & (alignment - 1);
                    pointer    = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[addrOffset + gRadarOsal_heapObj[memoryType].heapAllocOffset];
                    gRadarOsal_heapObj[memoryType].heapAllocOffset += size + addrOffset;
                }
                if (gRadarOsal_heapObj[memoryType].heapAllocOffset > gRadarOsal_heapObj[memoryType].heapSize)
                {
                    pointer = NULL;
                    printf("out of L2 heap memory!\n");
                }
            }
            else
            {
                addrOffset = 0;
                if (alignment == 1)
                    pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
                else
                {
                    addrOffset = alignment - ((uint32_t)gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
                    pointer    = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
                }
                if (size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
                {
                    pointer = NULL;
                    printf("out of L2 scratch memory!\n");
                }
            }
            return (pointer);

        default:
            return (NULL);
    }
}

/*!
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.

   \param[in]    ptr
               input poointer to be freed.

   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memFree() is a no-op for L2 heap, and radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void radarOsal_memFree(void *ptr, uint32_t size)
{
    /* only free heap alloca when ptr is in DDR */
    if (((uint32_t)ptr >= (uint32_t)gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr) && ((uint32_t)ptr < ((uint32_t)gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr) + gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize))
    {
#if defined(_WIN32) || defined(CCS)
        free(ptr);
#else
        Utils_memFree(UTILS_HEAPID_DDR_CACHED_SR, ptr, size);
#endif
        return;
    }
    else
    {
        gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed        = 0;
        gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset           = 0;
        gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].maxScratchSizeUsed = 0;
    }
}

#else /* Osal functions for XWR16x MMW_SDK*/


radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*!
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.

   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memInit(radarOsal_heapConfig *config, uint8_t numHeap)
{
    int32_t i;
    if (numHeap > RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
    {
        printf("Maximum supported heap is %d, cannot initialize %d heaps, exit!\n", RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS, numHeap);
        exit(1);
    }
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));

    for (i = 0; i < numHeap; i++)
    {
        gRadarOsal_heapObj[config[i].heapType].heapType           = config[i].heapType;
        gRadarOsal_heapObj[config[i].heapType].heapAddr           = config[i].heapAddr;
        gRadarOsal_heapObj[config[i].heapType].heapSize           = config[i].heapSize;
        gRadarOsal_heapObj[config[i].heapType].heapAllocOffset    = 0;
        gRadarOsal_heapObj[config[i].heapType].scratchAddr        = (int8_t *)config[i].scratchAddr;
        gRadarOsal_heapObj[config[i].heapType].maxScratchSizeUsed = 0;
        gRadarOsal_heapObj[config[i].heapType].scratchSize        = config[i].scratchSize;
    }
    return (RADARMEMOSAL_PASS);
}

/*!
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.

   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memDeInit(void)
{
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));
    return (RADARMEMOSAL_PASS);
}


/*!
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal_HeapType.

   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.

   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. 0 for no alignment requirement. Alignment has to be number being power of 2.

   \return    NULL if malloc failed, void pointer if malloc passed.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void *radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment)
{
    void    *pointer = NULL;
    uint32_t addrOffset;

    if (memoryType > (uint8_t)RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
        return pointer;


    if (scratchFlag == 0)
    {
        if (alignment <= 1)
        {
            pointer = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size;
        }
        else
        {
            addrOffset = alignment - ((uint32_t)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset]) & (alignment - 1);
            pointer    = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[addrOffset + gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size + addrOffset;
        }
        if (gRadarOsal_heapObj[memoryType].heapAllocOffset > gRadarOsal_heapObj[memoryType].heapSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR heap memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 heap memory!\n");
            else
                printf("out of L1 heap memory!\n");
        }
    }
    else
    {
        addrOffset = 0;
        if (alignment <= 1)
            pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
        else
        {
            addrOffset = alignment - ((uint32_t)gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
            pointer    = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
        }
        if (gRadarOsal_heapObj[memoryType].maxScratchSizeUsed < size)
            gRadarOsal_heapObj[memoryType].maxScratchSizeUsed = size;
        if (size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR scratch memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 scratch memory!\n");
            else
                printf("out of L1 scratch memory!\n");
        }
    }

    return (pointer);
}


/*!
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.

   \param[in]    ptr
               input poointer to be freed.

   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memFree() is a no-op for L2 heap, and radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void radarOsal_memFree(void *ptr, uint32_t size)
{
    return;
}

#endif
#endif

#ifdef __TI_ARM_V7R4__
#ifndef SOC_XWR68XX // not XWR16x MMW_SDK
radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*!
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.

   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memInit(radarOsal_heapConfig *config, uint8_t numHeap)
{
    int32_t i;
    if (numHeap > RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
    {
        printf("Maximum supported heap is %d, cannot initialize %d heaps, exit!\n", RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS, numHeap);
        exit(1);
    }
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));

    for (i = 0; i < numHeap; i++)
    {
        if (strstr(config[i].heapName, "DDR") != NULL)
        {
            memcpy(gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapName, config[i].heapName, sizeof(config[i].heapName));
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType        = RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr        = (int8_t *)config[i].heapAddr; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize        = config[i].heapSize; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset = 0; /* not used for default DDR heap  */
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr     = (int8_t *)config[i].scratchAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize     = config[i].scratchSize;
        }
        else
        {
            memcpy(gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapName, config[i].heapName, sizeof(config[i].heapName));
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType = RADARMEMOSAL_HEAPTYPE_LL2;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr        = (int8_t *)config[i].heapAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize        = config[i].heapSize;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset = 0;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr     = (int8_t *)config[i].scratchAddr;
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize     = config[i].scratchSize;
        }
    }
    return (RADARMEMOSAL_PASS);
}

/*!
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.

   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memDeInit(void)
{
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));
    return (RADARMEMOSAL_PASS);
}


/*!
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal_HeapType.

   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.

   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. 0 for no alignment requirement. Alignment has to be number being power of 2.

   \return    NULL if malloc failed, void pointer if malloc passed.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void *radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment)
{
    void    *pointer = NULL;
    uint32_t addrOffset;

    if (memoryType > (uint8_t)RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
        return pointer;


    if (scratchFlag == 0)
    {
        if (alignment <= 1)
        {
            pointer = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size;
        }
        else
        {
            addrOffset = alignment - ((uint32_t)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset]) & (alignment - 1);
            pointer    = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[addrOffset + gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size + addrOffset;
        }
        if (gRadarOsal_heapObj[memoryType].heapAllocOffset > gRadarOsal_heapObj[memoryType].heapSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR heap memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 heap memory!\n");
            else
                printf("out of L1 heap memory!\n");
        }
    }
    else
    {
        addrOffset = 0;
        if (alignment <= 1)
            pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
        else
        {
            addrOffset = alignment - ((uint32_t)gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
            pointer    = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
        }
        if (gRadarOsal_heapObj[memoryType].maxScratchSizeUsed < size)
            gRadarOsal_heapObj[memoryType].maxScratchSizeUsed = size;
        if (size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR scratch memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 scratch memory!\n");
            else
                printf("out of L1 scratch memory!\n");
        }
    }

    return (pointer);
}


/*!
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.

   \param[in]    ptr
               input poointer to be freed.

   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memFree() is a no-op for L2 heap, and radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void radarOsal_memFree(void *ptr, uint32_t size)
{
    return;
}
#else /* Osal functions for XWR16x MMW_SDK*/

radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*!
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.

   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memInit(radarOsal_heapConfig *config, uint8_t numHeap)
{
    int32_t i;
    if (numHeap > RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
    {
        printf("Maximum supported heap is %d, cannot initialize %d heaps, exit!\n", RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS, numHeap);
        exit(1);
    }
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));

    for (i = 0; i < numHeap; i++)
    {
        gRadarOsal_heapObj[config[i].heapType].heapType           = config[i].heapType;
        gRadarOsal_heapObj[config[i].heapType].heapAddr           = config[i].heapAddr;
        gRadarOsal_heapObj[config[i].heapType].heapSize           = config[i].heapSize;
        gRadarOsal_heapObj[config[i].heapType].heapAllocOffset    = 0;
        gRadarOsal_heapObj[config[i].heapType].scratchAddr        = (int8_t *)config[i].scratchAddr;
        gRadarOsal_heapObj[config[i].heapType].maxScratchSizeUsed = 0;
        gRadarOsal_heapObj[config[i].heapType].scratchSize        = config[i].scratchSize;
    }
    return (RADARMEMOSAL_PASS);
}

/*!
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.

   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memDeInit(void)
{
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));
    return (RADARMEMOSAL_PASS);
}


/*!
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal_HeapType.

   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.

   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. 0 for no alignment requirement. Alignment has to be number being power of 2.

   \return    NULL if malloc failed, void pointer if malloc passed.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void *radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment)
{
    void    *pointer = NULL;
    uint32_t addrOffset;

    if (memoryType > (uint8_t)RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
        return pointer;


    if (scratchFlag == 0)
    {
        if (alignment <= 1)
        {
            pointer = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size;
        }
        else
        {
            addrOffset = alignment - ((uint32_t)&gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset]) & (alignment - 1);
            pointer    = (void *)&gRadarOsal_heapObj[memoryType].heapAddr[addrOffset + gRadarOsal_heapObj[memoryType].heapAllocOffset];
            gRadarOsal_heapObj[memoryType].heapAllocOffset += size + addrOffset;
        }
        if (gRadarOsal_heapObj[memoryType].heapAllocOffset > gRadarOsal_heapObj[memoryType].heapSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR heap memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 heap memory!\n");
            else
                printf("out of L1 heap memory!\n");
        }
    }
    else
    {
        addrOffset = 0;
        if (alignment <= 1)
            pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
        else
        {
            addrOffset = alignment - ((uint32_t)gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
            pointer    = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
        }
        if (gRadarOsal_heapObj[memoryType].maxScratchSizeUsed < size)
            gRadarOsal_heapObj[memoryType].maxScratchSizeUsed = size;
        if (size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
        {
            pointer = NULL;
            if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
                printf("out of DDR scratch memory!\n");
            else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
                printf("out of L2 scratch memory!\n");
            else
                printf("out of L1 scratch memory!\n");
        }
    }

    return (pointer);
}


/*!
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.

   \param[in]    ptr
               input poointer to be freed.

   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memFree() is a no-op for L2 heap, and radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void radarOsal_memFree(void *ptr, uint32_t size)
{
    return;
}

#endif
#endif
