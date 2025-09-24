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

#ifndef _RADARPROCESS_H
#define _RADARPROCESS_H


#include <swpform.h>
/*#include <modules/rangeProc/highAccuRangeProc/api/RADARDEMO_highAccuRangeProc.h>*/
#include "RADARDEMO_highAccuRangeProc.h"

#if defined(_WIN32) || defined(CCS)
#include <stdio.h>
#endif


/**
 *  \def  M_PI
 *
 *  \brief   constant value used for pi.
 *
 *  \sa
 */
#define M_PI 3.14159265358979323846f

#define NUM_RADAR_SENSORS 1 // 1 radar sensor -> a maximum number of 256
#define NUM_RADAR_TXANT   2 // 2 transmitting antennas
#define NUM_RADAR_RXANT   4 // 4 receiving antennas

#define MAX_FFT1D_SIZE (1024)

#define RADAR_DSP_BUFF_CONT_MAX_NUM_BUFF 2U

typedef enum
{
    LOW_RESOLUTION  = 0,
    HIGH_RESOLUTION = 1,
    DUAL_RESOLUTION = 2,
    TEST_TONE       = 3
} ResolutionMode_e;

typedef enum
{
    /** Detection Object Data */
    RadarDsp_outputDataType_OBJ_DATA = 0,

    /** Range-Doppler Heap Map. */
    RadarDsp_outputDataType_HEAT_MAP = 1,

    /** Marker for validation. */
    RadarDsp_outputDataType_MAX

} RadarDsp_outputDataType;

typedef struct
{
    /** Type of the data carried in the buffer. */
    RadarDsp_outputDataType type;

    /** Data buffer pointer. */
    void *buff;

} RadarDsp_outputBuffType;

typedef struct
{
    /** Number of valid buffers in the array. */
    uint16_t numBuff;

    /** Output buffers. */
    RadarDsp_outputBuffType elem[RADAR_DSP_BUFF_CONT_MAX_NUM_BUFF];

} RadarDsp_outputBuffCntxt;

// radar process output data structure
typedef struct
{
    uint32_t fft1DSize; /**< 1D FFT size*/
    float    rangeEst;
    float    rangeEst1;
    float    rangeEst2;
    float    deltaPhaseEst;
    float    linearSNRest;
    float    fft1Dinput[2 * MAX_FFT1D_SIZE];
} radarProcessOutput_NetworkLink_t;

typedef radarProcessOutput_NetworkLink_t radarProcessOutput_t;

typedef struct _radarProcessBenchmarkElem_
{
    uint32_t rangeEstCycles;
} radarProcessBenchmarkElem;


typedef struct _radarProcessBenchmarkObj_
{
    uint32_t                   bufferLen;
    uint32_t                   bufferIdx;
    radarProcessBenchmarkElem *buffer;
} radarProcessBenchmarkObj;


// user input configuration parameters
typedef struct _radarModuleConfig_
{
    // rangeFFT/Doppler parameters
    uint16_t framePeriod; /**< Frame period in msec. */
    uint16_t numAdcSamplePerChirp; /**< number of adc samples per chirp. */
    uint16_t numChirpPerFrame; /**< number of chirps per frame. */
    uint16_t numTxAntenna; /**< number of antennas. */
    uint16_t numPhyRxAntenna; /**< number of physical RX antennas. */
    uint16_t rangeWinSize; /**< range window size. */
    float    chirpInterval;
    float    win1D[16];

    // detection parameters
    RADARDEMO_highAccuRangeProc_config highAccuConfig;

} radarProcessConfig_t;
typedef enum
{
    PROCESS_OK = 0,
    PROCESS_ERROR_INIT_MEMALLOC_FAILED,
    PROCESS_ERROR_RANGEPROC_INIT_FAILED,
    PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED,
    PROCESS_ERROR_NOT_SUPPORTED
} ProcessErrorCodes;

#endif // _PROCESS_H

/* Nothing past this point */
