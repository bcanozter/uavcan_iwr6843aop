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

#ifndef RADARDEMO_HIGHACCURANGEPROC_H
#define RADARDEMO_HIGHACCURANGEPROC_H

#include "swpform.h"
/*#include <modules/utilities/radarOsal_malloc.h>*/
#include "radarOsal_malloc.h"
#include <math.h>

typedef enum
{
    RADARDEMO_HIGHACCURANGEPROC_NO_ERROR = 0, /**< no error */
    RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_HANDLE, /**< RADARDEMO_highAccuRangeProc_create failed to allocate handle */
    RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM, /**< RADARDEMO_highAccuRangeProc_create failed to allocate memory for buffers in local instance  */
    RADARDEMO_HIGHACCURANGEPROC_INOUTPTR_NOTCORRECT /**< input and/or output buffer for RADARDEMO_highAccuRangeProc_run are either NULL, or not aligned properly  */
} RADARDEMO_highAccuRangeProc_errorCode;

/**
 *  \struct   _RADARDEMO_highAccuRangeProc_input_
 *   {
 *   	cplx16_t     *inputSignal;
 *		uint32_t     chirpNumber;
 *   }   RADARDEMO_highAccuRangeProc_input;
 *
 *  \brief   Structure for input to RADARDEMO_highAccuRangeProc module.
 *
 *
 */

typedef struct _RADARDEMO_highAccuRangeProc_input_
{
    cplx16_t *inputSignal; /**< Input signal from ADC*/
    int32_t   chirpNumber; /**< chirp number: 0 to number of chirps per frame. If set to negative number, then input is accumulated signal over all chirps within the frame.*/
} RADARDEMO_highAccuRangeProc_input;

typedef struct _RADARDEMO_highAccuRangeProc_output_
{
    float rangeEst; /**< Range estimation from fine frequency estimation
                                             if config.enablePhaseEst = 0, this is from zoomed in FFT only
                                             if config.enablePhaseEst = 1, this is from zoomed in FFT plus delta phase correction.*/
    float rangeEst1;
    float rangeEst2;

    float deltaPhaseEst; /**< delta phase estimation, only valid when enablePhaseEst is set in config.*/
    float linearSNREst; /**< Estimated linear SNR.*/
} RADARDEMO_highAccuRangeProc_output;


typedef struct _RADARDEMO_highAccuRangeProc_config_
{
    uint32_t fft1DSize; /**< 1D FFT size*/
    uint32_t nSamplesPerChirp; /**< number of samples per chirp*/
    uint32_t numChirpsPerFrame; /**< number of chirp per frame*/
    float    maxBeatFreq; /**< maximum beat frequency*/
    float    chirpBandwidth; /**< chirp bandwidth.*/
    float    chirpRampTime; /**< chirp ramp duration.*/
    float    fc; /**< chirp start frequency.*/
    float    chirpSlope; /**< chirp slope.*/
    float    adcStartTimeConst; /**< ADC start constant.*/
    float    adcSampleRate; /**< ADC sampling rate.*/
    float   *win1D; /**< pointer to 1D windowing function.*/
    int16_t  win1DLength; /**< half length of the 1D windowing function.*/
    uint8_t  numRangeBinZoomIn; /**< number of bins to zoom in for frequenc estimation.*/
    uint8_t  enablePhaseEst; /**< enable estimation using phase correction.*/
    uint8_t  enableLinearFit; /**< enable linear fit in phase estimation.*/
    uint8_t  enableFilter; /**< enable filtering in phase estimation.*/
    uint16_t skipLeft; /**< number of samples to skip from the left.*/
    uint16_t skipRight; /**< number of samples to skip from the right.*/
    float   *fft1DIn; /**< pointer to 1D FFT input.*/
    uint8_t  enableRangeLimit; /**< enable RangeLimit.*/
    float    skipMin; /**< number of samples to skip from 0 to min range(m).*/
    float    skipMax; /**< number of samples to skip from the max range(m) to faraway.*/


} RADARDEMO_highAccuRangeProc_config;


/*!
   \fn     RADARDEMO_highAccuRangeProc_create

   \brief   Create and initialize RADARDEMO_highAccuRangeProc module.

   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_highAccuRangeProc module.

   \param[in]    errorCode
               Pointer to error code.

   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.

   \pre       none

   \post      none


 */

extern void *RADARDEMO_highAccuRangeProc_create(
    IN RADARDEMO_highAccuRangeProc_config     *moduleConfig,
    OUT RADARDEMO_highAccuRangeProc_errorCode *errorCode);

/*!
   \fn     RADARDEMO_highAccuRangeProc_delete

   \brief   Delete RADARDEMO_highAccuRangeProc module.

   \param[in]    handle
               Module handle.

   \pre       none

   \post      none


 */

extern void RADARDEMO_highAccuRangeProc_delete(
    IN void *handle);


/*!
   \fn     RADARDEMO_highAccuRangeProc_run

   \brief   Range processing, always called per chirp per antenna.

   \param[in]    handle
               Module handle.

   \param[in]    rangeProcInput
               Input signal from ADC and corresponding chirp number.

   \param[out]    rangeProcOutput
               Outputs from range processing.

   \return    errorCode
               Error code.

        \pre       none

   \post      none


 */

RADARDEMO_highAccuRangeProc_errorCode RADARDEMO_highAccuRangeProc_run(
    IN void                                *handle,
    IN RADARDEMO_highAccuRangeProc_input   *rangeProcInput,
    OUT RADARDEMO_highAccuRangeProc_output *rangeProcOutput);

extern void tw_gen_float(float *w, int n);

#endif // RADARDEMO_RANGEPROC_H
