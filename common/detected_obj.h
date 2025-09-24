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

/*
 * detect_obj.h
 *
 */

#ifndef DETECT_OBJ_H_
#define DETECT_OBJ_H_

/*! @brief Maximum number of detected objects by HWA. */
#define MMW_MAX_OBJ_OUT 100

/*! @brief Converts Doppler index to signed variable. Value greater than or equal
 *         half the Doppler FFT size will become negative.
 *         Needed for extended maximum velocity.
 */
#define DOPPLER_IDX_TO_SIGNED(_idx, _fftSize) ((_idx) < (_fftSize) / 2 ? \
                                                   ((int16_t)(_idx)) : \
                                                   ((int16_t)(_idx) - (int16_t)(_fftSize)))

/*! @brief Converts signed Doppler index to unsigned variable (zero to FFT size -1).
 */
#define DOPPLER_IDX_TO_UNSIGNED(_idx, _fftSize) ((_idx) & (_fftSize - 1))

/*!
 *  @brief    Detected object estimated parameters
 *
 */
typedef volatile struct MmwDemo_detectedObj_t
{
    uint16_t rangeIdx; /*!< @brief Range index */
    int16_t  dopplerIdx; /*!< @brief Doppler index. Note that it is changed
                              to signed integer in order to handle extended maximum velocity.
                              Neagative values correspond to the object moving toward
                              sensor, and positive values correspond to the
                              object moving away from the sensor */
    uint16_t peakVal; /*!< @brief Peak value */
    int16_t  x; /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
    int16_t  y; /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
    int16_t  z; /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
} MmwDemo_detectedObj;


#endif /* DETECT_OBJ_H_ */
