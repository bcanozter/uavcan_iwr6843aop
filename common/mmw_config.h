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

#ifndef MMW_CONFIG_H
#define MMW_CONFIG_H

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
/*#include <chains/RadarReceiverHighAcc_mmSDK/radarProcess.h>*/
#include "radarProcess.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define radarModuleHighAccuConfig RADARDEMO_highAccuRangeProc_config

    /**
     *  @b Description
     *  @n
     *      Structure stores meta information for detected objects.
     *
     *
     *  @details
     *      This structure holds the meta information for detected Objects.
     */
    typedef struct MmwDemo_detOutputHdr_t
    {
        /*! brief   Output buffer magic word */
        uint16_t magicWord[4];

        /*! brief   chirp processing margin in usec */
        uint32_t chirpProcessingMarginInUsec;

        /*! brief   frame processing margin in usec */
        uint32_t frameProcessingMarginInUsec;

        /*! brief   chirp processing margin in usec */
        uint32_t chirpProcessingLoading;

        /*! brief   frame processing margin in usec */
        uint32_t frameProcessingLoading;

        /*! brief   Number of detected Objects */
        uint32_t numDetectedObj;

        /*! brief   Frame number */
        uint32_t frameNumber;

    } MmwDemo_detOutputHdr;

    /**
     * @brief
     *  ADCBUF configuration
     *
     * @details
     *  The structure is used to hold all the relevant configuration
     *  which is used to configure ADCBUF.
     */
    typedef struct MmwDemo_ADCBufCfg_t
    {
        /*! ADCBUF out format:
            0-Complex,
            1-Real */
        uint8_t adcFmt;

        /*! ADCBUF IQ swap selection:
            0-I in LSB, Q in MSB,
            1-Q in LSB, I in MSB */
        uint8_t iqSwapSel;

        /*! ADCBUF channel interleave configuration:
            0-interleaved(not supported on XWR16xx),
            1- non-interleaved */
        uint8_t chInterleave;

        /**
         * @brief   Chirp Threshold configuration used for ADCBUF buffer
         */
        uint8_t chirpThreshold;
    } MmwDemo_ADCBufCfg;

    /**
     * @brief
     *  Millimeter Wave Demo Gui Monitor Selection
     *
     * @details
     *  The structure contains the flags which select what information is sent to gui
     *
     */
    typedef struct MmwDemo_GuiMonSel_t
    {
        /*! @brief   Send list of detected objects (structure outObj) */
        uint8_t detectedObjects;

        /*! @brief   Send range FFT input  */
        uint8_t logRangeInput;

        /*! @brief   dummy */
        uint8_t dummy;

        /*! @brief   log stats */
        uint8_t logStats;
    } MmwDemo_GuiMonSel;

    /**
     * @brief
     *  Millimeter Wave Demo configuration
     *
     * @details
     *  The structure is used to hold all the relevant configuration
     *  which is used to execute the Millimeter Wave Demo.
     */
    typedef struct MmwDemo_Cfg_t
    {
        /*! @brief   CPU Clock Frequency. */
        uint32_t sysClockFrequency;

        /*! @brief   UART Logging Baud Rate. */
        uint32_t loggingBaudRate;

        /*! @brief   UART Command Baud Rate. */
        uint32_t commandBaudRate;

        /*! @brief   mmWave Control Configuration. */
        MMWave_CtrlCfg ctrlCfg;

        /*! @brief   mmWave Open Configuration. */
        MMWave_OpenCfg openCfg;

        /*! @brief   Gui Monitor Selection */
        MmwDemo_GuiMonSel guiMonSel;

        /*! @brief   high accuracy range est module Configuration */
        radarModuleHighAccuConfig highAccuRangeCfg;

        /*! @brief   ADC Buffer Configuration */
        MmwDemo_ADCBufCfg adcBufCfg;

        /*! @brief   Datapath output loggerSetting
                     0 (default): MSS UART logger
                     1: DSS UART logger
         */
        uint8_t dataLogger;
    } MmwDemo_Cfg;

#ifdef __cplusplus
}
#endif

#endif /* MMW_CONFIG_H */
