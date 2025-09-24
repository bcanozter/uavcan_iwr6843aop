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

#ifndef MSS_MMW_DEMO_H
#define MSS_MMW_DEMO_H

#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>

#include <ti/sysbios/knl/Semaphore.h>

/* MMW Demo Include Files */
//#include <chains/RadarReceiverHighAcc_mmSDK/mmw_HighAccDemo/common/mmw_config.h>
#include "mmw_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*! @brief Version of the MMW Demo. */
#define MMW_VERSION "0.0.0.2"

/*! @brief   sensor start CLI event */
#define MMWDEMO_CLI_SENSORSTART_EVT Event_Id_00

/*! @brief   sensor stop CLI  event */
#define MMWDEMO_CLI_SENSORSTOP_EVT Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define MMWDEMO_CLI_FRAMESTART_EVT Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT Event_Id_04

/* All CLI events */
#define MMWDEMO_CLI_EVENTS (MMWDEMO_CLI_SENSORSTART_EVT | \
                            MMWDEMO_CLI_SENSORSTOP_EVT | \
                            MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS (MMWDEMO_BSS_CPUFAULT_EVT | \
                                  MMWDEMO_BSS_ESMFAULT_EVT)

    extern radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

    /*! @brief R4F cycle profiling structure to accumulate different
        processing times getting the results from DSP and optional algorithms such as tracking, and time
        spent on sending final results to UART */
    typedef struct cycleLog_t_
    {
        float copyResultsTimeCurrInusec; /*!< @brief current time spent on copying results from DSP from shared memory */
        float copyResultsTimeMaxInusec; /*!< @brief maximum time spent on copying results from DSP from shared memory */
        float trackingTimeCurrInusec; /*!< @brief current time spent on tracking */
        float trackingTimeMaxInusec; /*!< @brief maximum time spent on tracking */
        float sendingToUARTTimeCurrInusec; /*!< @brief current time spent on sending final results to UART */
        float sendingToUARTTimeMaxInusec; /*!< @brief maximum time spent on sending final results to UART */
    } cycleLog_t;


    /**
     * @brief
     *  Millimeter Wave Demo statistics
     *
     * @details
     *  The structure is used to hold the statistics information for the
     *  Millimeter Wave demo
     */
    typedef struct MmwDemo_MSS_STATS_t
    {
        /*! @brief   CLI event for sensorStart */
        uint8_t cliSensorStartEvt;

        /*! @brief   CLI event for sensorStop */
        uint8_t cliSensorStopEvt;

        /*! @brief   CLI event for frameStart */
        uint8_t cliFrameStartEvt;

        /*! @brief   Counter which tracks the number of datapath config event detected
         *           The event is triggered in mmwave config callback function */
        uint8_t datapathConfigEvt;

        /*! @brief   Counter which tracks the number of failed calibration reports
         *           The event is triggered by an asynchronous event from the BSS */
        uint32_t numFailedTimingReports;

        /*! @brief   Counter which tracks the number of calibration reports received
         *           The event is triggered by an asynchronous event from the BSS */
        uint32_t numCalibrationReports;

        /*! @brief   Counter which tracks the number of datapath start event  detected
         *           The event is triggered in mmwave start callback function */
        uint8_t datapathStartEvt;

        /*! @brief   Counter which tracks the number of datapath stop event detected
         *           The event is triggered in mmwave stop callback function */
        uint8_t datapathStopEvt;
    } MmwDemo_MSS_STATS;


    /**
     * @brief
     *  Millimeter Wave Demo Data Path Information.
     *
     * @details
     *  The structure is used to hold all the relevant information for
     *  the data path.
     */
    typedef struct MmwDemo_MSS_DataPathObj_t
    {
        /*! @brief   Signal processing output: point cloud  buffer*/
        struct
        {
            MmwDemo_detOutputHdr header;
            radarProcessOutput_t pointCloudBuf;
        } inputInfo;

        uint32_t inputInfoBuffSize;

        cycleLog_t cycleLog;

    } MmwDemo_MSS_DataPathObj;


    /**
     * @brief
     *  Millimeter Wave Demo MCB
     *
     * @details
     *  The structure is used to hold all the relevant information for the
     *  Millimeter Wave demo
     */
    typedef struct MmwDemo_MCB_t
    {
        /*! @brief   Configuration which is used to execute the demo */
        MmwDemo_Cfg cfg;

        /*! * @brief   Handle to the SOC Module */
        SOC_Handle socHandle;

        /*! @brief   UART Logging Handle */
        UART_Handle loggingUartHandle;

        /*! @brief   UART Command Rx/Tx Handle */
        UART_Handle commandUartHandle;

        /*! @brief   This is the mmWave control handle which is used
         * to configure the BSS. */
        MMWave_Handle ctrlHandle;

        /*!@brief   Handle to the peer Mailbox */
        Mbox_Handle peerMailbox;

        /*! @brief   Semaphore handle for the mailbox communication */
        Semaphore_Handle mboxSemHandle;

        /*! @brief   MSS system event handle */
        Event_Handle eventHandle;

        /*! @brief   Handle to the SOC chirp interrupt listener Handle */
        SOC_SysIntListenerHandle chirpIntHandle;

        /*! @brief   Handle to the SOC frame start interrupt listener Handle */
        SOC_SysIntListenerHandle frameStartIntHandle;

        /*! @brief   Data Path object: currently only for receiving data from DSS. Potentially adding other block such as tracking here*/
        MmwDemo_MSS_DataPathObj mssDataPathObj;

        /*! @brief   Has the mmWave module been opened? */
        bool isMMWaveOpen;

        /*! @brief   mmw Demo stats */
        MmwDemo_MSS_STATS stats;
    } MmwDemo_MCB;

    /**************************************************************************
     *************************** Extern Definitions ***************************
     **************************************************************************/
    extern int32_t MmwDemo_mssDataPathConfig(void);
    extern int32_t MmwDemo_mssDataPathStart(void);
    extern int32_t MmwDemo_mssDataPathStop(void);

#ifdef __cplusplus
}
#endif

#endif /* MSS_MMW_DEMO_H */
