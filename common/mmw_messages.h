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

#ifndef MMW_MESSAGES_H
#define MMW_MESSAGES_H

#include "mmw_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief
     *  Message types used in Millimeter Wave Demo for Mailbox communication
     * between MSS and DSS.
     *
     * @details
     *  The enum is used to hold all the messages types used for Mailbox communication
     * between MSS and DSS in mmw Demo.
     */
    typedef enum MmwDemo_message_type_e
    {
        /*! @brief   message types for MSS to DSS communication */
        MMWDEMO_MSS2DSS_GUIMON_CFG = 0xFEED0001,
        MMWDEMO_MSS2DSS_HIGHACCURANGE_CFG,
        MMWDEMO_MSS2DSS_DETOBJ_SHIPPED,
        MMWDEMO_MSS2DSS_ADCBUFCFG,
        MMWDEMO_MSS2DSS_SET_DATALOGGER,

        /*! @brief   message types for DSS to MSS communication */
        MMWDEMO_DSS2MSS_CONFIGDONE = 0xFEED0100,
        MMWDEMO_DSS2MSS_DETOBJ_READY

    } MmwDemo_message_type;

    /**
     * @brief
     *  Message for reporting detected objects from data path.
     *
     * @details
     *  The structure defines the message body for detected objects from from data path.
     */
    typedef struct MmwDemo_detObjMsg_t
    {
        /*! @brief Address of the detected objects matix */
        uint32_t detObjOutAddress;

        /*! @brief size of the detected objects matix in bytes */
        uint32_t detObjOutsize;
    } MmwDemo_detObjMsg;

    /**
     * @brief
     *  Message body used in Millimeter Wave Demo for passing configuration from MSS
     * to DSS.
     *
     * @details
     *  The union defines the message body for various configuration messages.
     */
    typedef union MmwDemo_message_body_u
    {
        /*! @brief   Gui Monitor Selection */
        MmwDemo_GuiMonSel guiMonSel;

        /*! @brief   high accuracy range est module Configuration */
        radarModuleHighAccuConfig highAccuRangeCfg;

        /*! @brief   Detected Objects message */
        MmwDemo_detObjMsg detObj;

        /*! @brief   ADC Buffer configuration */
        MmwDemo_ADCBufCfg adcBufCfg;

        /*! @brief   Datapath output logger setting */
        uint8_t dataLogger;
    } MmwDemo_message_body;

    /**
     * @brief
     *  DSS/MSS communication messages
     *
     * @details
     *  The structure defines the message structure used to commuincate between MSS
     * and DSS.
     */
    typedef struct MmwDemo_message_t
    {
        /*! @brief   message type */
        MmwDemo_message_type type;

        /*! @brief   message length : PROC_TBA does body need to be pointer and not a union structure?*/
        // uint32_t                  len;

        /*! @brief  message body */
        MmwDemo_message_body body;

    } MmwDemo_message;

#ifdef __cplusplus
}
#endif

#endif /* MMW_MESSAGES_H */
