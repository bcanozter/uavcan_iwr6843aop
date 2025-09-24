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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Demo Include Files */
#include "mss_mmw.h"
/*#include <chains/RadarReceiverHighAcc_mmSDK/mmw_HighAccDemo/common/mmw_messages.h>*/
#include "mmw_messages.h"
#include "mmw_output.h"
#include "mmw_config.h"
#include "detected_obj.h"
#include <ti/common/mmwave_sdk_version.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/* UAVCAN / Canard Include Files*/
#define MmwDemo_debugAssert(expression) {                                      \
    _MmwDemo_debugAssert(expression,      \
             __FILE__, __LINE__);         \
    DebugP_assert(expression);             \
   }
#include "canard.h"
#include <dronecan_msgs.h>
#include <ti/drivers/canfd/canfd.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Clock.h>

#define UAVCAN_TASK_PRIORITY 3
#define UAVCAN_RX_EVENT_TASK_PRIORITY 3

#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_NODE_NAME "org.bco.mmwave"

#define UAVCAN_GETNODEINFO_REQUEST_EVENT (1 << 0)
static uint8_t transfer_id = 0;

CanardInstance canard;
static uint8_t canard_memory_pool[2048];
static uint32_t get_uptime_sec(void);
static uint64_t get_uptime_usec(void);
static void send_getnodeinfo_response(void);
Task_Handle dronecanTask;
Task_Handle uavcanRxTask;
static CanardRxTransfer g_rxTransferInfo;
static Event_Handle g_rxEventHandle;
static GateMutex_Handle gateMutexCanard;
static volatile uint32_t clk_sys_ticks = 0;

void clk0Fxn(UArg arg0);
Clock_Struct clk0Struct;
Clock_Handle clkHandle;

/**************************************************************************
 *************************** MCAN Global Definitions ***************************
 **************************************************************************/

volatile uint32_t gTxDoneFlag = 0, gRxDoneFlag = 0, gParityErrFlag = 0;
volatile uint32_t gTxPkts = 0, gRxPkts = 0, gErrStatusInt = 0;

volatile uint32_t iterationCount = 0U;
uint32_t dataLength = 0U;
uint32_t msgLstErrCnt = 0U;
uint32_t gDisplayStats = 0;
uint8_t rxData[64U];
uint32_t txDataLength, rxDataLength;
CANFD_MCANFrameType frameType = CANFD_MCANFrameType_CLASSIC;
static void MCANAppInitParams(CANFD_MCANInitParams *mcanCfgParams);
CANFD_Handle canHandle;
CANFD_MsgObjHandle txMsgObjHandle;
CANFD_MsgObjHandle rxMsgObjHandle;
CANFD_MCANMsgObjCfgParams txMsgObjectParams;

int32_t Can_Transmit_Schedule(uint32_t msg_id, uint8_t *txmsg, uint32_t len);

int32_t Can_Transmit_Schedule(uint32_t msg_id, uint8_t *txmsg, uint32_t len)
{

    volatile uint32_t index = 0;
    int32_t retVal = 0;
    int32_t errCode = 0;

    if (frameType == CANFD_MCANFrameType_FD)
    {
        Task_sleep(1);

        // transmit 64 bit frames while Tx message is greater than 64
        while (len > 64U)
        {
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, 64U, &txmsg[index], &errCode);
            if (retVal < 0)
            {
                CLI_write("Error: CANFD_MCANFrameType_FD failed [Error code %d]\n", errCode);
            }
            index = index + 64U;
            len = len - 64U;

            Task_sleep(1);
        }
        // transmit remaining data
        retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, len, &txmsg[index], &errCode);
        if (retVal < 0)
        {
            CLI_write("Error: CANFD instance 0 transmit data failed [Error code %d]\n", errCode);
        }
    }
    else
    {
        while (len > 8U)
        {
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, 8U, &txmsg[index], &errCode);
            if (retVal < 0)
            {
                CLI_write("Error: CANFD instance 0 transmit data failed [Error code %d]\n", errCode);
            }
            index = index + 8U;
            len = len - 8U;
        }
        retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);
        if (retVal < 0)
        {
            CLI_write("Error: CANFD instance 0 transmit data failed [Error code %d]\n", errCode);
        }
        while (retVal < 0)
        {
            // CLI_write("Debug: Error transmitting CAN data %x , Errcode %x\n", retVal, errCode);
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);
        }
    }

    return retVal;
}
/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression)
    {
        CLI_write("Exception: %s, line %d.\n", file, line);
    }
}
/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB gMmwMssMCB;

#define SOC_XWR68XX_MSS_MAXNUMHEAPS (RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
#define SOC_XWR68XX_MSS_L1_SCRATCH_SIZE 0x2100U
#define SOC_XWR68XX_MSS_L2_BUFF_SIZE 0x12000U
#define SOC_XWR68XX_MSS_L3RAM_BUFF_SIZE 0x0000U

/*! L2 RAM buffer */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[SOC_XWR68XX_MSS_L2_BUFF_SIZE];

/*! L2 RAM scratch */
#pragma DATA_SECTION(gMmwL1Scratch, ".l2data");
#pragma DATA_ALIGN(gMmwL1Scratch, 8);
uint8_t gMmwL1Scratch[SOC_XWR68XX_MSS_L1_SCRATCH_SIZE];

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
/* CLI Init function */
extern void MmwDemo_CLIInit(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

/* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStart(void);
int32_t MmwDemo_mssDataPathStop(void);

/* mmwave library call back fundtions */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg *ptrCtrlCfg);
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg *ptrCalibrationCfg);
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg *ptrOpenCfg);
static void MmwDemo_mssMmwaveCloseCallbackFxn(void);

void MmwDemo_mssMmwaveStopcallbackFxn(void);
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* MMW demo Task */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1);
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1);
void MmwDemo_transmitProcessedOutput();

/** test */

static void onTransferReceived(CanardInstance *ins,
                               CanardRxTransfer *transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID))
    {
        memset(&g_rxTransferInfo, 0, sizeof(CanardRxTransfer));
        // CLI_write("[DEBUG] GetNodeInfo request from src: %d\n", transfer->source_node_id);
        g_rxTransferInfo.timestamp_usec = transfer->timestamp_usec;
        g_rxTransferInfo.source_node_id = transfer->source_node_id;
        g_rxTransferInfo.data_type_id = transfer->data_type_id;
        g_rxTransferInfo.transfer_id = transfer->transfer_id;
        g_rxTransferInfo.priority = transfer->priority;
        g_rxTransferInfo.transfer_type = transfer->transfer_type;
        g_rxTransferInfo.payload_head = transfer->payload_head;
        g_rxTransferInfo.payload_len = transfer->payload_len;
        g_rxTransferInfo.payload_middle = transfer->payload_middle;
        g_rxTransferInfo.payload_tail = transfer->payload_tail;
        Event_post(g_rxEventHandle, UAVCAN_GETNODEINFO_REQUEST_EVENT);
    }
}
static void uavcan_rx_event_task(UArg arg0, UArg arg1)
{
    uint32_t events;
    int retval;

    while (1)
    {
        events = Event_pend(g_rxEventHandle, UAVCAN_GETNODEINFO_REQUEST_EVENT, Event_Id_NONE, BIOS_WAIT_FOREVER);
        if (events & UAVCAN_GETNODEINFO_REQUEST_EVENT)
        {
            IArg key = GateMutex_enter(gateMutexCanard);
            send_getnodeinfo_response();
            GateMutex_leave(gateMutexCanard, key);
        }
    }
}

static void send_getnodeinfo_response(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;
    memset(&buffer, 0, sizeof(buffer));
    memset(&pkt, 0, sizeof(struct uavcan_protocol_GetNodeInfoResponse));
    struct uavcan_protocol_NodeStatus node_status;
    node_status.uptime_sec = get_uptime_sec();
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    node_status.vendor_specific_status_code = 0;
    pkt.status = node_status;

    pkt.software_version.major = APP_VERSION_MAJOR;
    pkt.software_version.minor = APP_VERSION_MINOR;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0;

    pkt.hardware_version.major = 1;
    pkt.hardware_version.minor = 0;
    const uint8_t id[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0};
    memcpy(pkt.hardware_version.unique_id, id, sizeof(id));
    const uint8_t node_name_len = strlen(APP_NODE_NAME);
    strncpy((char *)pkt.name.data, APP_NODE_NAME, node_name_len);
    pkt.name.len = node_name_len;

    uint32_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           g_rxTransferInfo.source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &g_rxTransferInfo.transfer_id,
                           g_rxTransferInfo.priority,
                           CanardResponse,
                           buffer,
                           total_size);
}
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        }
    }
    return false;
}

static void publish_debug_msg(void)
{
    struct uavcan_protocol_debug_KeyValue msg;
    memset(&msg, 0, sizeof(struct uavcan_protocol_debug_KeyValue));
    uint8_t debug_buffer[UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE];
    memset(debug_buffer, 0, sizeof(debug_buffer));

    msg.value = 123.456f;
    const char debug_msg[] = "Hello World!";
    const uint8_t debug_msg_len = strlen(debug_msg);
    memcpy((char *)msg.key.data, debug_msg, debug_msg_len);
    msg.key.len = debug_msg_len;

    uint32_t debug_len = uavcan_protocol_debug_KeyValue_encode(&msg, debug_buffer);

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    debug_buffer,
                    debug_len);
}

static void broadcast_node_status(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
    struct uavcan_protocol_NodeStatus node_status;
    node_status.uptime_sec = get_uptime_sec();
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    node_status.vendor_specific_status_code = 0;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

static void handle_canard_tx_queue(void)
{
    IArg key = GateMutex_enter(gateMutexCanard);

    canardCleanupStaleTransfers(&canard, get_uptime_usec());
    Task_sleep(1);
    int32_t errCode = 0;

    for (CanardCANFrame *txf = NULL;
         (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        errCode = Can_Transmit_Schedule(txf->id | CANARD_CAN_FRAME_EFF, txf->data, txf->data_len);
        Task_sleep(10);
        if (errCode == 0)
        {
            canardPopTxQueue(&canard);
        }
    }
    GateMutex_leave(gateMutexCanard, key);
}

void print_task_stats(void)
{
    Task_Stat stat;
    // CLI_write("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");
    // Task_stat(gMmwMssMCB, &stat);
    // CLI_write("%20s %12d %12d %12d\n", "Init",
    //           stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(dronecanTask, &stat);
    CLI_write("%20s %12d %12d %12d\n", "dronecan",
              stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(uavcanRxTask, &stat);
    CLI_write("%20s %12d %12d %12d\n", "uavcanrx",
              stat.stackSize, stat.used, stat.stackSize - stat.used);

    CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    CLI_write("%20s %12s %12s %12s\n", " ", "capacity_blocks", "current_usage_blocks", "peak_usage_blocks");
    CLI_write("%20s %12u %12u %12u\n", "canard pool",
              stats.capacity_blocks, stats.current_usage_blocks, stats.peak_usage_blocks);
}

static void uavcan_task(UArg arg0, UArg arg1)
{
    // Init canard
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);
    canardSetLocalNodeID(&canard, 69);

    while (1)
    {
        // publish_debug_msg();
        broadcast_node_status();
        handle_canard_tx_queue();
        print_task_stats();
        Task_sleep(1000);
    }
}

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/** @brief Transmits detection data over UART
 *
 *    The following data is transmitted:
 *    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
 *       and icluding the number of TLV items
 *    TLV Items:
 *    2. If detectedObjects flag is set, pbjOut structure containing range,
 *       doppler, and X,Y,Z location for detected objects,
 *       size = sizeof(objOut_t) * number of detected objects
 *    3. If logMagRange flag is set,  rangeProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    4. If noiseProfile flag is set,  noiseProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    7. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
 *       range cubed matrix, size = number of Rx Azimuth virtual antennas *
 *       number of chirps per frame * sizeof(uint32_t)
 *    8. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
 *       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
 *    9. If statsInfo flag is set, the stats information
 *   @param[in] uartHandle   UART driver handle
 *   @param[in] obj          Pointer data path object MmwDemo_DataPathObj
 */

void MmwDemo_transmitProcessedOutput()
{
    MmwDemo_output_message_header header;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    MmwDemo_detOutputHdr *ptrDetOutputHdr;
    radarProcessOutput_t *outputData;
    MmwDemo_output_message_tl tl[MMWDEMO_OUTPUT_MSG_MAX];
    uint32_t fft1D_length;
    MmwDemo_MSS_DataPathObj *obj = &gMmwMssMCB.mssDataPathObj;
    MmwDemo_GuiMonSel guiMonSel = gMmwMssMCB.cfg.guiMonSel;
    UART_Handle uartHandle = gMmwMssMCB.loggingUartHandle;

    ptrDetOutputHdr = (MmwDemo_detOutputHdr *)&obj->inputInfo.header;
    outputData = (radarProcessOutput_t *)&obj->inputInfo.pointCloudBuf;

    fft1D_length = outputData->fft1DSize;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* Header: */
    header.platform = 0xA6843;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = 1;
    header.version = MMWAVE_SDK_VERSION_BUILD | // DEBUG_VERSION
                     (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                     (MMWAVE_SDK_VERSION_MINOR << 16) |
                     (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    // detectedObjects
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(MmwDemo_detectedObj) * 1 +
                            sizeof(MmwDemo_output_message_dataObjDescr);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    // Range FFT input
    if (guiMonSel.logRangeInput)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = 2 * sizeof(float) * fft1D_length;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    // statsInfo
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1)) / MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles = Pmu_getCount(0);
    header.frameNumber = 0;

    UART_writePolling(uartHandle,
                      (uint8_t *)&header,
                      sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* Send detected Objects */
    {
        MmwDemo_output_message_dataObjDescr descr;
        MmwDemo_detectedObj dummyDetectionOut;
        int32_t tempRange, tempRange1, tempRange2;

        memset((void *)&dummyDetectionOut, 0, sizeof(MmwDemo_detectedObj));

        tempRange = (int32_t)(outputData->rangeEst * 1048576.f);
        tempRange1 = (int32_t)(outputData->rangeEst1 * 1048576.f);
        tempRange2 = (int32_t)(outputData->rangeEst2 * 1048576.f);

        dummyDetectionOut.rangeIdx = (uint16_t)tempRange & 0xFFFF;
        dummyDetectionOut.x = tempRange >> 16;

        dummyDetectionOut.peakVal = (uint16_t)tempRange1 & 0xFFFF;
        dummyDetectionOut.y = tempRange1 >> 16;

        dummyDetectionOut.dopplerIdx = (uint16_t)tempRange2 & 0xFFFF;
        dummyDetectionOut.z = tempRange2 >> 16;

        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));
        /* Send objects descriptor */
        descr.numDetetedObj = 1;
        descr.xyzQFormat = 20;
        UART_writePolling(uartHandle, (uint8_t *)&descr, sizeof(MmwDemo_output_message_dataObjDescr));

        /*Send array of objects */
        UART_writePolling(uartHandle, (uint8_t *)&dummyDetectionOut, sizeof(MmwDemo_detectedObj) * 1);
        tlvIdx++;
    }

    /* Send Range FFT input */
    if (guiMonSel.logRangeInput)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        UART_writePolling(uartHandle, (uint8_t *)outputData->fft1Dinput, 2 * fft1D_length * sizeof(float));
        tlvIdx++;
    }

    /* Send stats information */
    {
        MmwDemo_output_message_stats stats;
        stats.interChirpProcessingMargin = (uint32_t)ptrDetOutputHdr->chirpProcessingMarginInUsec;
        stats.interFrameProcessingMargin = (uint32_t)ptrDetOutputHdr->frameProcessingMarginInUsec;
        stats.interFrameProcessingTime = 0;
        stats.transmitOutputTime = (uint32_t)obj->cycleLog.sendingToUARTTimeCurrInusec;
        stats.activeFrameCPULoad = (uint32_t)ptrDetOutputHdr->chirpProcessingLoading;
        stats.interFrameCPULoad = (uint32_t)ptrDetOutputHdr->frameProcessingLoading;

        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle,
                          (uint8_t *)&stats,
                          tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1));
    if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)padding,
                          numPaddingBytes);
    }

#if 0
                	gMmwMssMCB.mssDataPathObj.inputInfoBuffSize = message.body.detObj.detObjOutsize;

					timeStart = Cycleprofiler_getTimeStamp();
					memcpy((void *)&(gMmwMssMCB.mssDataPathObj.inputInfo),
                			(uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress, SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
							message.body.detObj.detObjOutsize);
                    /*UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress,
                                                           SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                            message.body.detObj.detObjOutsize);*/
					gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec;

					timeStart = Cycleprofiler_getTimeStamp();
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)&(gMmwMssMCB.mssDataPathObj.inputInfo),
							gMmwMssMCB.mssDataPathObj.inputInfoBuffSize);
					gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec;

                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(MmwDemo_message));

                    message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                    if (MmwDemo_mboxWrite(&message) != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }
#endif
}

/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

#if 0
    System_printf ("Debug: BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen);
#endif

    /* Process the received message: */
    switch (msgId)
    {
    case RL_RF_ASYNC_EVENT_MSG:
    {
        /* Received Asychronous Message: */
        switch (asyncSB)
        {
        case RL_RF_AE_CPUFAULT_SB:
        {
            /* Post event to datapath task notify BSS events */
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
            break;
        }
        case RL_RF_AE_ESMFAULT_SB:
        {
            /* Post event to datapath task notify BSS events */
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
            break;
        }
        case RL_RF_AE_INITCALIBSTATUS_SB:
        {
            rlRfInitComplete_t *ptrRFInitCompleteMessage;
            uint32_t calibrationStatus;

            /* Get the RF-Init completion message: */
            ptrRFInitCompleteMessage = (rlRfInitComplete_t *)payload;
            calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

            /* Display the calibration status: */
            CLI_write("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
            break;
        }

        case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
        {
            /* This event is not handled on MSS */
            break;
        }
        case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
        {
            /* Increment the statistics for the number of failed reports */
            gMmwMssMCB.stats.numFailedTimingReports++;

            break;
        }
        case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
        {
            /* Increment the statistics for the number of received calibration reports */
            gMmwMssMCB.stats.numCalibrationReports++;

            break;
        }
        case RL_RF_AE_FRAME_END_SB:
        {
            /*Received Frame Stop async event from BSS.
              No further action required on MSS as it will
              wait for a message from DSS when it is done (MMWDEMO_DSS2MSS_STOPDONE)*/
            break;
        }
        default:
        {
            System_printf("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
            break;
        }
        }
        break;
    }
    default:
    {
        System_printf("Error: Asynchronous message %d is NOT handled\n", msgId);
        break;
    }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked after the configuration
 *      has been used to configure the mmWave link and the BSS. This is applicable only for
 *      the XWR16xx. The BSS can be configured only by the MSS *or* DSS. The callback API is
 *      triggered on the remote execution domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg *ptrCtrlCfg)
{
    /* For mmw Demo, mmwave_config() will always be called from MSS,
       due to the fact CLI is running on MSS, hence this callback won't be called */

    gMmwMssMCB.stats.datapathConfigEvt++;
}
/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg *ptrOpenCfg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveCloseCallbackFxn(void)
{
    return;
}
/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg *ptrCalibrationCfg)
{
    /* Post an event to main data path task.
       This function in only called when mmwave_start() is called on DSS */
    gMmwMssMCB.stats.datapathStartEvt++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStopCallbackFxn(void)
{
    /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMmwMssMCB.stats.datapathStopEvt++;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message *message)
{
    int32_t retVal = -1;

    retVal = Mailbox_write(gMmwMssMCB.peerMailbox, (uint8_t *)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from
 *      Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message message;
    int32_t retVal = 0;
    uint32_t timeStart;

    /* wait for new message and process all the messages received from the peer */
    while (1)
    {
        Semaphore_pend(gMmwMssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMmwMssMCB.peerMailbox, (uint8_t *)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush(gMmwMssMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
            case MMWDEMO_DSS2MSS_DETOBJ_READY:
                /* Got detetced objectes , shipped out through UART */

                gMmwMssMCB.mssDataPathObj.inputInfoBuffSize = message.body.detObj.detObjOutsize;

                memcpy((void *)&(gMmwMssMCB.mssDataPathObj.inputInfo),
                       (uint8_t *)SOC_translateAddress(message.body.detObj.detObjOutAddress, SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL),
                       message.body.detObj.detObjOutsize);

                timeStart = Cycleprofiler_getTimeStamp();
                gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = 1.f;
                MmwDemo_transmitProcessedOutput();
                gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = 2.f;
                gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart) / (float)R4F_CLOCK_MHZ);

#if 0
                    /*UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress,
                                                           SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                            message.body.detObj.detObjOutsize);*/
					gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec;

					timeStart = Cycleprofiler_getTimeStamp();
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)&(gMmwMssMCB.mssDataPathObj.inputInfo),
							gMmwMssMCB.mssDataPathObj.inputInfoBuffSize);
					gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec;
#endif

                /* Send a message to MSS to log the output data */
                memset((void *)&message, 0, sizeof(MmwDemo_message));

                message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                if (MmwDemo_mboxWrite(&message) != 0)
                {
                    System_printf("Error: Mailbox send message id=%d failed \n", message.type);
                }
                break;
            default:
            {
                /* Message not support */
                System_printf("Error: unsupport Mailbox message id=%d\n", message.type);
                break;
            }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback(
    Mbox_Handle handle,
    Mailbox_Type peer)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post(gMmwMssMCB.mboxSemHandle);
}

void MmwDemo_printHeapStats()
{
    Memory_Stats startMemoryStats;

    HeapMem_getStats(heap0, &startMemoryStats);
    System_printf("Debug: System Heap (TCM): Size: %d, Used = %d, Free = %d bytes\n", startMemoryStats.totalSize, startMemoryStats.totalSize - startMemoryStats.totalFreeSize, startMemoryStats.totalFreeSize);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on MSS. After received Configuration from
 *    CLI, this function will start the system configuration process, inclucing mmwaveLink, BSS
 *    and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathConfig(void)
{
    int32_t errCode;
    radarOsal_heapConfig heapconfig[SOC_XWR68XX_MSS_MAXNUMHEAPS];
    /* Setup the calibration frequency: */
    // gMmwMssMCB.cfg.ctrlCfg.freqLimitLow  = 760U;
    // gMmwMssMCB.cfg.ctrlCfg.freqLimitHigh = 810U;

    /* Configure the mmWave module: */
    /*if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
        return -1;
    }
*/
    if (gMmwMssMCB.isMMWaveOpen == false)
    {
        /* NO: Setup the calibration frequency: */
        gMmwMssMCB.cfg.openCfg.freqLimitLow = 600U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 640U;

        /* start/stop async events */
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;
#ifndef SOC_XWR68XX_ES1
        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any
         * monitoring related functionality
         */
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit = 1;
#endif
        /* Open the mmWave module: */
        if (MMWave_open(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
        {
            System_printf("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
            return -1;
        }

        /* mmWave module has been opened. */
        gMmwMssMCB.isMMWaveOpen = true;
    }
    /* Configure the mmWave module: */
    if (MMWave_config(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        System_printf("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
        return -1;
    }

    // heap init
    memset(heapconfig, 0, sizeof(heapconfig));
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType = RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr = NULL; //(int8_t *) &gMmwL3[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize = SOC_XWR68XX_MSS_L3RAM_BUFF_SIZE;
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr = NULL; /* not DDR scratch for TM demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize = 0;    /* not DDR scratch for TM demo  */

    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType = RADARMEMOSAL_HEAPTYPE_LL2;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr = (int8_t *)&gMmwL2[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize = SOC_XWR68XX_MSS_L2_BUFF_SIZE;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr = NULL;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize = 0;

    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType = RADARMEMOSAL_HEAPTYPE_LL1;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr = NULL; /* not used as L1 heap in TM demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize = 0;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr = (int8_t *)&gMmwL1Scratch[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize = SOC_XWR68XX_MSS_L1_SCRATCH_SIZE;

    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapType = RADARMEMOSAL_HEAPTYPE_HSRAM;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAddr = NULL;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize = 0;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchAddr = NULL; /* not HSRAM scratch for TM demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize = 0;    /* not HSRAM scratch for TM demo  */

    if (radarOsal_memInit(&heapconfig[0], SOC_XWR68XX_MSS_MAXNUMHEAPS) == RADARMEMOSAL_FAIL)
    {
        System_printf("Error: radarOsal_memInit fail\n");
        return -1;
    }
    memset(&(gMmwMssMCB.mssDataPathObj.cycleLog), 0, sizeof(gMmwMssMCB.mssDataPathObj.cycleLog));

    MmwDemo_printHeapStats();

    radarOsal_memDeInit();

    System_printf("Debug: MMWDemoMSS mmWave  config succeeded \n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStart(void)
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    /* Initialize the calibration configuration: */
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    /*  calibrationCfg.enableCalibration    = true;
      calibrationCfg.enablePeriodicity    = true;
      calibrationCfg.periodicTimeInFrames = 10U;*/

    /* Start the mmWave module: The configuration has been applied successfully. */
    // if (MMWave_start (gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    // {
    /* Error: Unable to start the mmWave control */
    // System_printf ("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
    // return -1;
    // }

    /* Populate the calibration configuration: */
    // calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.dfeDataOutputMode =
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf("Debug: MMWDemoMSS mmWave Start succeeded \n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStop(void)
{
    int32_t errCode;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_stop(gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf("Error: MMWDemoMSS mmWave Stop failed [Error code %d]\n", errCode);
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute(gMmwMssMCB.ctrlHandle, &errCode) < 0)
            System_printf("Error: mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1)
{
    UInt event;

    /* Data Path management task Main loop */
    while (1)
    {
        event = Event_pend(gMmwMssMCB.eventHandle,
                           Event_Id_NONE,
                           MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS,
                           BIOS_WAIT_FOREVER);

        /************************************************************************
         * CLI event:: SensorStart
         ************************************************************************/

        if (event & MMWDEMO_CLI_SENSORSTART_EVT)
        {
            System_printf("Debug: MMWDemoMSS Received CLI sensorStart Event\n");

            /* Setup the data path: */
            if (MmwDemo_mssDataPathConfig() < 0)
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: SensorStop
         ************************************************************************/
        if (event & MMWDEMO_CLI_SENSORSTOP_EVT)
        {
            if (MmwDemo_mssDataPathStop() < 0)
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: Framestart
         ************************************************************************/
        if (event & MMWDEMO_CLI_FRAMESTART_EVT)
        {
            if (MmwDemo_mssDataPathStart() < 0)
            {
                continue;
            }
        }

        /************************************************************************
         * BSS event:: CPU fault
         ************************************************************************/
        if (event & MMWDEMO_BSS_CPUFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }

        /************************************************************************
         * BSS event:: ESM fault
         ************************************************************************/
        if (event & MMWDEMO_BSS_ESMFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }
    }

    System_printf("Debug: MMWDemoDSS Data path exit\n");
}

//** CANFD Init */

static void MCANAppErrStatusCallback(CANFD_Handle handle, CANFD_Reason reason,
                                     CANFD_ErrStatusResp *errStatusResp)
{
    /*Record the error count */
    gErrStatusInt++;
    return;
}

static uint32_t get_uptime_sec(void)
{
    // uint32_t seconds = ((Clock_getTicks() * Clock_tickPeriod) / 1000000U);
    // CLI_write("get_uptime_sec: %lu",seconds);
    return (uint32_t)(((uint64_t)clk_sys_ticks * Clock_tickPeriod) / 1000000U);
}

static uint64_t get_uptime_usec(void)
{
    return ((uint64_t)clk_sys_ticks * Clock_tickPeriod);
}

#define SOURCE_ID_FROM_ID(x) ((uint8_t)(((x) >> 0U) & 0x7FU))
#define DEST_ID_FROM_ID(x) ((uint8_t)(((x) >> 8U) & 0x7FU))
static void MCANAppCallback(CANFD_MsgObjHandle handle, CANFD_Reason reason)
{
    int32_t errCode, retVal;
    uint32_t id;
    CanardCANFrame rx_frame;
    CANFD_MCANFrameType rxFrameType;
    CANFD_MCANXidType rxIdType;

    if (reason == CANFD_Reason_TX_COMPLETION)
    {
        gTxPkts++;
        gTxDoneFlag = 1;
        return;
    }
    if (reason == CANFD_Reason_RX)
    {
        /* Reset the receive buffer */
        memset(&rxData, 0, sizeof(rxData));
        retVal = CANFD_getData(handle, &id, &rxFrameType, &rxIdType, &rxDataLength, &rxData[0], &errCode);
        if (retVal < 0)
        {
            CLI_write("Error: CAN receive data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
            return;
        }
        if (rxFrameType != frameType)
        {
            CLI_write("Error: CAN received incorrect frame type Sent %d Received %d for iteration %d failed\n", frameType, rxFrameType, iterationCount);
            return;
        }

        if (rxIdType == CANFD_MCANXidType_29_BIT)
        {
            rx_frame.id = id | CANARD_CAN_FRAME_EFF;
            rx_frame.iface_id = 0;
            rx_frame.data_len = rxDataLength;
            memcpy(rx_frame.data, &rxData, rxDataLength);
            retVal = canardHandleRxFrame(&canard, &rx_frame, get_uptime_usec());
            if (retVal < 0)
            {
                // CLI_write("Error: canardHandleRxFrame %d\n", retVal);
                if (retVal == -CANARD_ERROR_RX_MISSED_START)
                {
                    uint64_t dummy_signature;
                    shouldAcceptTransfer(&canard,
                                         &dummy_signature,
                                         extractDataType(rx_frame.id),
                                         extractTransferType(rx_frame.id),
                                         0);
                }
            }
        }

        gRxPkts++;
        gRxDoneFlag = 1;
        return;
    }
    if (reason == CANFD_Reason_TX_CANCELED)
    {
        gTxPkts++;
        gTxDoneFlag = 1;
        gRxDoneFlag = 1;
        return;
    }
}

/**************************************************************************
******************** CAN Parameters initialize Function *****************
**************************************************************************/
static void MCANAppInitParams(CANFD_MCANInitParams *mcanCfgParams)
{
    /*Intialize MCAN Config Params*/
    memset(mcanCfgParams, sizeof(CANFD_MCANInitParams), 0);

    mcanCfgParams->fdMode = 0x0U;
    mcanCfgParams->brsEnable = 0x0U;
    mcanCfgParams->txpEnable = 0x0U;
    mcanCfgParams->efbi = 0x0U;
    mcanCfgParams->pxhddisable = 0x0U;
    mcanCfgParams->darEnable = 0x1U;
    mcanCfgParams->wkupReqEnable = 0x1U;
    mcanCfgParams->autoWkupEnable = 0x1U;
    mcanCfgParams->emulationEnable = 0x0U;
    mcanCfgParams->emulationFAck = 0x0U;
    mcanCfgParams->clkStopFAck = 0x0U;
    mcanCfgParams->wdcPreload = 0x0U;
    mcanCfgParams->tdcEnable = 0x1U;
    mcanCfgParams->tdcConfig.tdcf = 0U;
    mcanCfgParams->tdcConfig.tdco = 8U;
    mcanCfgParams->monEnable = 0x0U;
    mcanCfgParams->asmEnable = 0x0U;
    mcanCfgParams->tsPrescalar = 0x0U;
    mcanCfgParams->tsSelect = 0x0U;
    mcanCfgParams->timeoutSelect = CANFD_MCANTimeOutSelect_CONT;
    mcanCfgParams->timeoutPreload = 0x0U;
    mcanCfgParams->timeoutCntEnable = 0x0U;

    mcanCfgParams->filterConfig.rrfe = 0x0U;
    mcanCfgParams->filterConfig.rrfs = 0x0U;
    mcanCfgParams->filterConfig.anfe = 0x0U;
    mcanCfgParams->filterConfig.anfs = 0x0U;

    mcanCfgParams->msgRAMConfig.lss = 127U;
    mcanCfgParams->msgRAMConfig.lse = 64U;
    mcanCfgParams->msgRAMConfig.txBufNum = 32U;
    mcanCfgParams->msgRAMConfig.txFIFOSize = 0U;
    mcanCfgParams->msgRAMConfig.txBufMode = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOSize = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOWaterMark = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0size = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO0OpMode = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO0waterMark = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1size = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1waterMark = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1OpMode = 64U;

    mcanCfgParams->eccConfig.enable = 1;
    mcanCfgParams->eccConfig.enableChk = 1;
    mcanCfgParams->eccConfig.enableRdModWr = 1;

    mcanCfgParams->errInterruptEnable = 1U;
    mcanCfgParams->dataInterruptEnable = 1U;
    mcanCfgParams->appErrCallBack = MCANAppErrStatusCallback;
    mcanCfgParams->appDataCallBack = MCANAppCallback;
}

/**************************************************************************
*************************** CAN Driver Initialize Function ***********************
**************************************************************************/
void Can_Initialize(void)
{
    int32_t errCode = 0;
    int32_t retVal = 0;
    CANFD_MCANInitParams mcanCfgParams;
    CANFD_MCANBitTimingParams mcanBitTimingParams;
    // CANFD_MCANMsgObjCfgParams rxMsgObjectParams;
    CANFD_MCANRxMsgObjRangeCfgParams rxRangeMsgObjectParams;
    gTxDoneFlag = 0;
    gRxDoneFlag = 0;

    /* Setup the PINMUX to bring out the XWR16xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINE14_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINE14_PADAE, SOC_XWR68XX_PINE14_PADAE_CANFD_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PIND13_PADAD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PIND13_PADAD, SOC_XWR68XX_PIND13_PADAD_CANFD_RX);

    /* Configure the divide value for MCAN source clock */
    SOC_setPeripheralClock(gMmwMssMCB.socHandle, SOC_MODULE_MCAN, SOC_CLKSOURCE_VCLK, 4U, &errCode);

    /* Initialize peripheral memory */
    SOC_initPeripheralRam(gMmwMssMCB.socHandle, SOC_MODULE_MCAN, &errCode);
    MCANAppInitParams(&mcanCfgParams);

    /* Initialize the CANFD 0 driver */
    canHandle = CANFD_init(0, &mcanCfgParams, &errCode);
    if (canHandle == NULL)
    {
        CLI_write("Error: CANFD Module Initialization failed [Error code %d]\n", errCode);
        return;
    }

    /* Configuring 1Mbps and 1Mbps as nominal and data bit-rate respectively
    * Bit Rate = CAN_CLK / (k(PHSEG1 + PROPSEG + PHSEG2 + 1) * BRP)
    Prop seg: 8
    Ph seg 1: 6
    Ph Seg2 : 5
    Sync jump: 1
    BRP(Baud rate Prescaler): 2
    Nominal Bit rate = (40)/(((8+6+5)+1)*BRP) = 1Mhz
    Timing Params for Data Bit rate:
    Prop seg: 1
    Ph seg 1: 1
    Ph Seg2 : 1
    Sync jump: 1
    BRP(Baud rate Prescaler): 2
    Data Bit rate = (40)/(((1+1+1)+1)*BRP) = 5Mhz
    */
    mcanBitTimingParams.nomBrp = 0x2U;
    mcanBitTimingParams.nomPropSeg = 0x8U;
    mcanBitTimingParams.nomPseg1 = 0x6U;
    mcanBitTimingParams.nomPseg2 = 0x5U;
    mcanBitTimingParams.nomSjw = 0x1U;

    mcanBitTimingParams.dataBrp = 0x2U;
    mcanBitTimingParams.dataPropSeg = 0x8U;
    mcanBitTimingParams.dataPseg1 = 0x6U;
    mcanBitTimingParams.dataPseg2 = 0x5U;
    mcanBitTimingParams.dataSjw = 0x1U;

    /* Configure the CAN driver */
    retVal = CANFD_configBitTime(canHandle, &mcanBitTimingParams, &errCode);
    if (retVal < 0)
    {
        CLI_write("Error: CANFD Module configure bit time failed [Error code %d]\n", errCode);
        return;
    }

    /* Setup the transmit message object */
    txMsgObjectParams.direction = CANFD_Direction_TX;
    txMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    txMsgObjectParams.msgIdentifier = 0x1FFFFFFFU;
    txMsgObjHandle = CANFD_createMsgObject(canHandle, &txMsgObjectParams, &errCode);
    if (txMsgObjHandle == NULL)
    {
        CLI_write("Error: CANFD create Tx message object failed [Error code %d]\n", errCode);
        return;
    }

    rxRangeMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    rxRangeMsgObjectParams.startMsgIdentifier = 0x00000000U;
    rxRangeMsgObjectParams.endMsgIdentifier = 0x1FFFFFFFU;

    rxMsgObjHandle = CANFD_createRxRangeMsgObject(canHandle, &rxRangeMsgObjectParams, &errCode);
    if (rxMsgObjHandle == NULL)
    {
        CLI_write(
            "Error: CANFD create range Rx message object failed [Error code %d]\n",
            errCode);
        return;
    }

    CLI_write("Debug: CANFD Instance 0 Initialization was successful\n");
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    MMWave_InitCfg initCfg;
    UART_Params uartParams;
    Task_Params taskParams;
    Semaphore_Params semParams;
    Mailbox_Config mboxCfg;
    Error_Block eb;

    /* Debug Message: */
    System_printf("Debug: MMWDemoMSS Launched the Initialization Task\n");

    /* Initialize CAN */
    Can_Initialize();

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP8_PADBM, SOC_XWR68XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate = gMmwMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone = 1U;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate = gMmwMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId = MAILBOX_CH_ID_0;
    mboxCfg.writeMode = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMmwMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16 * 1024;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwMssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwMssMCB.eventHandle == NULL)
    {
        DebugP_assert(0);
        return;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Populate the init configuration for mmwave library: */
    initCfg.domain = MMWave_Domain_MSS;
    initCfg.socHandle = gMmwMssMCB.socHandle;
    initCfg.eventFxn = MmwDemo_mssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel = CRC_Channel_CH1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn = MmwDemo_mssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn = MmwDemo_mssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_mssMmwaveCloseCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_mssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn = MmwDemo_mssMmwaveStopCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init(&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf("Error: MMWDemoMSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf("Debug: MMWDemoMSS mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync(gMmwMssMCB.ctrlHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 5 * 1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Create a data path management task to handle data Path events
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stackSize = 3 * 1024;
    Task_create(MmwDemo_mssCtrlPathTask, &taskParams, NULL);

    /* DRONECAN TASK */
    GateMutex_Params gateMtxParams;
    GateMutex_Params_init(&gateMtxParams);
    gateMutexCanard = GateMutex_create(&gateMtxParams, NULL);
    if (gateMutexCanard == NULL)
    {
        CLI_write("Error: gateMutexCanard create failed\n");
        MmwDemo_debugAssert(0);
        return;
    }
    Event_Params eventParams;
    Event_Params_init(&eventParams);
    g_rxEventHandle = Event_create(&eventParams, NULL);
    if (g_rxEventHandle == NULL)
    {
        CLI_write("Error: RX Event creation failed!\n");
        return;
    }
    Task_Params_init(&taskParams);
    taskParams.priority = UAVCAN_TASK_PRIORITY;
    taskParams.stackSize = 2 * 1024;
    dronecanTask = Task_create(uavcan_task, &taskParams, NULL);
    Task_Params_init(&taskParams);
    taskParams.priority = UAVCAN_RX_EVENT_TASK_PRIORITY;
    taskParams.stackSize = 2 * 1024;
    uavcanRxTask = Task_create(uavcan_rx_event_task, &taskParams, NULL);
    if (uavcanRxTask == NULL)
    {
        CLI_write("Error: UAVCAN RX event proc task create failed\n");
    }

    /*****************************************************************************
     * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
     * Start CLI to get configuration from user
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Benchmarking Count init
     *****************************************************************************/
    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);

    return;
}

void clk0Fxn(UArg arg0)
{
    clk_sys_ticks = Clock_getTicks();
    // CLI_write("System time in clk0Fxn = %lu\n", (ULong)time);
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main(void)
{
    Clock_Params clkParams;
    Task_Params taskParams;
    int32_t errCode;
    SOC_Cfg socCfg;

    /* Initialize the ESM: */
    ESM_init(0U); // dont clear errors as TI RTOS does it

    /* Initialize and populate the demo MCB */
    memset((void *)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

    /* Initialize the SOC confiugration: */
    memset((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwMssMCB.socHandle = SOC_init(&socCfg, &errCode);
    if (gMmwMssMCB.socHandle == NULL)
    {
        System_printf("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gMmwMssMCB.cfg.loggingBaudRate = 921600;
    gMmwMssMCB.cfg.commandBaudRate = 115200;

    Cycleprofiler_init();

    /* Debug Message: */
    System_printf("**********************************************\n");
    System_printf("Debug: Launching the Millimeter Wave Demo\n");
    System_printf("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MmwDemo_mssInitTask, &taskParams, NULL);
    /* Clock Init */
    Clock_Params_init(&clkParams);
    clkParams.period    = 5000 / Clock_tickPeriod;
    clkParams.startFlag = TRUE;

    /* Construct a periodic Clock Instance */
    Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn, 5000 / Clock_tickPeriod, &clkParams);
    clkHandle = Clock_handle(&clk0Struct);

    Clock_start(clkHandle);
    /* Start BIOS */
    BIOS_start();
    return 0;
}
