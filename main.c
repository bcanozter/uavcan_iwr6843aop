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
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>
#include <ti/demo/xwr64xx/mmw/mmw_output.h>

#include <ti/drivers/canfd/canfd.h>
/* Demo Include Files */
#include <ti/demo/xwr64xx/mmw/mmw.h>
#include <ti/demo/xwr64xx/mmw/data_path.h>
#include <ti/demo/xwr64xx/mmw/mmw_config.h>
#include <ti/demo/xwr64xx/mmw/mmw_res.h>
#include <ti/board/antenna_geometry.h>
#include <ti/demo/utils/mmwdemo_flash.h>

/* UAVCAN / Canard Include Files*/

#include "canard.h"
#include <dronecan_msgs.h>

#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_NODE_NAME "org.bco.mmwave"

#define UAVCAN_GETNODEINFO_REQUEST_EVENT (1 << 0)
static uint8_t transfer_id = 0;

CanardInstance canard;
static uint8_t canard_memory_pool[1024];
static uint32_t get_uptime_sec(void);

static CanardRxTransfer g_rxTransferInfo;
static Event_Handle g_rxEventHandle;
static GateMutex_Handle gateMutexCanard;

/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from BSS
 * that need quick action in real-time.
 *
 * CLI task must be at a lower priority than object detection
 * dpm task priority because the dynamic CLI command handling in the objection detection
 * dpm task assumes CLI task is held back during this processing. The alternative
 * is to use a semaphore between the two tasks.
 */
#define MMWDEMO_CLI_TASK_PRIORITY 3
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY 4
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY 5
#define UAVCAN_TASK_PRIORITY 5
#define UAVCAN_RX_EVENT_TASK_PRIORITY 5

#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY)
#error CLI task priority must be < Object Detection DPM task priority
#endif

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x800
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET 0x800U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET 0x1000U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT 16U

/*! L3 RAM buffer for object detection DPC */
uint8_t gMmwL3[SOC_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

/*! TCM RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_TCM_SIZE (30U * 1024U)
uint8_t gDPC_ObjDetTCM[MMWDEMO_OBJDET_TCM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetTCM, ".DPC_objDetTcmbHeap");

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB gMmwMCB;

/**
 * @brief
 *  Global Variable for LDO BYPASS config, PLease consult your
 * board/EVM user guide before changing the values here
 */
rlRfLdoBypassCfg_t gRFLdoBypassCfg =
    {
        .ldoBypassEnable = 0,   /* 1.0V RF supply 1 and 1.0V RF supply 2 */
        .supplyMonIrDrop = 0,   /* IR drop of 3% */
        .ioSupplyIndicator = 0, /* 3.3 V IO supply */
};

/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_FLASH_SIZE 4096
#define MMWDEMO_CALIB_STORE_MAGIC (0x7CB28DF9U)

MmwDemo_calibData gCalibDataStorage;
#pragma DATA_ALIGN(gCalibDataStorage, 8);

/**************************************************************************
 *************************** MCAN Global Definitions ***************************
 **************************************************************************/

#define CAN_MESSAGE_FRAME_HEADER 0xBU
#define CAN_MESSAGE_PADDING 0xCU
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

typedef enum mmwDemo_can_message_type_e
{
    /*! @brief   List of detected points */
    CAN_MESSAGE_DETECTED_POINTS = 0xD1,

    /*! @brief   Range profile */
    CAN_MESSAGE_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    CAN_MESSAGE_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    CAN_MESSAGE_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    CAN_MESSAGE_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    CAN_MESSAGE_STATS,

    /*! @brief   Side information */
    CAN_MESSAGE_SIDE_INFO,
} mmwDemo_can_message_type;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit(uint8_t taskPriority);

/**************************************************************************
 ************************* UAVCAN / CANARD Functions **********************
 **************************************************************************/
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
            uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
            struct uavcan_protocol_GetNodeInfoResponse pkt;
            memset(&buffer, 0, sizeof(buffer));
            IArg key = GateMutex_enter(gateMutexCanard);
            memset(&pkt, 0, sizeof(pkt));
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
            GateMutex_leave(gateMutexCanard, key);

            if (retval < 0)
            {
                CLI_write("ERROR REQUEST OR RESPOND (RX task) %d\n", retval);
            }
        }
    }
}
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
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
    uint64_t current_usec = (uint64_t)Clock_getTicks() * Clock_tickPeriod;
    canardCleanupStaleTransfers(&canard, current_usec);
    Task_sleep(1);
    int32_t errCode = 0;
    IArg key = GateMutex_enter(gateMutexCanard);
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
        Task_sleep(1000);
    }
}

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);

void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj);
int32_t MmwDemo_dataPathConfig(void);
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathStart(void);
void MmwDemo_dataPathStop(MmwDemo_DataPathObj *obj);

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                     DPC_ObjectDetection_ExecuteResult *result,
                                     MmwDemo_output_message_stats *timingInfo);

void MmwDemo_initTask(UArg arg0, UArg arg1);
void MmwDemo_dataPathTask(UArg arg0, UArg arg1);
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* Calibration save/restore APIs */
static int32_t MmwDemo_calibInit(void);
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData);
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *calibrationData);

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
 *  @b Description
 *  @n
 *      Utility function to set the pending state of configuration.
 *
 *  @param[in] subFrameCfg Pointer to Sub-frame specific configuration
 *  @param[in] offset Configuration structure offset that uniquely identifies the
 *                    configuration to set to the pending state.
 *
 *  @retval None
 */
static void MmwDemo_setSubFramePendingState(MmwDemo_SubFrameCfg *subFrameCfg, uint32_t offset)
{
    switch (offset)
    {
    case MMWDEMO_GUIMONSEL_OFFSET:
        subFrameCfg->objDetDynCfg.isPrepareRangeAzimuthHeatMapPending = 1;
        break;
    case MMWDEMO_CFARCFGRANGE_OFFSET:
        subFrameCfg->objDetDynCfg.isCfarCfgRangePending = 1;
        break;
    case MMWDEMO_CFARCFGDOPPLER_OFFSET:
        subFrameCfg->objDetDynCfg.isCfarCfgDopplerPending = 1;
        break;
    case MMWDEMO_FOVRANGE_OFFSET:
        subFrameCfg->objDetDynCfg.isFovRangePending = 1;
        break;
    case MMWDEMO_FOVDOPPLER_OFFSET:
        subFrameCfg->objDetDynCfg.isFovDopplerPending = 1;
        break;
    case MMWDEMO_FOVAOA_OFFSET:
        subFrameCfg->objDetDynCfg.isFovAoaCfgPending = 1;
        break;
    case MMWDEMO_MULTIOBJBEAMFORMING_OFFSET:
        subFrameCfg->objDetDynCfg.isMultiObjBeamFormingCfgPending = 1;
        break;
    case MMWDEMO_CALIBDCRANGESIG_OFFSET:
        subFrameCfg->objDetDynCfg.isCalibDcRangeSigCfg = 1;
        break;
    case MMWDEMO_STATICCLUTTERREMOFVAL_OFFSET:
        subFrameCfg->objDetDynCfg.isStaticClutterRemovalCfgPending = 1;
        break;
    case MMWDEMO_EXTMAXVEL_OFFSET:
        subFrameCfg->objDetDynCfg.isExtMaxVelCfgPending = 1;
        break;
    case MMWDEMO_ADCBUFCFG_OFFSET:
        subFrameCfg->isAdcBufCfgPending = 1;
        break;
    case MMWDEMO_LVDSSTREAMCFG_OFFSET:
        subFrameCfg->isLvdsStreamCfgPending = 1;
        break;
    default:
        MmwDemo_debugAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all common configuration is pending
 *
 *  @param[in] cfg Pointer to Common configuration
 *
 *  @retval 1 if all common configuration is pending, else return 0.
 */
static uint8_t MmwDemo_isDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCompRxChannelBiasCfgPending == 1) &&
             (cfg->isMeasureRxChannelBiasCfgPending == 1);

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all sub-frame specific dynamic
 *      configuration is pending
 *
 *  @param[in] cfg Pointer to sub-frame specific configuration
 *
 *  @retval 1 if all sub-frame specific dynamic configuration is pending, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCalibDcRangeSigCfg == 1) &&
             (cfg->isCfarCfgDopplerPending == 1) &&
             (cfg->isCfarCfgRangePending == 1) &&
             (cfg->isFovDopplerPending == 1) &&
             (cfg->isFovRangePending == 1) &&
             (cfg->isMultiObjBeamFormingCfgPending == 1) &&
             (cfg->isPrepareRangeAzimuthHeatMapPending == 1) &&
             (cfg->isStaticClutterRemovalCfgPending == 1) &&
             (cfg->isFovAoaCfgPending == 1) &&
             (cfg->isExtMaxVelCfgPending == 1);

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all common configuration is in non-pending (cleared)
 *      state.
 *
 *  @param[in] cfg Pointer to common specific configuration
 *
 *  @retval 1 if all common configuration is in non-pending state, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCommonCfgInNonPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCompRxChannelBiasCfgPending == 0) &&
             (cfg->isMeasureRxChannelBiasCfgPending == 0);

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all sub-frame specific dynamic configuration
 *      is in non-pending (cleared) state.
 *
 *  @param[in] cfg Pointer to common specific configuration
 *
 *  @retval 1 if all sub-frame specific dynamic configuration is in non-pending
 *          state, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCfgInNonPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCalibDcRangeSigCfg == 0) &&
             (cfg->isCfarCfgDopplerPending == 0) &&
             (cfg->isCfarCfgRangePending == 0) &&
             (cfg->isFovDopplerPending == 0) &&
             (cfg->isFovRangePending == 0) &&
             (cfg->isMultiObjBeamFormingCfgPending == 0) &&
             (cfg->isPrepareRangeAzimuthHeatMapPending == 0) &&
             (cfg->isStaticClutterRemovalCfgPending == 0) &&
             (cfg->isFovAoaCfgPending == 0) &&
             (cfg->isExtMaxVelCfgPending == 0);

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending common configuration of Object Detection DPC
 *
 *  @param[in] cfg Object Detection DPC common configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_resetDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    cfg->isCompRxChannelBiasCfgPending = 0;
    cfg->isMeasureRxChannelBiasCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending sub-frame specific dynamic configuration
 *      of Object Detection DPC
 *
 *  @param[in] cfg Object Detection DPC sub-frame dynamic configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_resetDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    cfg->isCalibDcRangeSigCfg = 0;
    cfg->isCfarCfgDopplerPending = 0;
    cfg->isCfarCfgRangePending = 0;
    cfg->isFovDopplerPending = 0;
    cfg->isFovRangePending = 0;
    cfg->isMultiObjBeamFormingCfgPending = 0;
    cfg->isPrepareRangeAzimuthHeatMapPending = 0;
    cfg->isStaticClutterRemovalCfgPending = 0;
    cfg->isFovAoaCfgPending = 0;
    cfg->isExtMaxVelCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending static (non-dynamic) configuration
 *
 */
void MmwDemo_resetStaticCfgPendingState(void)
{
    uint8_t indx;

    for (indx = 0; indx < gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        gMmwMCB.subFrameCfg[indx].isAdcBufCfgPending = 0;
        gMmwMCB.subFrameCfg[indx].isLvdsStreamCfgPending = 0;
    }

    gMmwMCB.isAnaMonCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in pending state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in pending state, else return 0
 */
uint8_t MmwDemo_isAllCfgInPendingState(void)
{
    uint8_t indx, flag = 1;

    for (indx = 0; indx < gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && MmwDemo_isDynObjDetCfgPendingState(&gMmwMCB.subFrameCfg[indx].objDetDynCfg);
        flag = flag && (gMmwMCB.subFrameCfg[indx].isAdcBufCfgPending == 1);
        flag = flag && (gMmwMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 1);
    }

    flag = flag && MmwDemo_isDynObjDetCommonCfgPendingState(&gMmwMCB.dataPathObj.objDetCommonCfg);
    flag = flag && (gMmwMCB.isAnaMonCfgPending == 1);
    return (flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in non-pending (cleared) state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in non-pending state, else return 0
 */
uint8_t MmwDemo_isAllCfgInNonPendingState(void)
{
    uint8_t indx, flag = 1;

    for (indx = 0; indx < gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && MmwDemo_isDynObjDetCfgInNonPendingState(&gMmwMCB.subFrameCfg[indx].objDetDynCfg);
        flag = flag && (gMmwMCB.subFrameCfg[indx].isAdcBufCfgPending == 0);
        flag = flag && (gMmwMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 0);
    }

    flag = flag && (MmwDemo_isDynObjDetCommonCfgInNonPendingState(&gMmwMCB.dataPathObj.objDetCommonCfg) && flag);
    flag = flag && (gMmwMCB.isAnaMonCfgPending == 0);
    return (flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to apply configuration to specified sub-frame
 *
 *  @param[in] srcPtr Pointer to configuration
 *  @param[in] offset Offset of configuration within the parent structure
 *  @param[in] size   Size of configuration
 *  @param[in] subFrameNum Sub-frame Number (0 based) to apply to, broadcast to
 *                         all sub-frames if special code MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if (subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t indx;
        for (indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t)&gMmwMCB.subFrameCfg[indx] + offset), srcPtr, size);
            MmwDemo_setSubFramePendingState(&gMmwMCB.subFrameCfg[indx], offset);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t)&gMmwMCB.subFrameCfg[subFrameNum] + offset), srcPtr, size);
        MmwDemo_setSubFramePendingState(&gMmwMCB.subFrameCfg[subFrameNum], offset);
    }
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to issue MMWave_stop. The DPM_stop will be called
 *      in the BSS frame done call back case.
 *
 *  @retval None
 */
void MmwDemo_MMWave_stop(void)
{
    int32_t errCode;

    DebugP_log0("App: Issuing MMWave_stop\n");

    /* Stop the mmWave module: */
    if (MMWave_stop(gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            CLI_write("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                      mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            MmwDemo_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            CLI_write("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                      mmWaveErrorCode, subsysErrorCode);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
void MmwDemo_sensorStopEpilog(void)
{
    Task_Stat stat;
    Hwi_StackInfo stackInfo;
    Bool stackOverflow;

    /* Print task statistics, note data path has completely stopped due to
     * end of frame, so we can do non-real time processing like prints on
     * console */
    CLI_write("Data Path Stopped (last frame processing done)\n");
    CLI_write("Task Stack Usage (Note: CLI and sensor Management Task not reported) ==========\n");
    CLI_write("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");

    Task_stat(gMmwMCB.taskHandles.initTask, &stat);
    CLI_write("%20s %12d %12d %12d\n", "Init",
              stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMCB.taskHandles.mmwaveCtrl, &stat);
    CLI_write("%20s %12d %12d %12d\n", "Mmwave Control",
              stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMCB.taskHandles.objDetDpmTask, &stat);
    CLI_write("%20s %12d %12d %12d\n", "ObjDet DPM",
              stat.stackSize, stat.used, stat.stackSize - stat.used);

    CLI_write("HWI Stack (same as System Stack) Usage ============\n");
    stackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (stackOverflow == TRUE)
    {
        CLI_write("HWI Stack overflowed\n");
        MmwDemo_debugAssert(0);
    }
    else
    {
        CLI_write("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        CLI_write("%20s %12d %12d %12d\n", " ",
                  stackInfo.hwiStackSize, stackInfo.hwiStackPeak,
                  stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to Setup the HSI Clock. Required for LVDS streaming.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssSetHsiClk(void)
{
    rlDevHsiClk_t hsiClkgs;
    int32_t retVal;

    /*************************************************************************************
     * Setup the HSI Clock through the mmWave Link:
     *************************************************************************************/
    memset((void *)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));

    /* Setup the HSI Clock as per the Radar Interface Document:
     * - This is set to 600Mhz DDR Mode */
    hsiClkgs.hsiClk = 0x9;

    /* Setup the HSI in the radar link: */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1, &hsiClkgs);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Unable to set the HSI clock */
        CLI_write("Error: Setting up the HSI Clock Failed [Error %d]\n", retVal);
        return -1;
    }

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    Task_sleep(HSI_DCA_MIN_DELAY_MSEC);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization.
 *      User need to fill gMmwMCB.cfg.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen if true then issues MMwave_open
 *
 *  @retval  0 if no error, -1 if error
 */
int32_t MmwDemo_openSensor(bool isFirstTimeOpen)
{
    int32_t errCode;
    MMWave_ErrorLevel errorLevel;
    int16_t mmWaveErrorCode;
    int16_t subsysErrorCode;
    int32_t retVal;
    MMWave_CalibrationData calibrationDataCfg;
    MMWave_CalibrationData *ptrCalibrationDataCfg;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {

        CLI_write("Debug: Sending rlRfSetLdoBypassConfig with %d %d %d\n",
                  gRFLdoBypassCfg.ldoBypassEnable,
                  gRFLdoBypassCfg.supplyMonIrDrop,
                  gRFLdoBypassCfg.ioSupplyIndicator);
        retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfLdoBypassCfg_t *)&gRFLdoBypassCfg);
        if (retVal != 0)
        {
            CLI_write("Error: rlRfSetLdoBypassConfig retVal=%d\n", retVal);
            return -1;
        }

        /* Setup the calibration frequency */
        gMmwMCB.cfg.openCfg.freqLimitLow = 600U;
        gMmwMCB.cfg.openCfg.freqLimitHigh = 640U;

        /* start/stop async events */
        gMmwMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMCB.cfg.openCfg.disableFrameStopAsyncEvent = false;

        /* No custom calibration: */
        gMmwMCB.cfg.openCfg.useCustomCalibration = false;
        gMmwMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any
         * monitoring related functionality
         */
        gMmwMCB.cfg.openCfg.calibMonTimeUnit = 1;

        if ((gMmwMCB.calibCfg.saveEnable != 0) &&
            (gMmwMCB.calibCfg.restoreEnable != 0))
        {
            /* Error: only one can be enabled at at time */
            CLI_write("Error: MmwDemo failed with both save and restore enabled.\n");
            return -1;
        }

        if (gMmwMCB.calibCfg.restoreEnable != 0)
        {
            if (MmwDemo_calibRestore(&gCalibDataStorage) < 0)
            {
                /* Error: only one can be enable at at time */
                CLI_write("Error: MmwDemo failed restoring calibration data from flash.\n");
                return -1;
            }

            /*  Boot calibration during restore: Disable calibration for:
                 - Rx gain,
                 - Rx IQMM,
                 - Tx phase shifer,
                 - Tx Power

                 The above calibration data will be restored from flash. Since they are calibrated in a control
                 way to avoid interfaerence and spec violations.
                 In this demo, other bit fields(except the above) are enabled as indicated in customCalibrationEnableMask to perform boot time
                 calibration. The boot time calibration will overwrite the restored calibration data from flash.
                 However other bit fields can be disabled and calibration data can be restored from flash as well.

                 Note: In this demo, calibration masks are enabled for all bit fields when "saving" the data.
            */
            gMmwMCB.cfg.openCfg.useCustomCalibration = true;
            gMmwMCB.cfg.openCfg.customCalibrationEnableMask = 0x1F0U;

            calibrationDataCfg.ptrCalibData = &gCalibDataStorage.calibData;
            calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStorage.phaseShiftCalibData;
            ptrCalibrationDataCfg = &calibrationDataCfg;
        }
        else
        {
            ptrCalibrationDataCfg = NULL;
        }

        /* Open the mmWave module: */
        if (MMWave_open(gMmwMCB.ctrlHandle, &gMmwMCB.cfg.openCfg, ptrCalibrationDataCfg, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            CLI_write("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                      mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /* Save calibration data in flash */
        if (gMmwMCB.calibCfg.saveEnable != 0)
        {

            retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &gCalibDataStorage.calibData);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Calibration data restore failed */
                CLI_write("MSS demo failed rlRfCalibDataStore with Error[%d]\n", retVal);
                return -1;
            }

#if (defined(SOC_XWR18XX) || defined(SOC_XWR68XX))

            /* update txIndex in all chunks to get data from all Tx.
               This should be done regardless of num TX channels enabled in MMWave_OpenCfg_t::chCfg or number of Tx
               application is interested in. Data for all existing Tx channels should be retrieved
               from RadarSS and in the order as shown below.
               RadarSS will return non-zero phase shift values for all the channels enabled via
               MMWave_OpenCfg_t::chCfg and zero phase shift values for channels disabled in MMWave_OpenCfg_t::chCfg */
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[0].txIndex = 0;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[1].txIndex = 1;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[2].txIndex = 2;

            /* Basic validation passed: Restore the phase shift calibration data */
            retVal = rlRfPhShiftCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &(gCalibDataStorage.phaseShiftCalibData));
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Phase shift Calibration data restore failed */
                CLI_write("MSS demo failed rlRfPhShiftCalibDataStore with Error[%d]\n", retVal);
                return retVal;
            }
#endif
            /* Save data in flash */
            retVal = MmwDemo_calibSave(&gMmwMCB.calibCfg.calibDataHdr, &gCalibDataStorage);
            if (retVal < 0)
            {
                return retVal;
            }
        }

        /*Set up HSI clock*/
        if (MmwDemo_mssSetHsiClk() < 0)
        {
            CLI_write("Error: MmwDemo_mssSetHsiClk failed.\n");
            return -1;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to configure sensor.
 *      User need to fill gMmwMCB.cfg.ctrlCfg and add profiles/chirp to mmWave
 *      before calling this function
 *
 *  @retval  0 if no error, error code otherwise
 */
int32_t MmwDemo_configSensor(void)
{
    int32_t errCode = 0;

    /* Configure the mmWave module: */
    if (MMWave_config(gMmwMCB.ctrlHandle, &gMmwMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        CLI_write("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                  mmWaveErrorCode, subsysErrorCode);
        goto exit;
    }
    else
    {
        errCode = MmwDemo_dataPathConfig();
        goto exit;
    }

exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval  0 if no error, -1 if error
 */
int32_t MmwDemo_startSensor(void)
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    /*****************************************************************************
     * Data path :: start data path first - this will pend for DPC to ack
     *****************************************************************************/
    MmwDemo_dataPathStart();

    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Initialize the calibration configuration: */
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = gMmwMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    DebugP_log0("App: MMWave_start Issued\n");

    CLI_write("Starting Sensor (issuing MMWave_start)\n");

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error/Warning: Unable to start the mmWave module */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        CLI_write("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
        /* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
           assert here and re-start from the beginning. For now, choosing the latter path */
        MmwDemo_debugAssert(0);
        return -1;
    }

    /*****************************************************************************
     * The sensor has been started successfully. Switch on the LED
     *****************************************************************************/
    GPIO_write(gMmwMCB.cfg.platformCfg.SensorStatusGPIO, 1U);

    gMmwMCB.sensorStartCount++;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to convert the CFAR threshold
 *      from a CLI encoded dB value to a linear value
 *      as expected by the CFAR DPU
 *
 *  @param[in] codedCfarVal CFAR threshold in dB as encoded in the CLI
 *  @param[in] numVirtualAntennas Number of virtual antennas
 *
 *  @retval
 *      CFAR threshold in linear format
 */
static uint16_t MmwDemo_convertCfarToLinear(uint16_t codedCfarVal, uint8_t numVirtualAntennas)
{
    uint16_t linearVal;
    float dbVal, linVal;

    /* dbVal is a float value from 0-100dB. It needs to
    be converted to linear scale..
    First, recover float dbVal that was encoded in CLI. */
    dbVal = (float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);

    /* Now convert it to linear value according to the following:
    linear_value = dB_value * (256 / 6) * (numVirtualAntennas / (2^ ceil(log2(numVirtualAntennas)))) .
    */
    linVal = dbVal * (256.0 / 6.0) * ((float)numVirtualAntennas / (float)(1 << mathUtils_ceilLog2(numVirtualAntennas)));

    linearVal = (uint16_t)linVal;
    return (linearVal);
}

/**
 *  @b Description
 *  @n
 *      Stops the RF and datapath for the sensor. Blocks until both operation are not complete
 *      Prints epilog at the end
 *
 *  @retval  None
 */
void MmwDemo_stopSensor()
{
    int32_t errCode;

    MmwDemo_MMWave_stop();

    /* Wait until DPM_stop is completed */
    Semaphore_pend(gMmwMCB.DPMstopSemHandle, BIOS_WAIT_FOREVER);

    /* Delete any active streaming session */
    if (gMmwMCB.lvdsStream.hwSessionHandle != NULL)
    {
        /* Evaluate need to deactivate h/w session:
         * One sub-frame case:
         *   if h/w only enabled, deactivation never happened, hence need to deactivate
         *   if h/w and s/w both enabled, then s/w would leave h/w activated when it is done
         *   so need to deactivate
         *   (only s/w enabled cannot be the case here because we are checking for non-null h/w session)
         * Multi sub-frame case:
         *   Given stop, we must have re-configured the next sub-frame by now which is next of the
         *   last sub-frame i.e we must have re-configured sub-frame 0. So if sub-frame 0 had
         *   h/w enabled, then it is left in active state and need to deactivate. For all
         *   other cases, h/w was already deactivated when done.
         */
        if ((gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) ||
            ((gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) &&
             (gMmwMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)))
        {
            if (CBUFF_deactivateSession(gMmwMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                CLI_write("CBUFF_deactivateSession failed with errorCode = %d\n", errCode);
                MmwDemo_debugAssert(0);
            }
        }
        MmwDemo_LVDSStreamDeleteHwSession();
    }

    /* Delete s/w session if it exists. S/w session never needs to be deactivated in stop because
     * it always (unconditionally) deactivates itself upon completion.
     */
    if (gMmwMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }

    MmwDemo_sensorStopEpilog();

    /* The sensor has been stopped successfully. Switch off the LED */
    GPIO_write(gMmwMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

    gMmwMCB.sensorStopCount++;

    /* print for user */
    CLI_write("Sensor has been stopped: startCount: %d stopCount %d\n",
              gMmwMCB.sensorStartCount, gMmwMCB.sensorStopCount);
}

/**
 *  @b Description
 *  @n
 *  This function is called at the init time from @ref MmwDemo_initTask.
 *  It initializes drivers: ADCBUF, HWA, EDMA, and semaphores used
 *  by  @ref MmwDemo_dataPathTask
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj)
{
    MmwDemo_dataPathObjInit(obj);

    /* Initialize HWA */
    MmwDemo_hwaInit(obj);

    /* Initialize EDMA */
    MmwDemo_edmaInit(obj);
}

/**
 *  @b Description
 *  @n
 *      Opens HWA, EDMA and ADCBUF drivers
 *
 *  @param[in] obj pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj)
{
    MmwDemo_hwaOpen(obj, gMmwMCB.socHandle);
    MmwDemo_edmaOpen(obj);
    obj->adcbufHandle = MmwDemo_ADCBufOpen(gMmwMCB.socHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *
 *  @param[in] subFrameCfg Pointer to sub-frame config
 *  @param[in] numChirpsPerChirpEvent number of chirps per chirp event
 *  @param[in] validProfileIdx valid profile index
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx)
{
    MmwDemo_AnaMonitorCfg *ptrAnaMonitorCfg;
    ADCBuf_CQConf cqConfig;
    rlRxSatMonConf_t *ptrSatMonCfg;
    rlSigImgMonConf_t *ptrSigImgMonCfg;
    int32_t retVal;
    uint16_t cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &gMmwMCB.anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &gMmwMCB.cqSatMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        if (ptrSatMonCfg->profileIndx != validProfileIdx)
        {
            CLI_write("Error: Saturation monitoring (globally) enabled but not configured for profile(%d)\n",
                      validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if (retVal != 0)
        {
            CLI_write("Error: rlRfRxIfSatMonConfig returns error = %d for profile(%d)\n",
                      retVal, ptrSatMonCfg->profileIndx);
            goto exit;
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &gMmwMCB.cqSigImgMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        if (ptrSigImgMonCfg->profileIndx != validProfileIdx)
        {
            CLI_write("Error: Sig/Image monitoring (globally) enabled but not configured for profile(%d)\n",
                      validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if (retVal != 0)
        {
            CLI_write("Error: rlRfRxSigImgMonConfig returns error = %d for profile(%d)\n",
                      retVal, ptrSigImgMonCfg->profileIndx);
            goto exit;
        }
    }

    retVal = mmwDemo_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        CLI_write("Error: rlRfAnaMonConfig returns error = %d\n", retVal);
        goto exit;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0;                                /* 16bit for mmw demo */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET; /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* Address should be 16 bytes aligned */

        retVal = ADCBuf_control(gMmwMCB.dataPathObj.adcbufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            CLI_write("Error: MMWDemoDSS Unable to configure the CQ\n");
            MmwDemo_debugAssert(0);
        }
    }

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* This is for 16bit format in mmw demo, signal/image band data has 2 bytes/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* This is for 16bit format in mmw demo, saturation data has one byte/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      0 if no error, error code otherwise.
 */
int32_t MmwDemo_dataPathConfig(void)
{
    int32_t errCode;
    MMWave_CtrlCfg *ptrCtrlCfg;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;
    MmwDemo_SubFrameCfg *subFrameCfg;
    int8_t subFrameIndx;
    MmwDemo_RFParserOutParams RFparserOutParams;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMCB.cfg.ctrlCfg;

    gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);

    DebugP_log0("App: Issuing Pre-start Common Config IOCTL\n");

    gMmwMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMCB.socHandle, &errCode);
    if (errCode < 0)
    {
        CLI_write("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        goto exit;
    }

    /* Copy antenna geometry definition */
#if defined(XWR68XX_AOP_ANTENNA_PATTERN)
    extern ANTDEF_AntGeometry gAntDef_IWR6843AOP;
    dataPathObj->objDetCommonCfg.preStartCommonCfg.antDef = gAntDef_IWR6843AOP;
#else
    extern ANTDEF_AntGeometry gAntDef_default;
    dataPathObj->objDetCommonCfg.preStartCommonCfg.antDef = gAntDef_default;
#endif

    /* DPC pre-start common config */
    errCode = DPM_ioctl(dataPathObj->objDetDpmHandle,
                        DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                        &dataPathObj->objDetCommonCfg.preStartCommonCfg,
                        sizeof(DPC_ObjectDetection_PreStartCommonCfg));

    if (errCode < 0)
    {
        CLI_write("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    MmwDemo_resetDynObjDetCommonCfgPendingState(&dataPathObj->objDetCommonCfg);

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the MmwDemo_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for (subFrameIndx = gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames - 1; subFrameIndx >= 0;
         subFrameIndx--)
    {
        subFrameCfg = &gMmwMCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                               &gMmwMCB.cfg.openCfg, ptrCtrlCfg,
                                               &subFrameCfg->adcBufCfg,
                                               gMmwMCB.rfFreqScaleFactor,
                                               false /* no BPM in 64xx demo */);

        /* if number of doppler chirps is too low, interpolate to be able to detect
         * better with CFAR tuning. E.g. a 2-pt FFT will be problematic in terms
         * of distinguishing direction of motion */
        if (RFparserOutParams.numDopplerChirps <= 4)
        {
            RFparserOutParams.dopplerStep = RFparserOutParams.dopplerStep / (8 / RFparserOutParams.numDopplerBins);
            RFparserOutParams.numDopplerBins = 8;
        }

        if (errCode != 0)
        {
            CLI_write("Error: MmwDemo_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }
        {
            DPC_ObjectDetection_PreStartCfg objDetPreStartCfg;
            DPC_ObjectDetection_StaticCfg *staticCfg = &objDetPreStartCfg.staticCfg;

            subFrameCfg->numRangeBins = RFparserOutParams.numRangeBins;
            /* Workaround for range DPU limitation for FFT size 1024 and 12 virtual antennas case*/
            if ((RFparserOutParams.numVirtualAntennas == 12) && (RFparserOutParams.numRangeBins == 1024))
            {
                subFrameCfg->numRangeBins = 1022;
                RFparserOutParams.numRangeBins = 1022;
            }

            subFrameCfg->numDopplerBins = RFparserOutParams.numDopplerBins;
            subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
            subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
            subFrameCfg->objDetDynCfg.dynCfg.prepareRangeAzimuthHeatMap = subFrameCfg->guiMonSel.rangeAzimuthHeatMap;
            subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
            subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
            subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;

            errCode = MmwDemo_ADCBufConfig(gMmwMCB.dataPathObj.adcbufHandle,
                                           gMmwMCB.cfg.openCfg.chCfg.rxChannelEn,
                                           subFrameCfg->numChirpsPerChirpEvent,
                                           subFrameCfg->adcBufChanDataSize,
                                           &subFrameCfg->adcBufCfg,
                                           &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
            if (errCode < 0)
            {
                CLI_write("Error: ADCBuf config failed with error [%d]\n", errCode);
                goto exit;
            }

            errCode = MmwDemo_configCQ(subFrameCfg, RFparserOutParams.numChirpsPerChirpEvent,
                                       RFparserOutParams.validProfileIdx);

            if (errCode < 0)
            {
                goto exit;
            }

            /* DPC pre-start config */
            {
                int32_t i;

                objDetPreStartCfg.subFrameNum = subFrameIndx;

                /* Fill static configuration */
                staticCfg->ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
                staticCfg->ADCBufData.dataProperty.adcBits = 2; /* 16-bit */

                /* only complex format supported */
                MmwDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0);

                if (subFrameCfg->adcBufCfg.iqSwapSel == 1)
                {
                    staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_IMRE;
                }
                else
                {
                    staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_REIM;
                }
                if (subFrameCfg->adcBufCfg.chInterleave == 0)
                {
                    staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_INTERLEAVE_MODE;
                }
                else
                {
                    staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
                }
                staticCfg->ADCBufData.dataProperty.numAdcSamples = RFparserOutParams.numAdcSamples;
                staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
                staticCfg->ADCBufData.dataProperty.numRxAntennas = RFparserOutParams.numRxAntennas;
                staticCfg->ADCBufData.dataSize = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
                staticCfg->dopplerStep = RFparserOutParams.dopplerStep;
                staticCfg->isValidProfileHasOneTxPerChirp = RFparserOutParams.validProfileHasOneTxPerChirp;
                staticCfg->numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
                staticCfg->numDopplerBins = RFparserOutParams.numDopplerBins;
                staticCfg->numDopplerChirps = RFparserOutParams.numDopplerChirps;
                staticCfg->numRangeBins = RFparserOutParams.numRangeBins;
                staticCfg->numTxAntennas = RFparserOutParams.numTxAntennas;
                staticCfg->numVirtualAntAzim = RFparserOutParams.numVirtualAntAzim;
                staticCfg->numVirtualAntElev = RFparserOutParams.numVirtualAntElev;
                staticCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
                staticCfg->rangeStep = RFparserOutParams.rangeStep;
                staticCfg->centerFreq = RFparserOutParams.centerFreq;

                /* Current 64xx/68xx SOC has higher receive level as compared to 18xx and hence using higher value for
                 * fftOutputDivShift to avoid overflow when converting from 24-bit to 16-bit
                 * TODO: Future RadarSS firmware should be evaluated to assess if these settings are correct
                 */
                if (RFparserOutParams.numRangeBins >= 1022)
                {
                    staticCfg->rangeFFTtuning.fftOutputDivShift = 1;
                    /* scale only 2 stages */
                    staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 2;
                }
                else if (RFparserOutParams.numRangeBins == 512)
                {
                    staticCfg->rangeFFTtuning.fftOutputDivShift = 2;
                    /* scale last stage */
                    staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 1;
                }
                else
                {
                    staticCfg->rangeFFTtuning.fftOutputDivShift = 3;
                    /* no scaling needed as ADC data is 16-bit and we have 8 bits to grow */
                    staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 0;
                }

                for (i = 0; i < RFparserOutParams.numRxAntennas; i++)
                {
                    staticCfg->rxAntOrder[i] = RFparserOutParams.rxAntOrder[i];
                }
                for (i = 0; i < RFparserOutParams.numTxAntennas; i++)
                {
                    staticCfg->txAntOrder[i] = RFparserOutParams.txAntOrder[i];
                }

                /* Convert CFAR threshold value */
                subFrameCfg->objDetDynCfg.dynCfg.cfarCfgRange.thresholdScale =
                    MmwDemo_convertCfarToLinear(subFrameCfg->objDetDynCfg.dynCfg.cfarCfgRange.thresholdScale,
                                                staticCfg->numVirtualAntennas);

                subFrameCfg->objDetDynCfg.dynCfg.cfarCfgDoppler.thresholdScale =
                    MmwDemo_convertCfarToLinear(subFrameCfg->objDetDynCfg.dynCfg.cfarCfgDoppler.thresholdScale,
                                                staticCfg->numVirtualAntennas);

                /* Fill dynamic configuration */
                objDetPreStartCfg.dynCfg = subFrameCfg->objDetDynCfg.dynCfg;

                DebugP_log1("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\n", subFrameIndx);

                /* send pre-start config */
                errCode = DPM_ioctl(dataPathObj->objDetDpmHandle,
                                    DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                    &objDetPreStartCfg,
                                    sizeof(DPC_ObjectDetection_PreStartCfg));

                MmwDemo_resetDynObjDetCfgPendingState(&subFrameCfg->objDetDynCfg);

                if (errCode < 0)
                {
                    CLI_write("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                    goto exit;
                }
            }
        }
    }

exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the data path manager and
 *      underlying processing chain. It will wait for the processing chain to ack
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathStart(void)
{
    int32_t errCode;

    DebugP_log0("App: Issuing DPM_start\n");

    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmwMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        MmwDemo_configLVDSHwData(0);
    }

    /* Start the DPM Profile: */
    if ((errCode = DPM_start(gMmwMCB.dataPathObj.objDetDpmHandle)) < 0)
    {
        /* Error: Unable to start the profile */
        CLI_write("Error: Unable to start the DPM [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
    }

    /* wait until start completed */
    Semaphore_pend(gMmwMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);

    DebugP_log0("App: DPM_start Done (post Semaphore_pend on reportFxn reporting start)\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to Stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathStop(MmwDemo_DataPathObj *obj)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_stop\n");

    retVal = DPM_stop(obj->objDetDpmHandle);
    if (retVal < 0)
    {
        CLI_write("DPM_stop failed[Error code %d]\n", retVal);
        MmwDemo_debugAssert(0);
    }
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
        if (MMWave_execute(gMmwMCB.ctrlHandle, &errCode) < 0)
        {
            CLI_write("Error: mmWave control execution failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert(0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
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
 *      Always return 0
 */
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

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
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_ESMFAULT_SB:
        {
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_ANALOG_FAULT_SB:
        {
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_INITCALIBSTATUS_SB:
        {
            rlRfInitComplete_t *ptrRFInitCompleteMessage;
            uint32_t calibrationStatus;

            /* Get the RF-Init completion message: */
            ptrRFInitCompleteMessage = (rlRfInitComplete_t *)payload;
            calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;

            /* Display the calibration status: */
            CLI_write("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
            break;
        }
        case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
        {
            gMmwMCB.stats.frameTriggerReady++;
            break;
        }
        case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
        {
            gMmwMCB.stats.failedTimingReports++;
            break;
        }
        case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
        {
            gMmwMCB.stats.calibrationReports++;
            break;
        }
        case RL_RF_AE_FRAME_END_SB:
        {
            gMmwMCB.stats.sensorStopped++;

            DebugP_log0("App: BSS stop (frame end) received\n");

            MmwDemo_dataPathStop(&gMmwMCB.dataPathObj);
            break;
        }
        default:
        {
            CLI_write("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
            break;
        }
        }
        break;
    }
    /* Async Event from MMWL */
    case RL_MMWL_ASYNC_EVENT_MSG:
    {
        switch (asyncSB)
        {
        case RL_MMWL_AE_MISMATCH_REPORT:
        {
            /* link reports protocol error in the async report from BSS */
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_MMWL_AE_INTERNALERR_REPORT:
        {
            /* link reports internal error during BSS communication */
            MmwDemo_debugAssert(0);
            break;
        }
        }
        break;
    }
    default:
    {
        CLI_write("Error: Asynchronous message %d is NOT handled\n", msgId);
        break;
    }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_DPC_ObjectDetection_reportFxn(
    DPM_Report reportType,
    uint32_t instanceId,
    int32_t errCode,
    uint32_t arg0,
    uint32_t arg1)
{

    /* Only errors are logged on the console: */
    if (errCode != 0)
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        CLI_write("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                  reportType, errCode, arg0, arg1);
        DebugP_assert(0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
    case DPM_Report_IOCTL:
    {
        /*****************************************************************
         * DPC has been configured without an error:
         * - This is an indication that the profile configuration commands
         *   went through without any issues.
         *****************************************************************/
        DebugP_log1("App: DPM Report IOCTL, command = %d\n", arg0);

        if (arg0 == DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG)
        {
            DPC_ObjectDetection_PreStartCfg *cfg;
            DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage *memUsage;
            Memory_Stats stats;

            Memory_getStats(NULL, &stats);

            /* Get memory usage from preStartCfg */
            cfg = (DPC_ObjectDetection_PreStartCfg *)arg1;
            memUsage = &cfg->memUsage;

            CLI_write(" ========== Memory Stats ==========\n");
            CLI_write("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
            CLI_write("%20s %12d %12d %12d %12d\n", "System Heap(TCM)",
                      memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                      memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                      memUsage->SystemHeapDPCUsed);

            CLI_write("%20s %12d %12d %12d\n", "L3",
                      sizeof(gMmwL3),
                      memUsage->L3RamUsage,
                      sizeof(gMmwL3) - memUsage->L3RamUsage);

            CLI_write("%20s %12d %12d %12d\n", "TCM",
                      sizeof(gDPC_ObjDetTCM),
                      memUsage->CoreLocalRamUsage,
                      sizeof(gDPC_ObjDetTCM) - memUsage->CoreLocalRamUsage);
        }

        break;
    }
    case DPM_Report_DPC_STARTED:
    {
        /*****************************************************************
         * DPC has been started without an error:
         * - notify sensor management task that DPC is started.
         *****************************************************************/
        DebugP_log0("App: DPM Report DPC Started\n");
        Semaphore_post(gMmwMCB.DPMstartSemHandle);
        break;
    }
    case DPM_Report_NOTIFY_DPC_RESULT:
    {
        /* we cannot be getting this because we are operating in Local domain,
         * it is only meant for remote/distributed domain.
         */
        MmwDemo_debugAssert(0);
        break;
    }
    case DPM_Report_DPC_ASSERT:
    {
        DPM_DPCAssert *ptrAssert;

        /*****************************************************************
         * DPC Fault has been detected:
         * - This implies that the DPC has crashed.
         * - The argument0 points to the DPC assertion information
         *****************************************************************/
        ptrAssert = (DPM_DPCAssert *)arg0;
        CLI_write("Obj Det DPC Exception: %s, line %d.\n", ptrAssert->fileName,
                  ptrAssert->lineNum);
        break;
    }
    case DPM_Report_DPC_STOPPED:
    {
        /*****************************************************************
         * DPC has been stopped without an error:
         * - This implies that the DPC can either be reconfigured or
         *   restarted.
         *****************************************************************/
        DebugP_log0("App: DPM Report DPC Stopped\n");
        Semaphore_post(gMmwMCB.DPMstopSemHandle);
        break;
    }
    case DPM_Report_DPC_INFO:
    {
        /* ObjDetHwa DPC does not support this IOCTL */
        break;
    }
    default:
    {
        DebugP_assert(0);
        break;
    }
    }
    return;
}

void MmwDemo_measurementResultOutput(DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg)
{
    CLI_write("compRangeBiasAndRxChanPhase");
    CLI_write(" %.7f", compRxChanCfg->rangeBias);
    int32_t i;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        CLI_write(" %.5f", (float)compRxChanCfg->rxChPhaseComp[i].real / 32768.);
        CLI_write(" %.5f", (float)compRxChanCfg->rxChPhaseComp[i].imag / 32768.);
    }
    CLI_write("\n");
}

/**
 *  @b Description
 *  @n
 *      Utility function to get temperature report from front end and
 *      save it in global structure.
 *
 *  @retval None
 */
void MmwDemo_getTemperatureReport()
{
    /* Get Temerature report */
    gMmwMCB.temperatureStats.tempReportValid = rlRfGetTemperatureReport(RL_DEVICE_MAP_INTERNAL_BSS,
                                                                        (rlRfTempData_t *)&gMmwMCB.temperatureStats.temperatureReport);
}

/** @brief Transmits detection data over UART
 *
 *    The following data is transmitted:
 *    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
 *       and including the number of TLV items
 *    TLV Items:
 *    2. If detectedObjects flag is 1 or 2, DPIF_PointCloudCartesian structure containing
 *       X,Y,Z location and velocity for detected objects,
 *       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
 *    3. If detectedObjects flag is 1, DPIF_PointCloudSideInfo structure containing SNR
 *       and noise for detected objects,
 *       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
 *    4. If logMagRange flag is set,  rangeProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    5. If noiseProfile flag is set,  noiseProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    6. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
 *       range cubed matrix, size = number of Rx Azimuth virtual antennas *
 *       number of chirps per frame * sizeof(uint32_t)
 *    7. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
 *       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
 *    8. If statsInfo flag is set, the stats information
 *   @param[in] uartHandle   UART driver handle
 *   @param[in] result       Pointer to result from object detection DPC processing
 *   @param[in] timingInfo   Pointer to sub-frame object stats that contains timing info
 */
void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                     DPC_ObjectDetection_ExecuteResult *result,
                                     MmwDemo_output_message_stats *timingInfo)
{
    MmwDemo_output_message_header header;
    MmwDemo_GuiMonSel *pGuiMonSel;
    MmwDemo_SubFrameCfg *subFrameCfg;
    uint32_t tlvIdx = 0;
    uint32_t i;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    uint16_t *detMatrix = (uint16_t *)result->detMatrix.data;

    MmwDemo_output_message_tl tl[MMWDEMO_OUTPUT_MSG_MAX];

    /* Get subframe configuration */
    subFrameCfg = &gMmwMCB.subFrameCfg[result->subFrameIdx];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &subFrameCfg->guiMonSel;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* Header: */
    header.platform = 0xA6843;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = result->numObjOut;
    header.version = MMWAVE_SDK_VERSION_BUILD | // DEBUG_VERSION
                     (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                     (MMWAVE_SDK_VERSION_MINOR << 16) |
                     (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    if ((pGuiMonSel->detectedObjects == 1) || (pGuiMonSel->detectedObjects == 2) &&
                                                  (result->numObjOut > 0))
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudCartesian) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    /* Side info */
    if ((pGuiMonSel->detectedObjects == 1) && (result->numObjOut > 0))
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->logMagRange)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
#if defined(USE_2D_AOA_DPU)
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP;
#else
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
#endif
        tl[tlvIdx].length = result->azimuthStaticHeatMapSize * sizeof(cmplx16ImRe_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeDopplerHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        tl[tlvIdx].length = subFrameCfg->numRangeBins * subFrameCfg->numDopplerBins * sizeof(uint16_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;

        MmwDemo_getTemperatureReport();
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_temperatureStats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    /* Fill header */
    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1)) / MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles = Pmu_getCount(0);
    header.frameNumber = result->stats->frameStartIntCounter;
    header.subFrameNumber = result->subFrameIdx;

    UART_writePolling(uartHandle,
                      (uint8_t *)&header,
                      sizeof(MmwDemo_output_message_header));

    // Can_Transmit_Schedule(CAN_MESSAGE_FRAME_HEADER,
    //                      (uint8_t *)&header,
    //                      sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* Send detected Objects */
    if ((pGuiMonSel->detectedObjects == 1) || (pGuiMonSel->detectedObjects == 2) &&
                                                  (result->numObjOut > 0))
    {
        /* Send the TLV header */
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        /*Send array of objects */
        UART_writePolling(uartHandle, (uint8_t *)result->objOut,
                          sizeof(DPIF_PointCloudCartesian) * result->numObjOut);

        /* Send the TLV header */
        // Can_Transmit_Schedule(CAN_MESSAGE_DETECTED_POINTS,
        //                              (uint8_t *)&tl[tlvIdx], //
        //                              sizeof(MmwDemo_output_message_tl));
        // /*Send array of objects */
        // Can_Transmit_Schedule(CAN_MESSAGE_DETECTED_POINTS,
        //                           (uint8_t*)result->objOut,
        //                           sizeof(DPIF_PointCloudCartesian) * result->numObjOut);

        tlvIdx++;
    }

    /* Send detected Objects Side Info */
    if ((pGuiMonSel->detectedObjects == 1) && (result->numObjOut > 0))
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        /*Send array of objects */
        UART_writePolling(uartHandle, (uint8_t *)result->objOutSideInfo,
                          sizeof(DPIF_PointCloudSideInfo) * result->numObjOut);

        /* Send the TLV header */
        // Can_Transmit_Schedule(CAN_MESSAGE_SIDE_INFO,
        //                              (uint8_t *)&tl[tlvIdx],
        //                              sizeof(MmwDemo_output_message_tl));
        // /*Send array of objects */
        // Can_Transmit_Schedule(CAN_MESSAGE_SIDE_INFO,
        //                           (uint8_t*)result->objOut,
        //                           sizeof(DPIF_PointCloudCartesian) * result->numObjOut);

        tlvIdx++;
    }

    // NOTE: If the framePeriodicity is not high enough, this will crash when accessing detMatrix
    /* Send Range profile */
    if (pGuiMonSel->logMagRange)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        //        /* Send the TLV header */
        //        Can_Transmit_Schedule(CAN_MESSAGE_RANGE_PROFILE,
        //                                     (uint8_t *)&tl[tlvIdx],
        //                                     sizeof(MmwDemo_output_message_tl));

        for (i = 0; i < subFrameCfg->numRangeBins; i++)
        {
            UART_writePolling(uartHandle,
                              (uint8_t *)&detMatrix[i * subFrameCfg->numDopplerBins],
                              sizeof(uint16_t));

            //            /* Send the TLV header */
            //            Can_Transmit_Schedule(CAN_MESSAGE_RANGE_PROFILE,
            //                                      (uint8_t*)&detMatrix[i * subFrameCfg->numDopplerBins],
            //                                      sizeof(uint16_t));
        }
        tlvIdx++;
    }

    // NOTE: If the framePeriodicity is not high enough, this will crash when accessing detMatrix
    /* Send noise profile */
    if (pGuiMonSel->noiseProfile)
    {
        uint32_t maxDopIdx = subFrameCfg->numDopplerBins / 2 - 1;
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        //        /* Send the TLV header */
        //        Can_Transmit_Schedule(CAN_MESSAGE_NOISE_PROFILE,
        //                                     (uint8_t *)&tl[tlvIdx],
        //                                     sizeof(MmwDemo_output_message_tl));

        for (i = 0; i < subFrameCfg->numRangeBins; i++)
        {
            UART_writePolling(uartHandle,
                              (uint8_t *)&detMatrix[i * subFrameCfg->numDopplerBins + maxDopIdx],
                              sizeof(uint16_t));

            //            Can_Transmit_Schedule(CAN_MESSAGE_NOISE_PROFILE,
            //                                (uint8_t*)&detMatrix[i * subFrameCfg->numDopplerBins + maxDopIdx],
            //                                sizeof(uint16_t));
        }
        tlvIdx++;
    }

    // NOTE: If the framePeriodicity is not high enough, this will crash when accessing azimuthStaticHeatMap
    /* Send data for static azimuth heatmap */
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));

        UART_writePolling(uartHandle,
                          (uint8_t *)result->azimuthStaticHeatMap,
                          result->azimuthStaticHeatMapSize * sizeof(cmplx16ImRe_t));

        //        /* Send the TLV header */
        //        Can_Transmit_Schedule(CAN_MESSAGE_AZIMUT_STATIC_HEAT_MAP,
        //                                     (uint8_t *)&tl[tlvIdx],
        //                                     sizeof(MmwDemo_output_message_tl));
        //        /*Send array of objects */
        //        Can_Transmit_Schedule(CAN_MESSAGE_AZIMUT_STATIC_HEAT_MAP,
        //                              (uint8_t *) result->azimuthStaticHeatMap,
        //                              result->azimuthStaticHeatMapSize * sizeof(cmplx16ImRe_t));

        tlvIdx++;
    }

    // NOTE: If the framePeriodicity is not high enough, this will crash when accessing detMatrix
    /* Send data for range/Doppler heatmap */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle,
                          (uint8_t *)detMatrix,
                          tl[tlvIdx].length);

        //        /* Send the TLV header */
        //        Can_Transmit_Schedule(CAN_MESSAGE_RANGE_DOPPLER_HEAT_MAP,
        //                                     (uint8_t *)&tl[tlvIdx],
        //                                     sizeof(MmwDemo_output_message_tl));
        //        /*Send array of objects */
        //        Can_Transmit_Schedule(CAN_MESSAGE_RANGE_DOPPLER_HEAT_MAP,
        //                              (uint8_t*)detMatrix,
        //                              tl[tlvIdx].length);

        tlvIdx++;
    }

    /* Send stats information */
    if (pGuiMonSel->statsInfo == 1)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle,
                          (uint8_t *)timingInfo,
                          tl[tlvIdx].length);

        // /* Send the TLV header */
        // Can_Transmit_Schedule(CAN_MESSAGE_STATS,
        //                              (uint8_t *)&tl[tlvIdx],
        //                              sizeof(MmwDemo_output_message_tl));
        // /*Send array of objects */
        // Can_Transmit_Schedule(CAN_MESSAGE_STATS,
        //                       (uint8_t*)timingInfo,
        //                       tl[tlvIdx].length);

        tlvIdx++;
        UART_writePolling(uartHandle,
                          (uint8_t *)&tl[tlvIdx],
                          sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle,
                          (uint8_t *)&gMmwMCB.temperatureStats,
                          tl[tlvIdx].length);

        // /* Send the TLV header */
        // Can_Transmit_Schedule(CAN_MESSAGE_STATS,
        //                              (uint8_t *)&tl[tlvIdx],
        //                              sizeof(MmwDemo_output_message_tl));
        // /*Send array of objects */
        // Can_Transmit_Schedule(CAN_MESSAGE_STATS,
        //                       (uint8_t*)&gMmwMCB.temperatureStats,
        //                       tl[tlvIdx].length);

        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1));
    if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling(uartHandle,
                          (uint8_t *)padding,
                          numPaddingBytes);

        // Can_Transmit_Schedule(CAN_MESSAGE_PADDING,
        //                           padding,
        //                           numPaddingBytes);
    }

    // CLI_write("Debug: Sent CAN Message!\n");
}

/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx Current sub-frame index
 *  @param[in] numSubFrames Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t MmwDemo_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return (nextIndx);
}

/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx Current sub-frame index
 *  @param[in] numSubFrames Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
static uint8_t MmwDemo_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return (prevIndx);
}

/**
 *  @b Description
 *  @n
 *      Processes any pending dynamic configuration commands for the specified
 *      sub-frame by fanning out to the respective DPUs using IOCTL interface, and
 *      resets (clears) the pending state after processing.
 *
 *  @param[in] subFrameIndx Sub-frame index of desired sub-frame to process
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx)
{
    int32_t retVal = 0;

    MmwDemo_DPC_ObjDet_CommonCfg *commonCfg = &gMmwMCB.dataPathObj.objDetCommonCfg;
    MmwDemo_DPC_ObjDet_DynCfg *subFrameCfg = &gMmwMCB.subFrameCfg[subFrameIndx].objDetDynCfg;
    uint8_t numVirtualAntennas = gMmwMCB.subFrameCfg[subFrameIndx].numVirtualAntennas;

    /* perform globals ones if first sub-frame */
    if (subFrameIndx == 0)
    {
        if (commonCfg->isMeasureRxChannelBiasCfgPending == 1)
        {
            retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                               DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE,
                               &commonCfg->preStartCommonCfg.measureRxChannelBiasCfg,
                               sizeof(DPC_ObjectDetection_MeasureRxChannelBiasCfg));
            if (retVal != 0)
            {
                goto exit;
            }
            commonCfg->isMeasureRxChannelBiasCfgPending = 0;
        }
        if (commonCfg->isCompRxChannelBiasCfgPending == 1)
        {
            retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                               DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE,
                               &commonCfg->preStartCommonCfg.compRxChanCfg,
                               sizeof(DPU_AoAProc_compRxChannelBiasCfg));
            if (retVal != 0)
            {
                goto exit;
            }
            commonCfg->isCompRxChannelBiasCfgPending = 0;
        }
    }

    /* Perform sub-frame specific ones */
    if (subFrameCfg->isCalibDcRangeSigCfg == 1)
    {
        DPC_ObjectDetection_CalibDcRangeSigCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.calibDcRangeSigCfg;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG,
                           &cfg,
                           sizeof(DPC_ObjectDetection_CalibDcRangeSigCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCalibDcRangeSigCfg = 0;
    }
    if (subFrameCfg->isCfarCfgDopplerPending == 1)
    {
        DPC_ObjectDetection_CfarCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        /* Update with correct threshold value based on number of virtual antennas */
        subFrameCfg->dynCfg.cfarCfgDoppler.thresholdScale =
            MmwDemo_convertCfarToLinear(subFrameCfg->dynCfg.cfarCfgDoppler.thresholdScale,
                                        numVirtualAntennas);

        cfg.cfg = subFrameCfg->dynCfg.cfarCfgDoppler;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_CFAR_DOPPLER_CFG,
                           &cfg,
                           sizeof(DPC_ObjectDetection_CfarCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCfarCfgDopplerPending = 0;
    }
    if (subFrameCfg->isCfarCfgRangePending == 1)
    {
        DPC_ObjectDetection_CfarCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        /* Update with correct threshold value based on number of virtual antennas */
        subFrameCfg->dynCfg.cfarCfgRange.thresholdScale =
            MmwDemo_convertCfarToLinear(subFrameCfg->dynCfg.cfarCfgRange.thresholdScale,
                                        numVirtualAntennas);

        cfg.cfg = subFrameCfg->dynCfg.cfarCfgRange;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_CFAR_RANGE_CFG,
                           &cfg,
                           sizeof(DPC_ObjectDetection_CfarCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCfarCfgRangePending = 0;
    }
    if (subFrameCfg->isFovDopplerPending == 1)
    {
        DPC_ObjectDetection_fovDopplerCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.fovDoppler;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_FOV_DOPPLER,
                           &cfg,
                           sizeof(DPC_ObjectDetection_fovDopplerCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovDopplerPending = 0;
    }
    if (subFrameCfg->isFovRangePending == 1)
    {
        DPC_ObjectDetection_fovRangeCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.fovRange;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_FOV_RANGE,
                           &cfg,
                           sizeof(DPC_ObjectDetection_fovRangeCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovRangePending = 0;
    }
    if (subFrameCfg->isMultiObjBeamFormingCfgPending == 1)
    {
        DPC_ObjectDetection_MultiObjBeamFormingCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.multiObjBeamFormingCfg;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_MULTI_OBJ_BEAM_FORM_CFG,
                           &cfg,
                           sizeof(DPC_ObjectDetection_MultiObjBeamFormingCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isMultiObjBeamFormingCfgPending = 0;
    }
    if (subFrameCfg->isPrepareRangeAzimuthHeatMapPending == 1)
    {
        DPC_ObjectDetection_RangeAzimuthHeatMapCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.prepareRangeAzimuthHeatMap = subFrameCfg->dynCfg.prepareRangeAzimuthHeatMap;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_RANGE_AZIMUTH_HEAT_MAP,
                           &cfg,
                           sizeof(DPC_ObjectDetection_RangeAzimuthHeatMapCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isPrepareRangeAzimuthHeatMapPending = 0;
    }
    if (subFrameCfg->isStaticClutterRemovalCfgPending == 1)
    {
        DPC_ObjectDetection_StaticClutterRemovalCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.staticClutterRemovalCfg;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_STATICCLUTTER_REMOVAL_CFG,
                           &cfg,
                           sizeof(DPC_ObjectDetection_StaticClutterRemovalCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isStaticClutterRemovalCfgPending = 0;
    }
    if (subFrameCfg->isFovAoaCfgPending == 1)
    {
        DPC_ObjectDetection_fovAoaCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.fovAoaCfg;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_FOV_AOA,
                           &cfg,
                           sizeof(DPC_ObjectDetection_fovAoaCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovAoaCfgPending = 0;
    }
    if (subFrameCfg->isExtMaxVelCfgPending == 1)
    {
        DPC_ObjectDetection_extMaxVelCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dynCfg.extMaxVelCfg;
        retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                           DPC_OBJDET_IOCTL__DYNAMIC_EXT_MAX_VELOCITY,
                           &cfg,
                           sizeof(DPC_ObjectDetection_extMaxVelCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isExtMaxVelCfgPending = 0;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Transmit user data over LVDS interface.
 *
 *  @param[in]  subFrameIndx Sub-frame index
 *  @param[in]  dpcResults   pointer to DPC result
 *
 */
void MmwDemo_transferLVDSUserData(uint8_t subFrameIndx,
                                  DPC_ObjectDetection_ExecuteResult *dpcResults)
{
    int32_t errCode;

    /* Delete previous SW session if it exists. SW session is being
       reconfigured every frame because number of detected objects
       may change from frame to frame which implies that the size of
       the streamed data may change. */
    if (gMmwMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }

    /* Configure SW session for this subframe */
    if (MmwDemo_LVDSStreamSwConfig(dpcResults->numObjOut,
                                   dpcResults->objOut,
                                   dpcResults->objOutSideInfo) < 0)
    {
        CLI_write("Failed LVDS stream SW configuration for sub-frame %d\n", subFrameIndx);
        MmwDemo_debugAssert(0);
        return;
    }

    /* Populate user data header that will be streamed out*/
    gMmwMCB.lvdsStream.userDataHeader.frameNum = dpcResults->stats->frameStartIntCounter;
    gMmwMCB.lvdsStream.userDataHeader.detObjNum = dpcResults->numObjOut;
    gMmwMCB.lvdsStream.userDataHeader.subFrameNum = (uint16_t)dpcResults->subFrameIdx;

    /* If SW LVDS stream is enabled, start the session here. User data will immediately
       start to stream over LVDS.*/
    if (CBUFF_activateSession(gMmwMCB.lvdsStream.swSessionHandle, &errCode) < 0)
    {
        CLI_write("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      DPM Execution Task. DPM execute results are processed here:
 *      a) Transmits results through UART port.
 *      b) Updates book-keeping code for timing info.
 *      c) Notifies DPC that results have been exported (using DPC IOCTL command)
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_DPC_ObjectDetection_dpmTask(UArg arg0, UArg arg1)
{
    int32_t retVal;
    DPM_Buffer resultBuffer;
    DPC_ObjectDetection_ExecuteResultExportedInfo exportInfo;
    DPC_ObjectDetection_ExecuteResult *result;

    while (1)
    {
        /* Execute the DPM module: */
        retVal = DPM_execute(gMmwMCB.dataPathObj.objDetDpmHandle, &resultBuffer);
        if (retVal < 0)
        {
            CLI_write("Error: DPM execution failed [Error code %d]\n", retVal);
            MmwDemo_debugAssert(0);
        }
        else
        {
            if (resultBuffer.size[0] == sizeof(DPC_ObjectDetection_ExecuteResult))
            {
                volatile uint32_t startTime, currentInterFrameProcessingEndTime;
                MmwDemo_SubFrameStats *currSubFrameStats, *prevSubFrameStats;
                uint16_t dummyRxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

                uint8_t numSubFrames;
                uint8_t nextSubFrameIndx, prevSubFrameIndx, currSubFrameIndx;

                result = (DPC_ObjectDetection_ExecuteResult *)resultBuffer.ptrBuffer[0];
                currSubFrameIndx = result->subFrameIdx;
                currSubFrameStats = &gMmwMCB.subFrameStats[currSubFrameIndx];

                currentInterFrameProcessingEndTime = Cycleprofiler_getTimeStamp();
                currSubFrameStats->outputStats.interFrameProcessingTime =
                    (currentInterFrameProcessingEndTime - result->stats->interFrameStartTimeStamp) / R4F_CLOCK_MHZ;

                numSubFrames = gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames;

                /* Update the processing end margin, note this is of the previous sub-frame
                 * and should work even when there is only 1 sub-frame (or non-advanced mode)
                 * as long as end time stamp is updated after using it. Note that
                 * "result->stats->subFramePreparationCycles" is of the previous sub-frame's
                 * preparation for the next (which is the current here) because sub-frame
                 * preparation happens when DPC processes the results exported IOCTL which
                 * is issued after all the result handling here. However, we also store
                 * this after using for debug purposes (to be able to see all sub-frames switching cycles
                 * in real-time)
                 */
                prevSubFrameIndx = MmwDemo_getPrevSubFrameIndx(currSubFrameIndx, numSubFrames);
                prevSubFrameStats = &gMmwMCB.subFrameStats[prevSubFrameIndx];
                prevSubFrameStats->outputStats.interFrameProcessingMargin =
                    (result->stats->frameStartTimeStamp - prevSubFrameStats->interFrameProcessingEndTime) / R4F_CLOCK_MHZ -
                    (result->stats->subFramePreparationCycles / R4F_CLOCK_MHZ + prevSubFrameStats->subFramePreparationTime) -
                    prevSubFrameStats->pendingConfigProcTime;

                /* Below is only for debug purposes */
                prevSubFrameStats->objDetSubFramePreparationTime = result->stats->subFramePreparationCycles / R4F_CLOCK_MHZ;

                /* This must be after above prevSubFrame processing */
                currSubFrameStats->interFrameProcessingEndTime = currentInterFrameProcessingEndTime;

                startTime = Cycleprofiler_getTimeStamp();

                /* Send out of CLI the range bias and phase config measurement if it was enabled. */
                if (gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg.enabled == 1)
                {
                    if (result->compRxChanBiasMeasurement != NULL)
                    {
                        MmwDemo_measurementResultOutput(result->compRxChanBiasMeasurement);
                    }
                    else
                    {
                        /* DPC is not ready to ship the measurement results */
                    }
                }

                if (gMmwMCB.subFrameCfg[currSubFrameIndx].lvdsStreamCfg.dataFmt !=
                    MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
                {
                    /* check Edma errors (which are considered fatal) for the current sub-frame's
                     * h/w session that is expected to be completed by now */
                    MmwDemo_checkEdmaErrors(MMW_LVDS_STREAM_EDMA_INSTANCE);

                    /* Pend for completion of h/w session, generally this will not wait
                     * because of time spent doing inter-frame processing is expected to
                     * be bigger than the transmission of the h/w session */
                    Semaphore_pend(gMmwMCB.lvdsStream.hwFrameDoneSemHandle, BIOS_WAIT_FOREVER);
                }

                /* Transfer data on LVDS if s/w session is enabled for the current sub-frame */
                if (gMmwMCB.subFrameCfg[currSubFrameIndx].lvdsStreamCfg.isSwEnabled == 1)
                {
                    MmwDemo_transferLVDSUserData(currSubFrameIndx, result);
                }

                MmwDemo_transmitProcessedOutput(gMmwMCB.loggingUartHandle, result,
                                                &currSubFrameStats->outputStats);

                /* Wait until s/w session is complete. We expect the LVDS transmission of
                 * s/w session to be completed by now because the UART transmission above is slower.
                 * Doing the wait immediately after starting the transmission above MmwDemo_transferLVDSUserData
                 * will serialize the LVDS and UART transfers so it is better to do after UART
                 * transmission (which is blocking call i.e UART transmission is completed when we exit
                 * out of above MmwDemo_transmitProcessedOutput). Note we cannot replace below code
                 * with check for previous sub-frame s/w session completion before the
                 * MmwDemo_transferLVDSUserData above because we need to ensure that
                 * current sub-frame/frame's contents are not being read during the
                 * next sub-frame/frame transmission, presently the data that is being
                 * transmitted is not double buffered to allow this */
                if (gMmwMCB.subFrameCfg[currSubFrameIndx].lvdsStreamCfg.isSwEnabled == 1)
                {
                    /* check Edma errors (which are considered fatal) for the s/w session
                     * that is expected to be completed by now */
                    MmwDemo_checkEdmaErrors(MMW_LVDS_STREAM_EDMA_INSTANCE);

                    /* Pend completion of s/w session, no wait is expected here */
                    Semaphore_pend(gMmwMCB.lvdsStream.swFrameDoneSemHandle, BIOS_WAIT_FOREVER);
                }

                currSubFrameStats->outputStats.transmitOutputTime =
                    (Cycleprofiler_getTimeStamp() - startTime) / R4F_CLOCK_MHZ;

                startTime = Cycleprofiler_getTimeStamp();

                /* For non-advanced frame case:
                 *   process all pending dynamic config commands.
                 * For advanced-frame case:
                 *  Process next sub-frame related pending dynamic config commands.
                 *  If the next sub-frame was the first sub-frame of the frame,
                 *  then process common (sub-frame independent) pending dynamic config
                 *  commands. */
                nextSubFrameIndx = MmwDemo_getNextSubFrameIndx(currSubFrameIndx,
                                                               numSubFrames);
                retVal = MmwDemo_processPendingDynamicCfgCommands(nextSubFrameIndx);
                if (retVal != 0)
                {
                    CLI_write("Error: Executing Pending Dynamic Configuration Commands for nextSubFrameIndx = %d [Error code %d]\n",
                              nextSubFrameIndx, retVal);
                    MmwDemo_debugAssert(0);
                }

                currSubFrameStats->pendingConfigProcTime =
                    (Cycleprofiler_getTimeStamp() - startTime) / R4F_CLOCK_MHZ;

                /* Configure ADC for next sub-frame */
                if (numSubFrames > 1)
                {
                    MmwDemo_SubFrameCfg *nextSubFrameCfg;

                    startTime = Cycleprofiler_getTimeStamp();

                    nextSubFrameCfg = &gMmwMCB.subFrameCfg[nextSubFrameIndx];
                    retVal = MmwDemo_ADCBufConfig(gMmwMCB.dataPathObj.adcbufHandle,
                                                  gMmwMCB.cfg.openCfg.chCfg.rxChannelEn,
                                                  nextSubFrameCfg->numChirpsPerChirpEvent,
                                                  nextSubFrameCfg->adcBufChanDataSize,
                                                  &nextSubFrameCfg->adcBufCfg,
                                                  &dummyRxChanOffset[0]);
                    if (retVal < 0)
                    {
                        CLI_write("Error: ADCBuf config failed with error [%d]\n", retVal);
                        MmwDemo_debugAssert(0);
                    }

                    /* Configure HW LVDS stream for this subframe? */
                    if (nextSubFrameCfg->lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
                    {
                        /* check Edma errors (which are considered fatal) for any previous session, even
                         * though we have checked for a previous s/w session, if s/w session weren't
                         * enabled, then this will check for previous h/w session related Edma errors */
                        MmwDemo_checkEdmaErrors(MMW_LVDS_STREAM_EDMA_INSTANCE);

                        MmwDemo_configLVDSHwData(nextSubFrameIndx);
                    }

                    currSubFrameStats->subFramePreparationTime =
                        (Cycleprofiler_getTimeStamp() - startTime) / R4F_CLOCK_MHZ;
                }
                else
                {
                    currSubFrameStats->subFramePreparationTime = 0;
                }

                /* indicate result consumed and end of frame/sub-frame processing */
                exportInfo.subFrameIdx = currSubFrameIndx;
                retVal = DPM_ioctl(gMmwMCB.dataPathObj.objDetDpmHandle,
                                   DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                                   &exportInfo,
                                   sizeof(DPC_ObjectDetection_ExecuteResultExportedInfo));
                if (retVal < 0)
                {
                    CLI_write("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                              retVal);
                    MmwDemo_debugAssert(0);
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during inter-frame
 *
 *  @param[in] subFrameIndx Sub-frame index of the sub-frame during which processing
 *             this function was called.
 *
 *  @retval None
 */
void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    MmwDemo_SubFrameStats *stats;

    stats = &gMmwMCB.subFrameStats[subFrameIndx];

    Load_update();
    stats->outputStats.interFrameCPULoad = Load_getCPULoad();
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of
 *      inter-frame/inter-sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during active frame
 *      (chirping)
 *
 *  @param[in] subFrameIndx Sub-frame index of the sub-frame during which processing
 *             this function was called.
 *
 *  @retval None
 */
void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    MmwDemo_SubFrameStats *stats;

    stats = &gMmwMCB.subFrameStats[subFrameIndx];

    Load_update();
    stats->outputStats.activeFrameCPULoad = Load_getCPULoad();
}

/**
 *  @b Description
 *  @n
 *      Platform specific hardware initialization.
 *
 *  @retval
 *      Not Applicable.
 */

void MmwDemo_platformInit(MmwDemo_platformCfg *config)
{
    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the PINMUX:
     * - for QSPI Flash
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR12_PADAP, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR12_PADAP, SOC_XWR68XX_PINR12_PADAP_QSPI_CLK);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP11_PADAQ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP11_PADAQ, SOC_XWR68XX_PINP11_PADAQ_QSPI_CSN);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR13_PADAL, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR13_PADAL, SOC_XWR68XX_PINR13_PADAL_QSPI_D0);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN12_PADAM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN12_PADAM, SOC_XWR68XX_PINN12_PADAM_QSPI_D1);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR14_PADAN, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR14_PADAN, SOC_XWR68XX_PINR14_PADAN_QSPI_D2);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP12_PADAO, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP12_PADAO, SOC_XWR68XX_PINP12_PADAO_QSPI_D3);

    /**********************************************************************
     * Setup the GPIO:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    config->SensorStatusGPIO = SOC_XWR68XX_GPIO_2;

    /* Initialize the DEMO configuration: */
    config->sysClockFrequency = MSS_SYS_VCLK;
    config->loggingBaudRate = 921600;
    config->commandBaudRate = 115200;

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig(config->SensorStatusGPIO, GPIO_CFG_OUTPUT);
}

/**
 *  @b Description
 *  @n
 *      Calibration save/restore initialization
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibInit(void)
{
    int32_t retVal = 0;
    rlVersion_t verArgs;

    /* Initialize verArgs */
    memset((void *)&verArgs, 0, sizeof(rlVersion_t));

    /* Get the version string: */
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if (retVal < 0)
    {
        CLI_write("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return -1;
    }

    /* Calibration save/restore init */
    gMmwMCB.calibCfg.sizeOfCalibDataStorage = sizeof(MmwDemo_calibData);
    gMmwMCB.calibCfg.calibDataHdr.magic = MMWDEMO_CALIB_STORE_MAGIC;
    memcpy((void *)&gMmwMCB.calibCfg.calibDataHdr.linkVer, (void *)&verArgs.mmWaveLink, sizeof(rlSwVersionParam_t));
    memcpy((void *)&gMmwMCB.calibCfg.calibDataHdr.radarSSVer, (void *)&verArgs.rf, sizeof(rlFwVersionParam_t));

    /* Check if Calibration data is over the Reserved storage */
    if (gMmwMCB.calibCfg.sizeOfCalibDataStorage <= MMWDEMO_CALIB_FLASH_SIZE)
    {
        gMmwMCB.calibCfg.calibDataHdr.hdrLen = sizeof(MmwDemo_calibDataHeader);
        gMmwMCB.calibCfg.calibDataHdr.dataLen = sizeof(MmwDemo_calibData) - sizeof(MmwDemo_calibDataHeader);

        /* Resets calibration data */
        memset((void *)&gCalibDataStorage, 0, sizeof(MmwDemo_calibData));

        retVal = mmwDemo_flashInit();
    }
    else
    {
        CLI_write("Error: Calibration data size is bigger than reserved size\n");
        retVal = -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibDataHdr     	Pointer to Calibration data header
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData)
{
    uint32_t flashOffset;
    int32_t retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMCB.calibCfg.flashOffset;

    /* Copy header  */
    memcpy((void *)&(ptrCalibrationData->calibDataHdr), ptrCalibDataHdr, sizeof(MmwDemo_calibDataHeader));

    /* Flash calibration data */
    retVal = mmwDemo_flashWrite(flashOffset, (uint32_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if (retVal < 0)
    {
        /* Flash Header failed */
        CLI_write("Error: MmwDemo failed flashing calibration data with error[%d].\n", retVal);
    }

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end through MMWave_open()
 *
 *  @param[in]  ptrCalibData     	Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *ptrCalibData)
{
    MmwDemo_calibDataHeader *pDataHdr;
    int32_t retVal = 0;
    uint32_t flashOffset;

    pDataHdr = &(ptrCalibData->calibDataHdr);

    /* Calculate the read size in bytes */
    flashOffset = gMmwMCB.calibCfg.flashOffset;

    /* Read calibration data header */
    if (mmwDemo_flashRead(flashOffset, (uint32_t *)pDataHdr, sizeof(MmwDemo_calibData)) < 0)
    {
        /* Error: only one can be enable at at time */
        CLI_write("Error: MmwDemo failed when reading calibration data from flash.\n");
        return -1;
    }

    /* Validate data header */
    if ((pDataHdr->magic != MMWDEMO_CALIB_STORE_MAGIC) ||
        (pDataHdr->hdrLen != gMmwMCB.calibCfg.calibDataHdr.hdrLen) ||
        (pDataHdr->dataLen != gMmwMCB.calibCfg.calibDataHdr.dataLen))
    {
        /* Header validation failed */
        CLI_write("Error: MmwDemo calibration data header validation failed.\n");
        retVal = -1;
    }
    /* Matching mmwLink version:
         In this demo, we would like to save/restore with the matching mmwLink and RF FW version.
         However, this logic can be changed to use data saved from previous mmwLink and FW releases,
         as long as the data format of the calibration data matches.
     */
    else if (memcmp((void *)&pDataHdr->linkVer, (void *)&gMmwMCB.calibCfg.calibDataHdr.linkVer, sizeof(rlSwVersionParam_t)) != 0)
    {
        CLI_write("Error: MmwDemo failed mmwLink version validation when restoring calibration data.\n");
        retVal = -1;
    }
    else if (memcmp((void *)&pDataHdr->radarSSVer, (void *)&gMmwMCB.calibCfg.calibDataHdr.radarSSVer, sizeof(rlFwVersionParam_t)) != 0)
    {
        CLI_write("Error: MmwDemo failed RF FW version validation when restoring calibration data.\n");
        retVal = -1;
    }

    return (retVal);
}

static void MCANAppErrStatusCallback(CANFD_Handle handle, CANFD_Reason reason,
                                     CANFD_ErrStatusResp *errStatusResp)
{
    /*Record the error count */
    gErrStatusInt++;
    return;
}

static uint32_t get_uptime_sec(void)
{
    uint32_t seconds = ((Clock_getTicks() * Clock_tickPeriod) / 1000000U);
    return seconds;
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
            retVal = canardHandleRxFrame(&canard, &rx_frame, (Clock_getTicks() * Clock_tickPeriod));
            if (retVal < 0)
            {
                CLI_write("Error: canardHandleRxFrame %d\n", retVal);
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
    SOC_setPeripheralClock(gMmwMCB.socHandle, SOC_MODULE_MCAN, SOC_CLKSOURCE_VCLK, 4U, &errCode);

    /* Initialize peripheral memory */
    SOC_initPeripheralRam(gMmwMCB.socHandle, SOC_MODULE_MCAN, &errCode);
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
void MmwDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    MMWave_InitCfg initCfg;
    UART_Params uartParams;
    Task_Params taskParams;
    Semaphore_Params semParams;
    DPM_InitCfg dpmInitCfg;
    DPC_ObjectDetection_InitParams objDetInitParams;
    uint32_t edmaCCIdx;
    int32_t i;
    // int32_t retVal;

    /* Debug Message: */
    CLI_write("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize CAN */
    Can_Initialize();

    /* Initialize the UART */
    UART_init();

    /* Initialize the GPIO */
    GPIO_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Platform specific configuration */
    MmwDemo_platformInit(&gMmwMCB.cfg.platformCfg);

    /* Initialize the Data Path: */
    MmwDemo_dataPathInit(&gMmwMCB.dataPathObj);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate = gMmwMCB.cfg.platformCfg.commandBaudRate;
    uartParams.isPinMuxDone = 1;

    /* Open the UART Instance */
    gMmwMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMCB.commandUartHandle == NULL)
    {
        // CLI_write("Error: Unable to open the Command UART Instance\n");
        MmwDemo_debugAssert(0);
        return;
    }
    CLI_write("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.commandUartHandle);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate = gMmwMCB.cfg.platformCfg.loggingBaudRate;
    uartParams.isPinMuxDone = 1U;

    /* Open the Logging UART Instance: */
    gMmwMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMCB.loggingUartHandle == NULL)
    {
        // CLI_write("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert(0);
        return;
    }
    CLI_write("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.loggingUartHandle);

    /* Create a binary semaphores which are used to signal DPM_start/DPM_stop are done.
     * The signaling (Semaphore_post) will be done from DPM registered report function
     * (which will execute in the DPM execute task context). */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMCB.DPMstartSemHandle = Semaphore_create(0, &semParams, NULL);
    gMmwMCB.DPMstopSemHandle = Semaphore_create(0, &semParams, NULL);
    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain = MMWave_Domain_MSS;
    initCfg.socHandle = gMmwMCB.socHandle;
    initCfg.eventFxn = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel = CRC_Channel_CH1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode = MMWave_ExecutionMode_ISOLATION;

    /* Initialize and setup the mmWave Control module */
    gMmwMCB.ctrlHandle = MMWave_init(&initCfg, &errCode);
    if (gMmwMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        CLI_write("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    CLI_write("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequisite and always needs to be invoked. */
    if (MMWave_sync(gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        CLI_write("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    CLI_write("Debug: mmWave Control Synchronization was successful\n");

    MmwDemo_dataPathOpen(&gMmwMCB.dataPathObj);

    /* Initialize LVDS streaming components */
    if ((errCode = MmwDemo_LVDSStreamInit()) < 0)
    {
        CLI_write("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n", errCode);
        return;
    }

    /* initialize cq configs to invalid profile index to be able to detect
     * unconfigured state of these when monitors for them are enabled.
     */
    for (i = 0; i < RL_MAX_PROFILES_CNT; i++)
    {
        gMmwMCB.cqSatMonCfg[i].profileIndx = (RL_MAX_PROFILES_CNT + 1);
        gMmwMCB.cqSigImgMonCfg[i].profileIndx = (RL_MAX_PROFILES_CNT + 1);
    }

    /* Configure banchmark counter */
    Cycleprofiler_init();

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3 * 1024;
    gMmwMCB.taskHandles.mmwaveCtrl = Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetection_InitParams));

    // /* Note this must be after MmwDemo_dataPathOpen() above which opens the hwa
    //  * and edma drivers */
    objDetInitParams.hwaHandle = gMmwMCB.dataPathObj.hwaHandle;
    for (edmaCCIdx = 0; edmaCCIdx < EDMA_NUM_CC; edmaCCIdx++)
    {
        objDetInitParams.edmaHandle[edmaCCIdx] = gMmwMCB.dataPathObj.edmaHandle[edmaCCIdx];
    }

    // /* Memory related config */
    objDetInitParams.L3ramCfg.addr = (void *)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreLocalRamCfg.addr = &gDPC_ObjDetTCM[0];
    objDetInitParams.CoreLocalRamCfg.size = sizeof(gDPC_ObjDetTCM);

    // // /* Call-back config */
    objDetInitParams.processCallBackCfg.processFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn;
    objDetInitParams.processCallBackCfg.processInterFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn;

    memset((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    // /* Setup the configuration: */
    dpmInitCfg.socHandle = gMmwMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg = &gDPC_ObjectDetectionCfg;
    dpmInitCfg.instanceId = 0xFEEDFEED;
    dpmInitCfg.domain = DPM_Domain_LOCALIZED;
    dpmInitCfg.reportFxn = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.arg = &objDetInitParams;
    dpmInitCfg.argSize = sizeof(DPC_ObjectDetection_InitParams);

    // /* Initialize the DPM Module: */
    gMmwMCB.dataPathObj.objDetDpmHandle = DPM_init(&dpmInitCfg, &errCode);
    if (gMmwMCB.dataPathObj.objDetDpmHandle == NULL)
    {
        CLI_write("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 4 * 1024;
    gMmwMCB.taskHandles.objDetDpmTask = Task_create(MmwDemo_DPC_ObjectDetection_dpmTask, &taskParams, NULL);

    // /* Calibration save/restore initialization */
    if (MmwDemo_calibInit() < 0)
    {
        CLI_write("Error: Calibration data initialization failed \n");
        MmwDemo_debugAssert(0);
    }
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
    taskParams.stackSize = 4 * 1024;
    Task_Handle dronecanTask = Task_create(uavcan_task, &taskParams, NULL);
    Task_Params_init(&taskParams);
    taskParams.priority = UAVCAN_RX_EVENT_TASK_PRIORITY;
    taskParams.stackSize = 2 * 1024;
    Task_Handle uavcanRxTask = Task_create(uavcan_rx_event_task, &taskParams, NULL);
    if (uavcanRxTask == NULL)
    {
        CLI_write("Error: UAVCAN RX event proc task create failed\n");
    }
    /*****************************************************************************
     * Initialize the CLI Module:
     *      User can choose to create their own task here with the same priority
     *      instead that does hard coded config instead of interactive CLI
     *****************************************************************************/
    MmwDemo_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
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
    Task_Params taskParams;
    int32_t errCode;
    SOC_Handle socHandle;
    SOC_Cfg socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;
    socCfg.dssCfg = SOC_DSSCfg_HALT;
    socCfg.mpuCfg = SOC_MPUCfg_CONFIG;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init(&socCfg, &errCode);
    if (socHandle == NULL)
    {
        // System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return -1;
    }

    /* Wait for BSS powerup */
    if (SOC_waitBSSPowerUp(socHandle, &errCode) < 0)
    {
        /* Debug Message: */
        CLI_write("Debug: SOC_waitBSSPowerUp failed with Error [%d]\n", errCode);
        return 0;
    }

    /* Check if the SOC is a secure device */
    if (SOC_isSecureDevice(socHandle, &errCode))
    {
        /* Disable firewall for JTAG and LOGGER (UART) which is needed by all unit tests */
        SOC_controlSecureFirewall(socHandle,
                                  (uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER),
                                  SOC_SECURE_FIREWALL_DISABLE,
                                  &errCode);
    }

    /* Initialize and populate the demo MCB */
    memset((void *)&gMmwMCB, 0, sizeof(MmwDemo_MCB));

    gMmwMCB.socHandle = socHandle;

    /* Debug Message: */
    CLI_write("**********************************************\n");
    CLI_write("Debug: Launching the MMW HWA Demo\n");
    CLI_write("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    gMmwMCB.taskHandles.initTask = Task_create(MmwDemo_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}