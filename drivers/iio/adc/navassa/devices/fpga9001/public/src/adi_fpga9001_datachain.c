/**
 * \file: adi_fpga9001_data_chain.c
 *
 * \brief Functions to configure the FPGA9001 Data Chains
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdlib.h>

#include "adi_adrv9001_user.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_datachain.h"
#include "adi_fpga9001_error.h"
#include "adi_fpga9001_hal.h"
#include "adi_fpga9001_ssi.h"
#include "fpga9001_bf_dp_tollgate.h"
#include "fpga9001_bf_dp_rx_dma.h"
#include "fpga9001_bf_dp_tx_dma.h"
#include "fpga9001_bf_dp_capture_control.h"
#include "fpga9001_bf_tdd_dp_ctrl.h"

#include "fpga9001_bf_axi_tdd_enable.h"


/************************** Static helper functions **************************/
/* This function should only be called with a single chain selected */
static fpga9001_BfDpTollgateChanAddr_e fpga9001_TollgateChanAddrGet(adi_fpga9001_Device_t *device,
                                                                    adi_common_Port_e port,
                                                                    adi_common_ChannelNumber_e channel)
{
    if (ADI_RX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_RX_DP_TOLLGATE_00;
    }
    else if (ADI_RX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_RX_DP_TOLLGATE_01;
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_TX_DP_TOLLGATE_00;
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_TX_DP_TOLLGATE_01;
    }
    else
    {
        return 0;
    }
}

static uint32_t fpga9001_DmaChanAddrGet(adi_fpga9001_Device_t *device,
                                        adi_common_Port_e port,
                                        adi_common_ChannelNumber_e channel)
{
    if (ADI_RX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_RX_DP_DMA_00;
    }
    else if (ADI_RX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_RX_DP_DMA_01;
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_TX_DP_DMA_00;
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_TX_DP_DMA_01;
    }
    else
    {
        return 0;
    }
}

int32_t adi_fpga9001_DataChain_Tollgate_Configure(adi_fpga9001_Device_t *device,
                                                  adi_common_Port_e port,
                                                  adi_common_ChannelNumber_e channel,
                                                  adi_fpga9001_TollgateCfg_t *tollGateCfg)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_NULL_PTR_RETURN(&device->common, tollGateCfg);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);

//    ADI_EXPECT(fpga9001_DpTollgateEdgeLevelBfSet, device, instanceAddress, tollGateCfg->tollGateEdgeOrLvl);
//    ADI_EXPECT(fpga9001_DpTollgateHighRisingLowFallingBfSet, device, instanceAddress, tollGateCfg->tollGateHiRiseOrLoFall);
    ADI_EXPECT(fpga9001_DpTollgateTriggerSelectBfSet, device, instanceAddress, tollGateCfg->tollGateTrigSource);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Tollgate_Inspect(adi_fpga9001_Device_t *device,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                adi_fpga9001_TollgateCfg_t *tollGateCfg)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;

#if ADI_FPGA9001_VERBOSE > 0
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_NULL_PTR_RETURN(&device->common, tollGateCfg);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpTollgateEdgeLevelBfGet, device, instanceAddress, &tollGateCfg->tollGateEdgeOrLvl);

    ADI_EXPECT(fpga9001_DpTollgateHighRisingLowFallingBfGet,
               device,
               instanceAddress,
               &tollGateCfg->tollGateHiRiseOrLoFall);

    ADI_EXPECT(fpga9001_DpTollgateTriggerSelectBfGet, device, instanceAddress, &tollGateCfg->tollGateTrigSource);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_TollgateReset_Set(adi_fpga9001_Device_t *device,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint8_t resetVal)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpTollgateResetTollgateBfSet, device, instanceAddress, resetVal);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_TollgateReset_Get(adi_fpga9001_Device_t *device,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint16_t *resetVal)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;
    uint8_t resetBitRead = 0;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_NULL_PTR_RETURN(&device->common, resetVal);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpTollgateResetTollgateBfGet, device, instanceAddress, &resetBitRead);

    *resetVal = (resetBitRead & 1);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_TollgateTrigger_Set(adi_fpga9001_Device_t *device,
                                                   adi_common_Port_e port,
                                                   adi_common_ChannelNumber_e channel,
                                                   uint32_t triggerSelect)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);
    adi_fpga9001_DataChain_TollgateReset_Set(device, port, channel, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_EXPECT(fpga9001_DpTollgateTriggerSelectBfSet, device, instanceAddress, triggerSelect);

    adi_fpga9001_DataChain_TollgateReset_Set(device, port, channel, 0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_TollgateTrigger_Get(adi_fpga9001_Device_t *device,
                                                   adi_common_Port_e port,
                                                   adi_common_ChannelNumber_e channel,
                                                   uint32_t *triggerSelect)
{
    fpga9001_BfDpTollgateChanAddr_e instanceAddress = FPGA9001_BF_RX_DP_TOLLGATE_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, triggerSelect);

    instanceAddress = fpga9001_TollgateChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpTollgateTriggerSelectBfGet, device, instanceAddress, (uint32_t *)triggerSelect);

    ADI_API_RETURN(device);
}

int32_t fpga9001_DpDmaEnableEnhancedModeBfSet(adi_fpga9001_Device_t * device, uint32_t baseAddr, uint8_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaEnableEnhancedModeBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaEnableEnhancedModeBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaEnableSgBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaEnableSgBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaEnableSgBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaLengthBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaLengthBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaLengthBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSgFirstDescriptorBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSgFirstDescriptorBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSqFirstDescriptorBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSgLastDescriptorBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSgLastDescriptorBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSgLastDescriptorBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSimpleStartAddrBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSimpleStartAddrBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSimpleStartAddrBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaValidDataWidthStreamBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaValidDataWidthStreamBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaValidDataWidthStreamBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaResetBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaResetBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaResetBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaRunStopBfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaRunStopBfSet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaRunStopBfSet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t adi_fpga9001_DataChain_Dma_Configure(adi_fpga9001_Device_t *device,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             adi_fpga9001_DmaCfg_t *dmaCfg)
{
    uint32_t instanceAddress = FPGA9001_BF_RX_DP_DMA_00;
    uint32_t MAX_DMA_SIZE = 0;
    uint32_t regVal = 0;
    static const uint8_t CONTROL_REG_OFFSET = 0x8;
    static const uint8_t VALID_DATA_WIDTH_OFFSET = 8;
    static const uint8_t VALID_DATA_WIDTH_MASK = 0xF;
    static const uint8_t ENABLE_ENHANCED_OFFSET = 3;
    static const uint8_t ENABLE_ENHANCED_MASK = 0x1;
    static const uint8_t ENABLE_SG_OFFSET = 2;
    static const uint8_t ENABLE_SG_MASK = 0x1;
    static const uint8_t CONTINUOUS_OFFSET = 1;
    static const uint8_t CONTINUOUS_MASK = 0x1;
    static const uint8_t RUN_STOP_OFFSET = 0;
    static const uint8_t RUN_STOP_MASK = 0x1;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, dmaCfg);

    /* DMA start address and length must be multiples of 64 */
    if (dmaCfg->simpleStartAddr % 64 != 0)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dmaCfg->simpleStartAddr,
                         "Invalid parameter value. dmaCfg->simpleStartAddr must be a multiple of 64");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    if (dmaCfg->length % 64 != 0)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dmaCfg->length,
                         "Invalid parameter value. dmaCfg->length must be a multiple of 64");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Validate dmaCfg->validDataWidthStream */
    switch (dmaCfg->validDataWidthStream)
    {
    case ADI_FPGA9001_STREAM_32_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_64_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_128_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_256_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_512_BITS:   /* Falls through */
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dmaCfg->validDataWidthStream,
                         "Invalid parameter value. dmaCfg->validDataWidthStream must be of adi_fpga9001_ValidDataWidth_e type.");
    }

    /* The max DMA size depends on the data chains selected */
    if (ADI_RX == port)
    {
        MAX_DMA_SIZE = RX_DMA_SIZE;
    }
    else if (ADI_TX == port)
    {
        MAX_DMA_SIZE = TX_DMA_SIZE;
    }
    else
    {
        /* Nothing to do here; validation should ensure this is never reached */
    }

    /* TODO: start address could be changed at runtime by this function, and this length check is not enough to ensure RAM buffers do not overlap */
    if (dmaCfg->length > MAX_DMA_SIZE)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dmaCfg->length,
                         "FPGA DMA capture length is too large for RAM buffer area");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    instanceAddress = fpga9001_DmaChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpDmaSimpleStartAddrBfSet, device, instanceAddress, dmaCfg->simpleStartAddr);

    ADI_EXPECT(fpga9001_DpDmaLengthBfSet, device, instanceAddress, dmaCfg->length);

    /* Write the DMA control register */
    regVal = ((dmaCfg->validDataWidthStream & VALID_DATA_WIDTH_MASK)    << VALID_DATA_WIDTH_OFFSET) |
             ((dmaCfg->enableEnhancedMode   & ENABLE_ENHANCED_MASK)     << ENABLE_ENHANCED_OFFSET)  |
             ((dmaCfg->enableSg             & ENABLE_SG_MASK)           << ENABLE_SG_OFFSET)        |
             ((dmaCfg->continuous           & CONTINUOUS_MASK)          << CONTINUOUS_OFFSET)       |
             ((dmaCfg->runStop              & RUN_STOP_MASK)            << RUN_STOP_OFFSET);

    ADI_EXPECT(adi_fpga9001_RegisterWrite, device, instanceAddress + CONTROL_REG_OFFSET, regVal);

//    ADI_EXPECT(fpga9001_DpDmaSgFirstDescriptorBfSet, device, instanceAddress, dmaCfg->sgFirstDescriptor);

    /* TODO: In TOKELAU, writing to this seems to be clearing the status register. debug later */
//    ADI_EXPECT(fpga9001_DpDmaSgLastDescriptorBfSet, device, instanceAddress, dmaCfg->sgLastDescriptor);

    ADI_API_RETURN(device);
}

int32_t fpga9001_DpDmaActiveBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaActiveBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaActiveBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaCompleteBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaCompleteBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaCompleteBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaEnableEnhancedModeBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaEnableEnhancedModeBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaEnableEnhancedModeBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaEnableSgBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaEnableSgBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaEnableSgBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaHaltCompleteBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaHaltCompleteBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaHaltCompleteBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaLengthBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaLengthBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaLengthBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaOverUnderflowBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaOverflowBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaUnderflowBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaRunStopBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaRunStopBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaRunStopBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaContinuousBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return 0;
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaContinuousBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSgFirstDescriptorBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSgFirstDescriptorBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSqFirstDescriptorBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSgLastDescriptorBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSgLastDescriptorBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSgLastDescriptorBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaSimpleStartAddrBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint32_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaSimpleStartAddrBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaSimpleStartAddrBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_DpDmaValidDataWidthStreamBfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *value)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_RX_DP_DMA_01:
        return fpga9001_DpRxDmaValidDataWidthStreamBfGet(device, (fpga9001_BfDpRxDmaChanAddr_e)baseAddr, value);
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_00:
    case (uint32_t)FPGA9001_BF_TX_DP_DMA_01:
        return fpga9001_DpTxDmaValidDataWidthStreamBfGet(device, (fpga9001_BfDpTxDmaChanAddr_e)baseAddr, value);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t adi_fpga9001_DataChain_Dma_Inspect(adi_fpga9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_fpga9001_DmaCfg_t *dmaCfg)
{
    uint32_t instanceAddress = FPGA9001_BF_RX_DP_DMA_00;
    uint8_t validDataWidthStream = 0;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, dmaCfg);

    instanceAddress = fpga9001_DmaChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpDmaActiveBfGet,                  device, instanceAddress, &dmaCfg->active);
    ADI_EXPECT(fpga9001_DpDmaCompleteBfGet,                device, instanceAddress, &dmaCfg->complete);
    ADI_EXPECT(fpga9001_DpDmaEnableEnhancedModeBfGet,      device, instanceAddress, &dmaCfg->enableEnhancedMode);
    ADI_EXPECT(fpga9001_DpDmaEnableSgBfGet,                device, instanceAddress, &dmaCfg->enableSg);
    ADI_EXPECT(fpga9001_DpDmaHaltCompleteBfGet,            device, instanceAddress, &dmaCfg->haltComplete);
    ADI_EXPECT(fpga9001_DpDmaLengthBfGet,                  device, instanceAddress, &dmaCfg->length);
    ADI_EXPECT(fpga9001_DpDmaOverUnderflowBfGet,           device, instanceAddress, &dmaCfg->overUnderflow);
    ADI_EXPECT(fpga9001_DpDmaRunStopBfGet,                 device, instanceAddress, &dmaCfg->runStop);
    ADI_EXPECT(fpga9001_DpDmaContinuousBfGet,              device, instanceAddress, &dmaCfg->continuous);
    ADI_EXPECT(fpga9001_DpDmaSgFirstDescriptorBfGet,       device, instanceAddress, &dmaCfg->sgFirstDescriptor);
    ADI_EXPECT(fpga9001_DpDmaSgLastDescriptorBfGet,        device, instanceAddress, &dmaCfg->sgLastDescriptor);
    ADI_EXPECT(fpga9001_DpDmaSimpleStartAddrBfGet,         device, instanceAddress, &dmaCfg->simpleStartAddr);
    ADI_EXPECT(fpga9001_DpDmaValidDataWidthStreamBfGet,    device, instanceAddress, &validDataWidthStream);
    dmaCfg->validDataWidthStream = (adi_fpga9001_ValidDataWidth_e)validDataWidthStream;

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_DmaReset_Set(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t reset)
{
    uint32_t instanceAddress = FPGA9001_BF_RX_DP_DMA_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_DmaChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpDmaResetBfSet, device, instanceAddress, reset);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_DmaRunStop_Set(adi_fpga9001_Device_t *device,
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel,
                                              uint8_t runStop)
{
    uint32_t instanceAddress = FPGA9001_BF_RX_DP_DMA_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    instanceAddress = fpga9001_DmaChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpDmaRunStopBfSet, device, instanceAddress, runStop);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_DmaLength_Set(adi_fpga9001_Device_t *device,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             uint32_t numBytes)
{
    uint32_t instanceAddress = FPGA9001_BF_RX_DP_DMA_00;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    if (numBytes > RX_DMA_SIZE)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         numBytes,
                         "FPGA Rx DMA capture length is too large for RAM buffer area");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    instanceAddress = fpga9001_DmaChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_DpDmaLengthBfSet, device, instanceAddress, numBytes);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Configure(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_TollgateCfg_t *tollgateCfg,
                                         adi_fpga9001_DmaCfg_t *dmaCfg)
{
    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    /* Capture control datapath active */
    if (ADI_RX == port)
    {
        fpga9001_DpCaptureControlDatapathActiveBfSet(device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 1);
    }
    if (ADI_TX == port)
    {
        fpga9001_DpCaptureControlDatapathActiveBfSet(device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 1);
    }

    /* Set up tollgate */
    ADI_EXPECT(adi_fpga9001_DataChain_Tollgate_Configure, device, port, channel, tollgateCfg);

    /* Configure DMAs */
    ADI_EXPECT(adi_fpga9001_DataChain_Dma_Configure, device, port, channel, dmaCfg);

    /* Reset capture control */
    if (ADI_RX == port)
    {
        ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 1);
    }
    if (ADI_TX == port)
    {
        ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 1);
    }

    /* Reset tollgate */
    ADI_EXPECT(adi_fpga9001_DataChain_TollgateReset_Set, device, port, channel, 1);

    ///* Reset DMAs */
    ADI_EXPECT(adi_fpga9001_DataChain_DmaReset_Set, device, port, channel, 1);

    /* Un-reset capture control */
    if (ADI_RX == port)
    {
        ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 0);
    }
    if (ADI_TX == port)
    {
        ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 0);
    }

    /* Un-reset tollgate */
    ADI_EXPECT(adi_fpga9001_DataChain_TollgateReset_Set, device, port, channel, 0);

    /* Un-reset DMAs */
    ADI_EXPECT(adi_fpga9001_DataChain_DmaReset_Set, device, port, channel, 0);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Reset(adi_fpga9001_Device_t *device)
{
    uint8_t active = 0;
    uint8_t haltComplete = 0;
    uint8_t i = 0;
    static const uint8_t DATA_CHAIN_RESET_MAX_COUNT = 5;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    /* reset  */
    ADI_EXPECT(fpga9001_DpCaptureControlDatapathActiveBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 0);
    ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 1);

    ADI_EXPECT(fpga9001_DpCaptureControlDatapathActiveBfSet, device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 0);
    ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 1);

    /* Clear Rx DMA 0 */
    ADI_EXPECT(fpga9001_DpDmaActiveBfGet, device, FPGA9001_BF_RX_DP_DMA_00, &active);

    adi_fpga9001_DataChain_DmaRunStop_Set(device, ADI_RX, ADI_CHANNEL_1, 0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    if (active == 1)
    {
        haltComplete = 0;
        for (i = 0; i < DATA_CHAIN_RESET_MAX_COUNT; i++)
        {
            ADI_EXPECT(fpga9001_DpDmaHaltCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_00, &haltComplete);
            if (haltComplete != 0)
            {
                break;
            }
        }

        if (i == DATA_CHAIN_RESET_MAX_COUNT)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "RX0 DMA Halt failed.");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    adi_fpga9001_DataChain_DmaReset_Set(device, ADI_RX, ADI_CHANNEL_1, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    /* End Rx DMA 0*/

    /* Clear Rx DMA 1 */
    active = 0;
    haltComplete = 0;
    ADI_EXPECT(fpga9001_DpDmaActiveBfGet, device, FPGA9001_BF_RX_DP_DMA_01, &active);

    adi_fpga9001_DataChain_DmaRunStop_Set(device, ADI_RX, ADI_CHANNEL_2, 0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    if (active == 1)
    {
        for (i = 0; i < DATA_CHAIN_RESET_MAX_COUNT; i++)
        {
            ADI_EXPECT(fpga9001_DpDmaHaltCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_01, &haltComplete);
            if (haltComplete != 0)
            {
                break;
            }
        }

        if (i == DATA_CHAIN_RESET_MAX_COUNT)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "RX1 DMA Halt failed.");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    adi_fpga9001_DataChain_DmaReset_Set(device, ADI_RX, ADI_CHANNEL_2, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    /* End Rx DMA 1*/

    /* Clear Tx DMA 0 */
    haltComplete = 0;
    ADI_EXPECT(fpga9001_DpDmaActiveBfGet, device, FPGA9001_BF_TX_DP_DMA_00, &active);

    adi_fpga9001_DataChain_DmaRunStop_Set(device, ADI_TX, ADI_CHANNEL_1, 0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    if (active == 1)
    {
        haltComplete = 0;
        for (i = 0; i < DATA_CHAIN_RESET_MAX_COUNT; i++)
        {
            ADI_EXPECT(fpga9001_DpDmaHaltCompleteBfGet, device, FPGA9001_BF_TX_DP_DMA_00, &haltComplete);
            if (haltComplete != 0)
            {
                break;
            }
        }

        if (i == DATA_CHAIN_RESET_MAX_COUNT)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "TX0 DMA Halt failed.");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    adi_fpga9001_DataChain_DmaReset_Set(device, ADI_TX, ADI_CHANNEL_1, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    /* End Tx DMA 0*/

    /* Clear DMA 1 */
    haltComplete = 0;
    active = 0;
    ADI_EXPECT(fpga9001_DpDmaActiveBfGet, device, FPGA9001_BF_TX_DP_DMA_01, &active);

    adi_fpga9001_DataChain_DmaRunStop_Set(device, ADI_TX, ADI_CHANNEL_2, 0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    if (active == 1)
    {
        for (i = 0; i < DATA_CHAIN_RESET_MAX_COUNT; i++)
        {
            ADI_EXPECT(fpga9001_DpDmaHaltCompleteBfGet, device, FPGA9001_BF_TX_DP_DMA_01, &haltComplete);
            if (haltComplete != 0)
            {
                break;
            }
        }

        if (i == DATA_CHAIN_RESET_MAX_COUNT)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "TX1 DMA Halt failed.");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    adi_fpga9001_DataChain_DmaReset_Set(device, ADI_TX, ADI_CHANNEL_2, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    /* End Tx DMA 1*/

    adi_fpga9001_DataChain_TollgateReset_Set(device, ADI_RX, ADI_CHANNEL_1, 1);
    adi_fpga9001_DataChain_TollgateReset_Set(device, ADI_RX, ADI_CHANNEL_2, 1);
    adi_fpga9001_DataChain_TollgateReset_Set(device, ADI_TX, ADI_CHANNEL_1, 1);
    adi_fpga9001_DataChain_TollgateReset_Set(device, ADI_TX, ADI_CHANNEL_2, 1);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_TxData_Stop(adi_fpga9001_Device_t *device)
{
#if ADI_FPGA9001_VERBOSE > 0
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif
    /* Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_EXPECT(fpga9001_DpTxDmaRunStopBfSet, device, FPGA9001_BF_TX_DP_DMA_00, 0);
    ADI_EXPECT(fpga9001_DpTxDmaRunStopBfSet, device, FPGA9001_BF_TX_DP_DMA_01, 0);

    ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_TX_DP_CAPTURE_CONTROL, 1);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_RxCapture_Wait(adi_fpga9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              uint32_t timeout_ms)
{
    uint8_t bfVal = 0;
    uint32_t i = 0;

    /* TODO: Refactor Range checks */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    for (i = 0; i < timeout_ms; i++)
    {
        if (ADI_CHANNEL_1 == channel)
        {
            ADI_EXPECT(fpga9001_DpDmaCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_00, &bfVal);
            if (bfVal == 1)
            {
                return ADI_COMMON_ACT_NO_ACTION;
            }
        }
        if (ADI_CHANNEL_2 == channel)
        {
            ADI_EXPECT(fpga9001_DpDmaCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_01, &bfVal);
            if (bfVal == 1)
            {
                return ADI_COMMON_ACT_NO_ACTION;
            }
        }

        adi_common_hal_Wait_us(&device->common, 1000);
    }

    ADI_ERROR_REPORT(&device->common,
                     ADI_COMMON_ERRSRC_API,
                     ADI_FPGA9001_ERR_RX_DATA_MOVER_WAIT_TIMEOUT,
                     ADI_COMMON_ACT_ERR_RESET_FEATURE,
                     NULL,
                     "RxCaptureWait timeout");

    ADI_API_RETURN(device);
}

static int32_t RamAddrOffsetGet(adi_fpga9001_Device_t *device,
                                adi_common_Port_e port,
                                adi_common_ChannelNumber_e channel,
                                uint32_t *ramAddress)
{
    if (ADI_RX == port && ADI_CHANNEL_1 == channel)
    {
        *ramAddress = ADI_FPGA9001_RX1_ADDR_OFFSET;
    }
    else if (ADI_RX == port && ADI_CHANNEL_2 == channel)
    {
        *ramAddress = ADI_FPGA9001_RX2_ADDR_OFFSET;
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        *ramAddress = ADI_FPGA9001_TX1_ADDR_OFFSET;
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        *ramAddress = ADI_FPGA9001_TX2_ADDR_OFFSET;
    }
    else
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
    
    return ADI_COMMON_ACT_NO_ACTION;
}

int32_t adi_fpga9001_DataChain_PerformRx(adi_fpga9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         uint32_t timeout_ms,
                                         uint32_t numBytes,
                                         adi_fpga9001_TollgateTriggerSources_e trigger)
{
    uint32_t ramAddress = 0;

    adi_fpga9001_TollgateCfg_t tollgateCfg = {
        .tollGateTrigSource = trigger
    };

    adi_fpga9001_DmaCfg_t dmaCfg = {
        .enableEnhancedMode = 0,
        .length = numBytes,
        .validDataWidthStream = ADI_FPGA9001_STREAM_32_BITS,
        .runStop = 1
    };

    ADI_API_ENTRY_EXPECT(device);

    ADI_EXPECT(RamAddrOffsetGet, device, ADI_RX, channel, &ramAddress);
    dmaCfg.simpleStartAddr = ADI_FPGA9001_RAM_START_ADDR + ramAddress;

    ADI_EXPECT(adi_fpga9001_DataChain_Configure, device, ADI_RX, channel, &tollgateCfg, &dmaCfg);
    ADI_EXPECT(adi_fpga9001_DataChain_RxCapture_Wait, device, channel, timeout_ms);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_PerformTx(adi_fpga9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_TollgateTriggerSources_e trigger)
{
    uint32_t ramAddress = 0;
    adi_fpga9001_TollgateCfg_t tollgateCfg = {
        .tollGateTrigSource = trigger
    };

    adi_fpga9001_DmaCfg_t dmaCfg = { 0 };

    ADI_API_ENTRY_EXPECT(device);

    ADI_EXPECT(RamAddrOffsetGet, device, ADI_TX, channel, &ramAddress);
    ADI_EXPECT(adi_fpga9001_DataChain_Dma_Inspect, device, ADI_TX, channel, &dmaCfg);
    dmaCfg.continuous = 1;
    dmaCfg.enableEnhancedMode = 0;
    dmaCfg.validDataWidthStream = ADI_FPGA9001_STREAM_32_BITS;
    dmaCfg.runStop = 1;
    dmaCfg.simpleStartAddr = ADI_FPGA9001_RAM_START_ADDR + ramAddress;
    ADI_EXPECT(adi_fpga9001_DataChain_Configure, device, ADI_TX, channel, &tollgateCfg, &dmaCfg);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_DataChain_Data_Get_16I16QValidate(adi_fpga9001_Device_t *device,
                                                              adi_common_Port_e port,
                                                              adi_common_ChannelNumber_e channel,
                                                              uint16_t *iData,
                                                              uint16_t *qData,
                                                              uint32_t length)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, iData);
    ADI_NULL_PTR_RETURN(&device->common, qData);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Get_16I16Q(adi_fpga9001_Device_t *device,
                                               adi_common_Port_e port,
                                               adi_common_ChannelNumber_e channel,
                                               uint16_t *iData,
                                               uint16_t *qData,
                                               uint32_t length)
{
    uint32_t i = 0;
    uint32_t *rdBuff = NULL;
    uint32_t ramAddress = 0;
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16I16QValidate, device, port, channel, iData, qData, length);

    rdBuff = calloc(length, sizeof(uint32_t));
    if (rdBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    recoveryAction = RamAddrOffsetGet(device, port, channel, &ramAddress);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    recoveryAction = adi_fpga9001_RamRead(device, ramAddress, rdBuff, length);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length; i++)
    {
        iData[i] = (uint16_t)((rdBuff[i] & 0xFFFF0000) >> 16);
        qData[i] = (uint16_t)(rdBuff[i]  & 0x0000FFFF);
    }

    free(rdBuff);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_DataChain_Data_Get_16IQInterleavedValidate(adi_fpga9001_Device_t *device,
                                                                       adi_common_Port_e port,
                                                                       adi_common_ChannelNumber_e channel,
                                                                       uint16_t *iqData,
                                                                       uint32_t length)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, iqData);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Get_16IQInterleaved(adi_fpga9001_Device_t *device,
                                                        adi_common_Port_e port,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint16_t *iqData,
                                                        uint32_t length)
{
    uint32_t i = 0;
    uint32_t *rdBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16IQInterleavedValidate, device, port, channel, iqData, length);

    rdBuff = calloc(length / 2, sizeof(uint32_t));
    if (rdBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    if (adi_fpga9001_RamRead(device, ramAddress, rdBuff, length / 2) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 2; i++)
    {
        iqData[2 * i]     = (uint16_t)((rdBuff[i] & 0xFFFF0000) >> 16);
        iqData[2 * i + 1] = (uint16_t)(rdBuff[i]  & 0x0000FFFF);
    }

    free(rdBuff);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_DataChain_Data_Get_16IValidate(adi_fpga9001_Device_t *device,
                                                           adi_common_Port_e port,
                                                           adi_common_ChannelNumber_e channel,
                                                           uint16_t *iData,
                                                           uint32_t length)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, iData);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Get_16I(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint16_t *iData,
                                            uint32_t length)
{
    uint32_t i = 0;
    uint32_t *rdBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16IValidate, device, port, channel, iData, length);

    rdBuff = calloc(length / 2, sizeof(uint32_t));
    if (rdBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }
    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    if (adi_fpga9001_RamRead(device, ramAddress, rdBuff, length / 2) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 2; i++)
    {
        iData[2 * i]     = (uint16_t)(rdBuff[i]  & 0x0000FFFF);
        iData[2 * i + 1] = (uint16_t)((rdBuff[i] & 0xFFFF0000) >> 16);
    }

    free(rdBuff);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_DataGet_SValidate(adi_fpga9001_Device_t *device,
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel,
                                              uint8_t *symbols,
                                              uint32_t length)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, symbols);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Get_8S(adi_fpga9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t *symbols,
                                           uint32_t length)
{
    uint32_t i = 0;
    uint32_t *rdBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataGet_SValidate, device, port, channel, symbols, length);

    rdBuff = calloc(length / 4, sizeof(uint32_t));
    if (rdBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    if (adi_fpga9001_RamRead(device, ramAddress, rdBuff, length / 4) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 4; i++)
    {
        symbols[4 * i]     = (uint8_t)(rdBuff[i]  & 0x000000FF);
        symbols[4 * i + 1] = (uint8_t)((rdBuff[i] & 0x0000FF00) >> 8);
        symbols[4 * i + 2] = (uint8_t)((rdBuff[i] & 0x00FF0000) >> 16);
        symbols[4 * i + 3] = (uint8_t)((rdBuff[i] & 0xFF000000) >> 24);
    }

    free(rdBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Get_2S(adi_fpga9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t *symbols,
                                           uint32_t length)
{
    uint32_t i = 0;
    uint32_t *rdBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataGet_SValidate, device, port, channel, symbols, length);

    rdBuff = calloc(length / 4, sizeof(uint32_t));
    if (rdBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    if (adi_fpga9001_RamRead(device, ramAddress, rdBuff, length / 4) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(rdBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 4; i++)
    {
        symbols[4 * i]     = (uint8_t)(rdBuff[i]  & 0x00000003);
        symbols[4 * i + 1] = (uint8_t)((rdBuff[i] & 0x00000300) >> 8);
        symbols[4 * i + 2] = (uint8_t)((rdBuff[i] & 0x00030000) >> 16);
        symbols[4 * i + 3] = (uint8_t)((rdBuff[i] & 0x03000000) >> 24);
    }

    free(rdBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Set_16I16Q(adi_fpga9001_Device_t *device,
                                               adi_common_Port_e port,
                                               adi_common_ChannelNumber_e channel,
                                               uint16_t *iData,
                                               uint16_t *qData,
                                               uint32_t length)
{
    uint32_t i = 0;
    uint32_t *wrBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16I16QValidate, device, port, channel, iData, qData, length);

    ADI_EXPECT(adi_fpga9001_DataChain_DmaLength_Set, device, port, channel, length * 4);

    wrBuff = calloc(length, sizeof(uint32_t));
    if (wrBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(wrBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length; i++)
    {
        wrBuff[i] = (uint32_t)(iData[i] << 16) | (uint32_t)qData[i];
    }

    adi_fpga9001_RamWrite(device, ramAddress, wrBuff, length);

    free(wrBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Set_16IQInterleaved(adi_fpga9001_Device_t *device,
                                                        adi_common_Port_e port,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint16_t *iqData,
                                                        uint32_t length)
{
    uint32_t i = 0;
    uint32_t *wrBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16IQInterleavedValidate, device, port, channel, iqData, length);

    ADI_EXPECT(adi_fpga9001_DataChain_DmaLength_Set, device, port, channel, length * 2);

    wrBuff = calloc(length / 2, sizeof(uint32_t));
    if (wrBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(wrBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 2; i++)
    {
        wrBuff[i] = (uint32_t)(iqData[2 * i] << 16) | (uint32_t)iqData[2 * i + 1];
        iqData[2 * i]     = (uint16_t)((wrBuff[i] & 0xFFFF0000) >> 16);
        iqData[2 * i + 1] = (uint16_t)(wrBuff[i]  & 0x0000FFFF);
    }

    adi_fpga9001_RamWrite(device, ramAddress, wrBuff, length / 2);

    free(wrBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Set_16I(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint16_t *iData,
                                            uint32_t length)
{
    uint32_t i = 0;
    uint32_t *wrBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataChain_Data_Get_16IValidate, device, port, channel, iData, length);

    ADI_EXPECT(adi_fpga9001_DataChain_DmaLength_Set, device, port, channel, length * 2);

    wrBuff = calloc(length / 2, sizeof(uint32_t));
    if (wrBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(wrBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 2; i++)
    {
        wrBuff[i] = (uint32_t)(iData[2 * i + 1] << 16) | (uint32_t)iData[2 * i];
    }

    adi_fpga9001_RamWrite(device, ramAddress, wrBuff, length / 2);

    free(wrBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Set_8S(adi_fpga9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t *symbols,
                                           uint32_t length)
{
    uint32_t i = 0;
    uint32_t *wrBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataGet_SValidate, device, port, channel, symbols, length);

    ADI_EXPECT(adi_fpga9001_DataChain_DmaLength_Set, device, port, channel, length);

    wrBuff = calloc(length / 4, sizeof(uint32_t));
    if (wrBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(wrBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 4; i++)
    {
        wrBuff[i] = (uint32_t)(symbols[4 * i + 3] << 24) |
                    (uint32_t)(symbols[4 * i + 2] << 16) |
                    (uint32_t)(symbols[4 * i + 1] << 8)  |
                    (uint32_t)symbols[4 * i];
    }

    adi_fpga9001_RamWrite(device, ramAddress, wrBuff, length / 4);

    free(wrBuff);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_DataChain_Data_Set_2S(adi_fpga9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t *symbols,
                                           uint32_t length)
{
    uint32_t i = 0;
    uint32_t *wrBuff = NULL;
    uint32_t ramAddress = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_DataGet_SValidate, device, port, channel, symbols, length);

    ADI_EXPECT(adi_fpga9001_DataChain_DmaLength_Set, device, port, channel, length);

    wrBuff = calloc(length / 4, sizeof(uint32_t));
    if (wrBuff == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_MEM_ALLOC_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         NULL,
                         "Unable to allocate memory");
        ADI_API_RETURN(device);
    }

    if (RamAddrOffsetGet(device, port, channel, &ramAddress) != ADI_COMMON_ACT_NO_ACTION)
    {
        free(wrBuff);
        ADI_API_RETURN(device);
    }

    for (i = 0; i < length / 4; i++)
    {
        wrBuff[i] = (uint32_t)((symbols[4 * i + 3] & 0x03) << 24) |
                    (uint32_t)((symbols[4 * i + 2] & 0x03) << 16) |
                    (uint32_t)((symbols[4 * i + 1] & 0x03) << 8) |
                    (uint32_t)(symbols[4 * i] & 0x03);
    }

    adi_fpga9001_RamWrite(device, ramAddress, wrBuff, length / 4);

    free(wrBuff);

    ADI_API_RETURN(device);
}
