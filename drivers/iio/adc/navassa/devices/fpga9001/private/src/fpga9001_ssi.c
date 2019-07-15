/**
 * \file
 *
 * \brief Helper functions for FPGA9001 CMOS Synchronous Serial Interface (SSI) configuration
 *
 * These functions call the appropriate CMOS SSI Rx/Tx bitfield set/get functions
 * based on the ssiSel/baseAddr
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include  "fpga9001_ssi.h"
#include "fpga9001_bf_axi_adrv9001_rx.h"
#include "fpga9001_bf_axi_adrv9001_tx.h"

uint32_t fpga9001_BfCmosSsiChanAddrGet(adi_fpga9001_Device_t *device,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel)
{
    if (ADI_RX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_RX_0;
    }
    else if (ADI_RX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_RX_1;
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_TX_0;
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_TX_1;
    }
    else
    {
        return 0;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane0BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay0BfSet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay0BfSet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane0BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay0BfGet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay0BfGet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane1BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay1BfSet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay1BfSet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane1BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay1BfGet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay1BfGet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane2BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay2BfSet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay2BfSet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane2BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay2BfGet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay2BfGet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane3BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay3BfSet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay3BfSet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t fpga9001_CmosSsiIdelayTapValueLane3BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue)
{
    switch (baseAddr)
    {
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_RX_1:
        return fpga9001_AxiAdrv9001RxRxWrdelay3BfGet(device, (fpga9001_BfAxiAdrv9001RxChanAddr_e)baseAddr, bfValue);
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_0:
    case (uint32_t)FPGA9001_BF_AXI_ADRV9001_TX_1:
        return fpga9001_AxiAdrv9001TxTxWrdelay3BfGet(device, (fpga9001_BfAxiAdrv9001TxChanAddr_e)baseAddr, bfValue);
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t adi_fpga9001_ssi_IDelayTapValue_SetValidate(adi_fpga9001_Device_t *device,
                                                    adi_common_Port_e port,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                                    uint8_t delay)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    switch (laneSel)
    {
    case ADI_FPGA9001_CMOS_LANE_0: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_1: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_2: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_3: /* Falls through */
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            laneSel,
            "Invalid parameter value. laneSel must be ADI_FPGA9001_CMOS_LANE_[0/1/2/3].");
    }

    ADI_RANGE_CHECK(device, delay, 0, 31);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_IDelayTapValue_GetValidate(adi_fpga9001_Device_t *device,
                                                    adi_common_Port_e port,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                                    uint8_t *delay)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    switch (laneSel)
    {
    case ADI_FPGA9001_CMOS_LANE_0: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_1: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_2: /* Falls through */
    case ADI_FPGA9001_CMOS_LANE_3: /* Falls through */
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            laneSel,
            "Invalid parameter value. laneSel must be RX0, RX1, TX0, or TX1.");
    }

    ADI_NULL_PTR_RETURN(&device->common, delay);

    ADI_API_RETURN(device);
}
    
int32_t adi_fpga9001_ssi_Mode_SetValidate(adi_fpga9001_Device_t *device,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel,
                                          adi_fpga9001_SsiMode_e mode)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    
    switch (mode)
    {
    case ADI_FPGA9001_CMOS_1LANE_16I16Q:    /* Falls through */
    case ADI_FPGA9001_CMOS_1LANE_16I:       /* Falls through */
    case ADI_FPGA9001_CMOS_1LANE_8I:        /* Falls through */
    case ADI_FPGA9001_CMOS_1LANE_2I:        /* Falls through */
    case ADI_FPGA9001_CMOS_4LANE_16I16Q:    /* Falls through */
    case ADI_FPGA9001_LVDS_2LANE_16I16Q:    /* Falls through */
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         mode,
                         "Invalid parameter value. mode must be a valid adi_fpga9001_SsiMode_e");
    }
        
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_ModeGetValidate(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_SsiMode_e *mode)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
        
    ADI_NULL_PTR_RETURN(&device->common, mode);

    ADI_API_RETURN(device);
}