/**
 * \file
 * \brief Functions for FPGA9001 CMOS Synchronous Serial Interface (SSI) configuration
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_common_types.h"
#include "adrv9001_reg_addr_macros.h"
#include "adi_fpga9001_ssi.h"
#include "adi_fpga9001_hal.h"
#include "fpga9001_ssi.h"
#include "fpga9001_bf_axi_tdd_frame.h"
#include "fpga9001_bf_axi_tdd_enable.h"

int32_t adi_fpga9001_ssi_IDelayTapValue_Set(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                            uint8_t delay)
{
    uint32_t baseAddr = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_IDelayTapValue_SetValidate, device, port, channel, laneSel, delay);

    baseAddr = fpga9001_BfCmosSsiChanAddrGet(device, port, channel);
    switch (laneSel)
    {
    case ADI_FPGA9001_CMOS_LANE_0:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane0BfSet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_1:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane1BfSet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_2:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane2BfSet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_3:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane3BfSet, device, baseAddr, delay);
        break;
    default:
        /* If ADI_VALIDATE_PARAMS is 1, this line should never be executed, therefore untestable */
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            laneSel,
            "Invalid parameter value. laneSel must be ADI_FPGA9001_CMOS_LANE_[0/1/2/3].");
    }

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_IDelayTapValue_Get(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                            uint8_t *delay)
{
    uint32_t baseAddr = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_IDelayTapValue_GetValidate, device, port, channel, laneSel, delay);

    baseAddr = fpga9001_BfCmosSsiChanAddrGet(device, port, channel);
    switch (laneSel)
    {
    case ADI_FPGA9001_CMOS_LANE_0:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane0BfGet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_1:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane1BfGet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_2:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane2BfGet, device, baseAddr, delay);
        break;
    case ADI_FPGA9001_CMOS_LANE_3:
        ADI_EXPECT(fpga9001_CmosSsiIdelayTapValueLane3BfGet, device, baseAddr, delay);
        break;
    default:
        /* If ADI_VALIDATE_PARAMS is 1, this line should never be executed, therefore untestable */
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            laneSel,
            "Invalid parameter value. laneSel must be ADI_FPGA9001_CMOS_LANE_[0/1/2/3].");
    }

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Mode_Set(adi_fpga9001_Device_t *device, 
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  adi_fpga9001_SsiMode_e mode)
{
    uint32_t baseAddr = 0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Mode_SetValidate, device, port, channel, mode);

    baseAddr = fpga9001_BfCmosSsiChanAddrGet(device, port, channel);

    ADI_EXPECT(fpga9001_AxiTddFrameTddframecountersresetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, 0x1);

    if (ADI_RX == port && ADI_CHANNEL_1 == channel)
    {
        ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0, 0x1);
    }
    else if (ADI_RX == port && ADI_CHANNEL_2 == channel)
    {
        ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1, 0x1);
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0, 0x1);
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1, 0x1);
    }
    else
    {
        /* Nothing to do here, validation should ensure this is never reached */
    }    

    ADI_EXPECT(adi_fpga9001_RegisterWrite, device, (baseAddr + 0x04), mode);
    ADI_EXPECT(adi_fpga9001_RegisterWrite, device, (baseAddr + 0x0c), 0x01);

    if (ADI_TX == port) 
    {
        ADI_EXPECT(adi_fpga9001_RegisterWrite, device, (baseAddr + 0x08), 0x01);
    }

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Mode_Get(adi_fpga9001_Device_t *device, 
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  adi_fpga9001_SsiMode_e *mode)
{
    uint32_t baseAddr = 0;
    uint32_t regVal = 0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_ModeGetValidate, device, port, channel, mode);
    
    baseAddr = fpga9001_BfCmosSsiChanAddrGet(device, port, channel);
    static const uint8_t CMOS_SSI_CONFIG_OFFSET = 0x4;
    
    ADI_EXPECT(adi_fpga9001_RegisterRead, device, baseAddr + CMOS_SSI_CONFIG_OFFSET, &regVal);
    
    *mode = (adi_fpga9001_SsiMode_e)(regVal & 0xFF);
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ssi_BytesPerSample_GetValidate(adi_fpga9001_Device_t *device, uint8_t *bytesPerSample)
{
    ADI_NULL_PTR_RETURN(&device->common, bytesPerSample);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_BytesPerSample_Get(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t *bytesPerSample)
{
    adi_fpga9001_SsiMode_e mode = ADI_FPGA9001_CMOS_1LANE_16I16Q;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_BytesPerSample_GetValidate, device, bytesPerSample);
    
    ADI_EXPECT(adi_fpga9001_ssi_Mode_Get, device, port, channel, &mode);
    
    switch (mode)
    {
    case ADI_FPGA9001_CMOS_1LANE_16I16Q:
        *bytesPerSample = 4;
        break;
    case ADI_FPGA9001_CMOS_1LANE_16I:
        *bytesPerSample = 2;
        break;
    case ADI_FPGA9001_CMOS_1LANE_8I:        /* Falls through */
    case ADI_FPGA9001_CMOS_1LANE_2I:
        *bytesPerSample = 1;
        break;
    case ADI_FPGA9001_CMOS_4LANE_16I16Q:    /* Falls through */
    case ADI_FPGA9001_LVDS_2LANE_16I16Q:
        *bytesPerSample = 4;
        break;
    default:
        /* If ADI_VALIDATE_PARAMS is 1, this line should never be executed, therefore untestable */
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, mode, "Invalid mode detected");
    }
    
    ADI_API_RETURN(device);
}
