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

#ifndef _FPGA9001_SSI_H_
#define _FPGA9001_SSI_H_

#include <stdint.h>
#include "adi_fpga9001_types.h"
#include "adi_fpga9001_ssi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t fpga9001_BfCmosSsiChanAddrGet(adi_fpga9001_Device_t *device,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel);

int32_t fpga9001_CmosSsiIdelayTapValueLane0BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane0BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane1BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane1BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane2BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane2BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane3BfSet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t bfValue);

int32_t fpga9001_CmosSsiIdelayTapValueLane3BfGet(adi_fpga9001_Device_t *device, uint32_t baseAddr, uint8_t *bfValue);

int32_t adi_fpga9001_ssi_IDelayTapValue_SetValidate(adi_fpga9001_Device_t *device,
                                                    adi_common_Port_e port,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                                    uint8_t delay);

int32_t adi_fpga9001_ssi_IDelayTapValue_GetValidate(adi_fpga9001_Device_t *device,
                                                    adi_common_Port_e port,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                                    uint8_t *delay);
    
int32_t adi_fpga9001_ssi_Mode_SetValidate(adi_fpga9001_Device_t *device,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel,
                                          adi_fpga9001_SsiMode_e mode);

int32_t adi_fpga9001_ssi_ModeGetValidate(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_SsiMode_e *mode);

#ifdef __cplusplus
}
#endif

#endif /* _FPGA9001_SSI_H_ */