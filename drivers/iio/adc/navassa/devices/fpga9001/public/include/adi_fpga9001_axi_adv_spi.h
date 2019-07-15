/**
* \file
* \brief Functions to configure the FPGA9001 AXI Advanced SPI
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_AXI_ADV_SPI_H_
#define _ADI_FPGA9001_AXI_ADV_SPI_H_

#include "adi_fpga9001_axi_adv_spi_types.h"
#include <stdint.h>
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t adi_fpga9001_AxiAdvSpiCfgSet(adi_fpga9001_Device_t *device, adi_fpga9001_AxiAdvSpiConfig_t *advSpiCfg, uint8_t slaveSelect);
    
int32_t adi_fpga9001_AxiAdvSpiChipSelectAll(adi_fpga9001_Device_t *device, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_AXI_ADV_SPI_H_ */