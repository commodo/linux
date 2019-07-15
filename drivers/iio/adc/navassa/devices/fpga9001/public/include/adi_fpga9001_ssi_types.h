/**
 * \file
 * \brief Types for FPGA9001 CMOS Synchronous Serial Interface (SSI) configuration
 *
 * ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_SSI_TYPES_H_
#define _ADI_FPGA9001_SSI_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief CMOS SSI Lane selections
 */
typedef enum adi_fpga9001_CmosSsiLaneSel
{
    ADI_FPGA9001_CMOS_LANE_0    = 0x1,
    ADI_FPGA9001_CMOS_LANE_1    = 0x2,
    ADI_FPGA9001_CMOS_LANE_2    = 0x4,
    ADI_FPGA9001_CMOS_LANE_3    = 0x8
} adi_fpga9001_CmosSsiLaneSel_e;

/**
 * \brief CMOS modes
 */
typedef enum adi_fpga9001_SsiMode
{
    ADI_FPGA9001_CMOS_1LANE_16I16Q = 0x40,
    ADI_FPGA9001_CMOS_1LANE_16I = 0x44,
    ADI_FPGA9001_CMOS_1LANE_8I = 0x48,
    ADI_FPGA9001_CMOS_1LANE_2I = 0x4c,
    ADI_FPGA9001_CMOS_4LANE_16I16Q = 0x60,
    ADI_FPGA9001_LVDS_2LANE_16I16Q = 0x20
} adi_fpga9001_SsiMode_e;
    
#ifdef __cplusplus
}
#endif

#endif // !_ADI_FPGA9001_SSI_TYPES_H_