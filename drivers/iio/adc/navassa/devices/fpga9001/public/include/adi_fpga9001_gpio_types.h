/**
* \file
* \brief Contains FPGA9001 GPIO enum and struct definitions
*
* ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */


#ifndef _ADI_FPGA9001_GPIO_TYPES_H_
#define _ADI_FPGA9001_GPIO_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADI_FPGA9001_NUM_GPIO_PINS 12

/**
 * \brief Possible modes of operation for FPGA9001 GPIO
 */
typedef enum adi_fpga9001_GpioModes
{
    ADI_FPGA9001_GPIO_MODE_NORMAL = 0,
    ADI_FPGA9001_GPIO_MODE_JTAG,
} adi_fpga9001_GpioModes_e;

/**
 * \brief Possible directions for FPGA9001 GPIO
 */
typedef enum adi_fpga9001_GpioPinDir
{
    ADI_FPGA9001_GPIO_PIN_OUTPUT = 1,
    ADI_FPGA9001_GPIO_PIN_INPUT = 0
} adi_fpga9001_GpioPinDir_e;

/**
 * \brief FPGA9001 GPIO pin selections
 */
typedef enum adi_fpga9001_GpioPin
{
    ADI_FPGA9001_GPIO_00  = 0x001,
    ADI_FPGA9001_GPIO_01  = 0x002,
    ADI_FPGA9001_GPIO_02  = 0x004,
    ADI_FPGA9001_GPIO_03  = 0x008,
    ADI_FPGA9001_GPIO_04  = 0x010,
    ADI_FPGA9001_GPIO_05  = 0x020,
    ADI_FPGA9001_GPIO_06  = 0x040,
    ADI_FPGA9001_GPIO_07  = 0x080,
    ADI_FPGA9001_GPIO_08  = 0x100,
    ADI_FPGA9001_GPIO_09  = 0x200,
    ADI_FPGA9001_GPIO_10  = 0x400,
    ADI_FPGA9001_GPIO_11  = 0x800,
    ADI_FPGA9001_GPIO_ALL = 0xFFF
} adi_fpga9001_GpioPin_e;


#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_GPIO_TYPES_H_ */