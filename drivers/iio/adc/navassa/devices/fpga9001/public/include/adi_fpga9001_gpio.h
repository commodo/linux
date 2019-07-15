/**
* \file
* \brief Functions to configure and control the FPGA9001 GPIO pins
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/
/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_GPIO_H_
#define _ADI_FPGA9001_GPIO_H_

#include "adi_fpga9001_gpio_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Set the gpioMode
*
* \param device     Pointer to the FPGA9001 data structure
* \param gpioMode   The mode to set
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioModeSet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e gpioMode);

/**
* \brief Get the current gpioMode
*
* \param device     Pointer to the FPGA9001 data structure
* \param gpioMode   The current mode
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioModeGet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e *gpioMode);

/**
* \brief Configure the direction (input/output) for the specified GPIO pins
*
* \note Only applicable for Normal GPIO mode
*
* \param device         Pointer to the FPGA9001 data structure
* \param gpioPinDir     The direction (input/output) to configure the pins
* \param pinSelMask     Bitmask indicating the GPIO pins for which to configure the direction
*                       0 = don't configure; 1 = configure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioDirSet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioPinDir_e gpioPinDir, uint16_t pinSelMask);

/**
* \brief Get the current direction for each GPIO pin
*
* \note Only applicable for Normal GPIO mode
*
* \param device
* \param pinDirMask Bitmask indicating the configured direction for each GPIO pin
*                   1 = output; 0 = input
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioDirGet(adi_fpga9001_Device_t *device, uint16_t *pinDirMask);

/**
* \brief Write the values to the GPIO output register.
*
* \note Only pins configured as outputs will be driven to the specified value
*
* \param device Pointer to the FPGA9001 data structure
* \param wrData The desired output values
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioWrite(adi_fpga9001_Device_t *device, uint16_t wrData);

/**
* \brief Read the values on the GPIO input register
*
* \param device Pointer to the FPGA9001 data structure
* \param rdData The current input values
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
*/
int32_t adi_fpga9001_GpioRead(adi_fpga9001_Device_t *device, uint16_t *rdData);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_GPIO_H_ */