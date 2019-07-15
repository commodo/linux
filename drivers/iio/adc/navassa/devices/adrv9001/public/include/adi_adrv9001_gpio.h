/**
 * \file
 * \brief ADRV9001 GPIO header file
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_GPIO_H_
#define _ADI_ADRV9001_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "adi_common_error.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_types.h"

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/**
 * \brief Gets the temperature in Celsius from the internal temperature sensor
 *
 * \pre This function may be called any time after the device is fully initialized
 *
 * \param[in]  device          Pointer to the device settings structure
 * \param[out] temperature_C   The current temperature, in Celsius
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Temperature_Get(adi_adrv9001_Device_t *device, int16_t *temperature_C);

    
/**
 * \brief Called whenever the BBIC detects a GP_INT assertion to find the source and clear it if possible.
 *
 *  When the BBIC detects a rising edge on the General Purpose Interrupt pin GP_INT, this function
 *  allows the BBIC an easy way to determine the GP_INT source, clear it if possible, and receive a recovery action.
 *
 *  The GP Interrupt pin is the logical OR of all the sources and the GP_Interrupt_Mask. The GP_Interrupt_Mask
 *  bit-field is used to control which of the 25 available interrupt sources can assert the GP_INT pin.  To enable an
 *  available interrupt source for GP_INT, write the corresponding bit in the GP_Interrupt_Mask bit-field to low.
 *  Writing an interrupt source bit in the GP_Interrupt_Mask bit-field to high will disable that interrupt source from
 *  asserting the GP_INT pin. The GP_Interrupt_status read-back will show the current value for all interrupt sources,
 *  even if they are disabled. However, the GP Interrupt pin will only assert for the enabled sources.
 *
 * \pre This function can be called any time after device initialization and after the interrupt mask bits have been set
 *
 * \param device      Pointer to the ADRV9001 data structure
 * \param gpIntStatus Pointer to status read-back word containing the GP_INT source registers. Bit mapping is:
 *
 *     bit[n] | GP Interrupt Mask
 *     -------|-----------------------
 *     bit27  | ADI_ADRV9001_GP_MASK_RX_DP_RECEIVE_ERROR
 *     bit26  | ADI_ADRV9001_GP_MASK_TX_DP_TRANSMIT_ERROR
 *     bit25  | ADI_ADRV9001_GP_MASK_RX_DP_READ_REQUEST_FROM_BBIC
 *     bit24  | ADI_ADRV9001_GP_MASK_TX_DP_WRITE_REQUEST_TO_BBIC
 *     bit23  | Not used
 *     bit22  | Not used
 *     bit21  | Not used
 *     bit20  | ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_3_ERROR
 *     bit19  | ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_2_ERROR
 *     bit18  | ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_1_ERROR
 *     bit17  | ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_0_ERROR
 *     bit16  | ADI_ADRV9001_GP_MASK_MAIN_STREAM_PROCESSOR_ERROR
 *     bit15  | ADI_ADRV9001_GP_MASK_LSSI_RX2_CLK_MCS
 *     bit14  | ADI_ADRV9001_GP_MASK_LSSI_RX1_CLK_MCS
 *     bit13  | ADI_ADRV9001_GP_MASK_CLK_1105_MCS_SECOND
 *     bit12  | ADI_ADRV9001_GP_MASK_CLK_1105_MCS
 *     bit11  | ADI_ADRV9001_GP_MASK_CLK_PLL_LOCK
 *     bit10  | ADI_ADRV9001_GP_MASK_AUX_PLL_LOCK
 *     bit9   | ADI_ADRV9001_GP_MASK_RF_PLL_LOCK2
 *     bit8   | ADI_ADRV9001_GP_MASK_RF_PLL_LOCK1
 *     bit7   | ADI_ADRV9001_GP_MASK_CLK_PLL_LOW_POWER_LOCK
 *     bit6   | ADI_ADRV9001_GP_MASK_TX2_PA_PROTECTION_ERROR
 *     bit5   | ADI_ADRV9001_GP_MASK_TX1_PA_PROTECTION_ERROR
 *     bit4   | ADI_ADRV9001_GP_MASK_CORE_ARM_MONITOR_ERROR
 *     bit3   | ADI_ADRV9001_GP_MASK_CORE_ARM_CALIBRATION_ERROR
 *     bit2   | ADI_ADRV9001_GP_MASK_CORE_ARM_SYSTEM_ERROR
 *     bit1   | ADI_ADRV9001_GP_MASK_CORE_FORCE_GP_INTERRUPT
 *     bit0   | ADI_ADRV9001_GP_MASK_CORE_ARM_ERROR
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_ADRV9001_ACT_ERR_RESET_ARM Recovery action for ARM/WATCHDOG errors
 * \retval ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR Recovery action for errors the BBIC must track and decide
 *                                   if the interrupt is an error or not.
 * \retval ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL ARM signals tracking cals need to be restarted
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpIntHandler(adi_adrv9001_Device_t *device, adi_adrv9001_gpIntStatus_t *gpIntStatus);

/**
 * \brief Sets the General Purpose (GP) interrupt register bit mask for GP_INT.
 *
 * \pre This function can be called any time after device initialization
 *
 * \param device     Pointer to the ADRV9001 data structure
 * \param maskSelect Enum indicating the GP interrupt mask register to write
 * \param maskArray  Data structure holding the GP interrupt masks to write
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG  Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE   Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION       Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpIntMaskSet(adi_adrv9001_Device_t *device, adi_adrv9001_gpMaskSelect_e maskSelect, adi_adrv9001_gpMaskArray_t *maskArray);

/**
 * \brief Gets the General Purpose (GP) interrupt register bit mask for GP_INT.
 *
 * \pre This function can be called any time after device initialization
 *
 * \param device     Pointer to the ADRV9001 data structure
 * \param maskSelect Enum indicating the GP interrupt mask register to write
 * \param maskArray  Pointer to data structure holding the GP interrupt masks retrieved
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG  Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE   Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION       Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpIntMaskGet(adi_adrv9001_Device_t *device, adi_adrv9001_gpMaskSelect_e maskSelect, adi_adrv9001_gpMaskArray_t *maskArray);

/**
 * \brief Reads the General Purpose (GP) interrupt status to determine what caused the GP Interrupt pin to assert.
 *        WARNING: Reading the GP Status will clear the status bits.
 *        DO NOT call this API before calling the API's adi_adrv9001_GpIntHandler
 *
 *  When the BBIC detects a rising edge on either General Purpose Interrupt pin GP_INT, this function
 *  allows the BBIC to determine the source of the interrupt.  The value returned in the status parameter
 *  will show one or more sources for the interrupt based table given below.
 *
 *  The status word read-back will show the current value for all interrupt sources, even if they are disabled by mask register.
 *  However, the GP Interrupt pin will only assert for the enabled sources.
 *
 * \pre This function can be called any time after device initialization and after the interrupt mask bits have been set
 *
 * \param device      Pointer to the ADRV9001 data structure
 * \param gpIntStatus Pointer to status read-back word
 *
 *     bit[n] | GP Interrupt Mask Bit
 *     -------|-----------------------
 *     bit27  | RX_DP_RECEIVE_ERROR
 *     bit26  | TX_DP_TRANSMIT_ERROR
 *     bit25  | RX_DP_READ_REQUEST_FROM_BBIC
 *     bit24  | TX_DP_WRITE_REQUEST_TO_BBIC
 *     bit23  | Not used
 *     bit22  | Not used
 *     bit21  | Not used
 *     bit20  | STREAM_PROCESSOR_3_ERROR
 *     bit19  | STREAM_PROCESSOR_2_ERROR
 *     bit18  | STREAM_PROCESSOR_1_ERROR
 *     bit17  | STREAM_PROCESSOR_0_ERROR
 *     bit16  | MAIN_STREAM_PROCESSOR_ERROR
 *     bit15  | LSSI_RX2_CLK_MCS
 *     bit14  | LSSI_RX1_CLK_MCS
 *     bit13  | CLK_1105_MCS_SECOND
 *     bit12  | CLK_1105_MCS
 *     bit11  | CLK_PLL_LOCK
 *     bit10  | AUX_PLL_LOCK
 *     bit9   | RF_PLL_LOCK2
 *     bit8   | RF_PLL_LOCK1
 *     bit7   | CLK_PLL_LOW_POWER_LOCK
 *     bit6   | TX2_PA_PROTECTION_ERROR
 *     bit5   | TX1_PA_PROTECTION_ERROR
 *     bit4   | CORE_ARM_MONITOR_ERROR
 *     bit3   | CORE_ARM_CALIBRATION_ERROR
 *     bit2   | CORE_ARM_SYSTEM_ERROR
 *     bit1   | CORE_FORCE_GP_INTERRUPT
 *     bit0   | CORE_ARM_ERROR
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpIntStatusGet(adi_adrv9001_Device_t *device, uint32_t *gpIntStatus);


/**
* \brief Gets the ADRV9001 low voltage GPIO pin directions
*
* This function will get the GPIO direction currently set in the device.
* The direction can be either output or input per pin. The return gpioOutEn
* function parameter returns a bit per GPIO pin.  1 = output from the ADRV9001
* Device, 0 = input into the ADRV9001 device.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioOutEn Pointer to a single uint16_t variable that returns the
*                  output enable reading per GPIO pin
*
* gpioOutEn[bit]  |  GPIO direction
* ----------------|-------------------
*        0        |        input
*        1        |        output
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutEnGet(adi_adrv9001_Device_t *device, uint16_t *gpioOutEn);

/**
* \brief Sets the selected ADRV9001 low voltage GPIO pins I/O direction to Input
*
* This function will set the GPIO direction currently for the requested GPIOs to input.
* The direction can be either output or input per pin. The gpioInputMask
* determines the GPIO pins which can be set to input.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioInputMask ADRV9001 GPIO pins which are required to be set as input
*        in the range 0x0000 - 0xFFFF
*
* GPIO_0 corresponds to LSB of gpioInputMask where as GPIO_15 corresponds
* to MSB of gpioInputMask. For example, to set GPIO_0, GPIO_1
* as inputs the gpioInputMask parameter would be equal to 0x0003.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioInputDirSet(adi_adrv9001_Device_t *device, uint16_t gpioInputMask);

/**
* \brief Sets the selected ADRV9001 low voltage GPIO pins I/O direction to Output
*
* This function will set the GPIO direction currently for the requested GPIOs to output.
* The direction can be either output or input per pin. The gpioOutputMask
* determines the GPIO pins which can be set to output.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioOutputMask ADRV9001 GPIO pins which are required to be set as output
*        in the range 0x0000 - 0xFFFF
*
* GPIO_0 corresponds to LSB of gpioOutputMask where as GPIO_15 corresponds
* to MSB of gpioOutputMask. For example, to set GPIO_0, GPIO_1
* as outputs the gpioOutputMask parameter would be equal to 0x0003.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutputDirSet(adi_adrv9001_Device_t *device, uint16_t gpioOutputMask);

/**
* \brief Sets the selected ADRV9001 analog GPIO pins I/O direction to Input
*
* This function will set the analog GPIO direction for the requested analog GPIO pins to input.
* The direction can be either output or input per pin. The gpioAnalogInputMask
* determines the GPIO pins which can be set to input.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioAnalogInputMask ADRV9001 analog GPIO pins which are required to be set as input
*        in the range 0x0000 - 0x0FFF
*
*  GPIO_ANA_0 corresponds to LSB of gpioAnalogInputMask where as GPIO_ANA_11 corresponds
*  to MSB of gpioAnalogInputMask. For example, to set GPIO_ANA_0, GPIO_ANA_1, GPIO_ANA_7,GPIO_ANA_6
*  as inputs the gpioAnalogInputMask parameter would be equal to 0x00C3.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioAnalogInputDirSet(adi_adrv9001_Device_t* device, uint16_t gpioAnalogInputMask);

/**
* \brief Sets the selected ADRV9001 analog GPIO pins I/O direction to Output
*
* This function will set the analog GPIO direction for the requested analog GPIO pins to output.
* The direction can be either output or input per pin. The gpioAnalogInputMask
* determines the GPIO pins which can be set to input.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioAnalogOutputMask ADRV9001 analog GPIO pins which are required to be set as output
*        in the range 0x0000 - 0x0FFF
*
*  GPIO_ANA_0 corresponds to LSB of gpioAnalogOutputMask where as GPIO_ANA_11 corresponds
*  to MSB of gpioAnalogOutputMask. For example, to set GPIO_ANA_0, GPIO_ANA_1, GPIO_ANA_7,GPIO_ANA_6
*  as outputs the gpioAnalogOutputMask parameter would be equal to 0x00C3.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioAnalogOutputDirSet(adi_adrv9001_Device_t* device, uint16_t gpioAnalogOutputMask);

/**
* \brief Sets the ADRV9001 GPIO output source for different GPIO functionality
*
* This function will only affect the GPIO pins that have their OE direction
* set to output.  Each two GPIO pins can be assigned a GPIO source.  Each
* GPIO byte (2 pins) must share that same GPIO output source.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioSrcCtrl Pointer to a gpio source control struture
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutSourceCtrlSet(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOutSourceCtrl_t *gpioSrcCtrl);

/**
* \brief Reads the ADRV9001 GPIO output source for each set of four low voltage
*        GPIO pins.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioSrcCtrl Pointer to a gpio source control structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutSourceCtrlGet(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOutSourceCtrl_t *gpioSrcCtrl);

/**
* \brief Sets the ADRV9001 low voltage GPIO output pins level
*
* This function will only affect the GPIO pins that have their OE direction set to output and
* that have the correct source control for the nibbles in GPIO_BITBANG_MODE
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioOutPinLevel Bit per GPIO pin, level to output for each GPIO pin. 0 = low output, 1 = high output
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT__NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutPinLevelSet(adi_adrv9001_Device_t *device, uint16_t gpioOutPinLevel);

/**
* \brief Reads the ADRV9001 GPIO pin output levels for BITBANG mode
*
*  This function allows reading the value that the GPIO output pins are
*  set to drive out the pins.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioOutPinLevel Pointer to a single uint16_t variable which returns
*                        the level set to output of each output GPIO pin
*                        (bit per pin)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT__NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioOutPinLevelGet(adi_adrv9001_Device_t *device, uint16_t *gpioOutPinLevel);

/**
* \brief Reads the ADRV9001 low voltage GPIO pin levels and returns their
*        contents in a single 16-bit word
*
*  The GPIO pins that are set to be inputs in BITBANG mode will read back and
*  be returned in the gpioPinLevel parameter. The return value is a bit per
*  pin.  GPIO 0 returns on bit 0 of the gpioPinLevel parameter.  A logic low
*  level returns a 0, a logic high level returns a 1.
*
* \param device Pointer to the ADRV9001 data structure
* \param gpioInPinLevel Pointer to a single uint16_t variable which returns the
*                     GPIO pin levels read back on the pins assigned as
*                     inputs (bit per pin)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT__NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_GpioInputPinLevelGet(adi_adrv9001_Device_t *device, uint16_t *gpioInPinLevel);

/**
 * \brief This API function configures the monitor output function for the GPIOs
 *
 * The monitor outputs allow visibility to some internal ADRV9001 signals.  Each
 * monitor index outputs a set of eight signals.  To output these signals on
 * the low voltage GPIO[15:0] pins, first set the desired GPIO[15:0] pin
 * direction, then set the GPIO source control to allow the monitor
 * signals to route to a set of 2 GPIO pins.  
 * When the source is set to monitor out for GPIO[15:0], monitorOut[7:0]
 * is routed to GPIO[7:0] and monitorOut[7:0] is also routed to GPIO[15:8].
 *
 * \param device Pointer to the ADRV9001 data structure
 * \param monitorIndex The index that outputs a set of 8 monitor outputs. See
 *          table in documentation for which signals output on each GPIO pins
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT__NO_ACTION Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpioMonitorOutSrcSet(adi_adrv9001_Device_t *device, uint8_t monitorIndex);

/**
 * \brief This API function reads the GPIO monitor index and mask from ADRV9001
 *
 * \param device Pointer to the ADRV9001 data structure
 * \param monitorIndex Pointer to single uint8_t variable which returns the
 *                     current monitor signal selection index
 *                     
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT__NO_ACTION Function completed successfully, no action required
 */
int32_t adi_adrv9001_GpioMonitorOutSrcGet(adi_adrv9001_Device_t *device, uint8_t *monitorIndex);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_GPIO_H_ */
