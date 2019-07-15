/**
* \file
* \brief Contains top level ADRV9001 related function prototypes for
*        adi_adrv9001.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_H_
#define _ADI_ADRV9001_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* ADI specific header files */
#include "adi_common_macros.h"
#include "adi_adrv9001_types.h"

/* Header files related to libraries */


/* System header files */


/*********************************************************************************************************/
/**
 * \brief Performs a Hardware Initialization for ADRV9001 Device.
 *
 * This API calls the ADI HAL function adi_hal_HwOpen for
 * ADRV9001 Hardware initialization.  This HAL function initializes all the external
 * hardware blocks required in the operation of the ADRV9001 device.
 * This API sets the HAL timeout limit for the HAL driver as per API
 * requirements.
 *
 * \pre This function may be called after device->common.devHalInfo has been initialized with
 * user values
 *
 * \param device		Pointer to ADRV9001 device data structure. Adrv9001Device_t member
 *						devHalInfo must be initialized with all the required information to initialize
 *						external Hardware required for ADRV9001 operation for example
 *						power, pull ups, SPI master etc
 * \param spiSettings	Pointer to ADRV9001 SPI controller settings - not platform hardware SPI settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_HwOpen(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spiSettings);

/*********************************************************************************************************/
/**
 * \brief Performs a Hardware Initialization for ADRV9001 Device without resetting the part.
 *
 * This API calls the ADI HAL function adi_hal_HwOpen for
 * ADRV9001 Hardware initialization.  This HAL function initializes all the external
 * hardware blocks required in the operation of the ADRV9001 device.
 * This API sets the HAL timeout limit for the HAL driver as per API
 * requirements.
 * This API will not reset the hardware.
 *
 * \pre This function may be called after device->common.devHalInfo has been initialized with
 * user values
 *
 * \param device		Pointer to ADRV9001 device data structure. Adrv9001Device_t member
 *						devHalInfo must be initialized with all the required information to initialize
 *						external Hardware required for ADRV9001 operation for example
 *						power, pull ups, SPI master etc
 * \param spiSettings	Pointer to ADRV9001 SPI controller settings - not platform hardware SPI settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_HwOpenNoReset(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spiSettings);

/**
 * \brief Performs a hardware shutdown for ADRV9001 Device.
 *
 * This API shall call the ADI HAL function adi_hal_HwClose for
 * ADRV9001 Hardware shutdown.  This HAL function shuts down all the external
 * hardware blocks required in the operation of the ADRV9001 device.
 *
 * \pre This function may be called any time after device->common.devHalInfo has been
 * initialized with user values
 *
 * \param device Pointer to ADRV9001 device data structure. Adrv9001Device_t member
 * devHalInfo shall be initialized with all the required information to initialize
 * supporting Hardware for ADRV9001 operation for example
 * power, pull ups, SPI master etc
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_HwClose(adi_adrv9001_Device_t *device);

/**
 * \brief Performs a hard reset on the ADRV9001 DUT (Toggles RESETB pin on device)
 *
 * Toggles the ADRV9001 devices RESETB pin.  Only resets the device with
 * the SPI chip select indicated in the device->spiSettings structure.
 *
 * \pre This function may be called any time after device->common.devHalInfo has been
 * initialized with user values
 *
 * \param device Pointer to ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_HwReset(adi_adrv9001_Device_t *device);

/**
 * \brief Initializes the ADRV9001 device based on the desired device settings.
 *
 * This function initializes the ADRV9001 device, analog clocks.
 * It does not load the ARM or perform any of the ARM init calibrations.  It leaves the
 * ADRV9001 in a state ready for multichip sync, the
 * ARM to be loaded, and the init calibrations run.
 *
 * \pre This function is the very first API to be called by the user to configure the device
 * after all dependent data structures have been initialized
 *
 * \param[in] device                         Pointer to ADRV9001 device data structure
 * \param[in] init                           Pointer to ADRV9001 initialization settings structures
 * \param[in] adrv9001DeviceClockOutDivisor  ADRV9001 device clock output divisor; An enum type ranging from 0 to 6,
 *                                           the divisor value will be 2^N (1, 2, 4, 8, 16, 32, 64)"?
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_InitAnalog(adi_adrv9001_Device_t *device,
                                adi_adrv9001_Init_t *init,
                                adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor);

/**
 * \brief Initializes the ADRV9001 device based on the desired device settings.
 *
 * This function initializes the ADRV9001 device, digital clocks,
 * FIR Filters, digital filtering.  It does not load the ARM
 * or perform any of the ARM init calibrations.  It leaves the
 * ADRV9001 in a state ready for multichip sync, the
 * ARM to be loaded, and the init calibrations run.
 *
 * \pre This function is the second API to be called by the user to configure the device
 * after all dependent data structures have been initialized
 *
 * \param device Pointer to ADRV9001 device data structure
 * \param init Pointer to ADRV9001 initialization settings structures
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_InitDigital(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init);

/**
 * \brief API To safely Shutdown ADRV9001
 *
 * The User should call this function to safely shutdown ADRV9001 Device.
 * The function performs a hardware reset to reset the ADRV9001 Device into a safe
 * state for shutdown or re-initialization.
 *
 * \pre This function may be called at any time but not before device->common.devHalInfo
 * has been configured with user device settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Shutdown(adi_adrv9001_Device_t *device);

/**
 * \brief Sets the ADRV9001 device SPI settings (3wire/4wire, msbFirst, etc).
 *
 * This function will use the settings in the passed SPI structure parameter
 * to set SPI stream mode, address auto increment direction, msbFirst/lsbFirst,
 * and 3wire/4wire mode for the ADRV9001 SPI controller.  The ADRV9001 device
 * always uses SPI MODE 0 (CPHA=0, CPOL=0) and a 16-bit instruction word.
 *
 * \pre This function is a helper function and does not need to be called
 *      directly by the user.
 *
 * \param device Structure pointer to ADRV9001 device data structure
 * \param SPI Pointer to ADRV9001 SPI controller settings - not platform hardware SPI settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_spi_Configure(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spi);

/**
 * \brief Gets the ADRV9001 device SPI settings (3wire/4wire, msbFirst, etc).
 *
 * This function will use the settings in the passed SPI structure parameter
 * to get SPI stream mode, address auto increment direction, msbFirst/lsbFirst,
 * and 3wire/4wire mode for the ADRV9001 SPI controller.  The ADRV9001 device
 * always uses SPI MODE 0 (CPHA=0, CPOL=0) and a 16-bit instruction word.
 *
 * \pre This function is a helper function and does not need to be called
 *      directly by the user.
 * \pre Can only call this function if config has been set or SpiVerify
 *      returns with no errors
 *
 * \param device Structure pointer to ADRV9001 device data structure
 * \param SPI Pointer to ADRV9001 SPI controller settings - not platform hardware SPI settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_spi_Inspect(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spi);

/**
 * \brief Verifies whether the existing SPI settings work.
 *
 * This function checks the SPI settings set through adi_adrv9001_setSpiSettings for
 * correct functionality. The function performs the following function:
 *
 * 1. Reads readonly register to check SPI read operation.
 * 2. Writes scratchpad register with 10110110, reads back the data
 * 3. Writes scratchpad register with 01001001, reads back the data
 *
 * The function performs the above operation on registers at the lower end of
 * the register address space, and on the upper end of the register address
 * space.
 *
 * \pre This function is a helper function and does not need to be called
 *      directly by the user.
 *
 * \param device Structure pointer to ADRV9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_spi_Verify(adi_adrv9001_Device_t *device);

/**
 * \brief Get API version number
 *
 * This function reads back the version number of the API
 *
 * \param device Pointer to the ADRV9001 data structure
 * \param apiVersion Pointer to structure where API version information is returned
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_ApiVersionGet(adi_adrv9001_Device_t *device, adi_common_ApiVersion_t *apiVersion);

/**
 * \brief Reads back the silicon revision for the ADRV9001 Device
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param siRevision Return value of the ADRV9001 silicon revision
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_DeviceRevGet(adi_adrv9001_Device_t *device, uint8_t *siRevision);

/**
 * \brief Reads back the Product ID for the ADRV9001 Device
 *
 *  productId  |  Description
 * ------------|---------------
 *       1     |  ...
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param productId Return value of the ADRV9001 product Id
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_ProductIdGet(adi_adrv9001_Device_t *device, uint8_t *productId);

/**
 * \brief Verifies whether the init structure profiles are valid.
 *
 * This function checks that the Rx/Tx/ORx profiles have valid clock rates in
 * order to operate together.  Rx/Tx and ORx share a common high speed digital
 * clock. If an invalid combination of profiles is detected, an error will be
 * returned. If a profile in the init structure is unused, the user should zero
 * out all members of that particular profile structure.
 *
 * \param device Structure pointer to ADRV9001 device data structure
 * \param init Pointer to ADRV9001 initialization settings structures
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_ProfilesVerify(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_H_ */
