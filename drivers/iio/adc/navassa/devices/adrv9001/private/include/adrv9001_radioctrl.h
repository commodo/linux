/**
 * \file
 * \brief Contains ADRV9001 radio control related private function prototypes for
 *        adrv9001_radioctrl.c that helps adi_adrv9001_radioctrl.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_RADIOCTRL_H_
#define _ADRV9001_RADIOCTRL_H_

#include "../devices/adrv9001/public/include/adi_adrv9001_types.h"
#include "../devices/adrv9001/public/include/adi_adrv9001_radio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief This function masks/unmasks the PLL unlock GP interrupt for a requested PLL
*
* When the PLL frequency is being updated, the PLL is unlocked which generates
* a GP interrupt. For use cases where PLL frequency is required to be changed,
* the GP interrupt needs to be masked so that it does not flag a false interrupt.
*
* \param device Pointer to the ADRV9001 data structure
* \param pllNameSel PLL for which frequency needs to be set
* \param gpInterruptMask1 flag to mask(1) / unmask(0) the requested PLL unlock interrupt
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_PllGpInterruptMaskSet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t gpInterruptMask1);

/**
* \brief This function gets PLL unlock GP interrupt mask for a requested PLL
*
* This function can be used to retrieve current GP Interrupt mask status
* for the requested PLL.
*
* \param device Pointer to the ADRV9001 data structure
* \param pllNameSel PLL for which frequency needs to get
* \param pllGpInterruptMask1 Current mask/unmask status of the requested PLL unlock GP Interrupt
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_PllGpInterruptMaskGet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t *pllGpInterruptMask1);

/**
* \brief This function attempts to configure the ADRV9001 ARM Control GPIO pin for the requested signal ID
*
* \pre This function can be called after ARM binary has been loaded.
*
* \param device Pointer to the ADRV9001 data structure
* \param armGpioSigMap Pointer to ARM GPIO pin structure requested for configuration
*
* \retval ADI_ADRV9001_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_ADRV9001_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_ADRV9001_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADI_ADRV9001_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_ArmGpioPinSet(adi_adrv9001_Device_t *device, adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMap);

/**
* \brief This function attempts to retrieve the ADRV9001 ARM Control GPIO pin for the requested signal ID
*
* \pre This function can be called after ARM binary has been loaded.
*
* \param device Pointer to the ADRV9001 data structure
* \param armGpioSignalSel  ARM GPIO Signal.
* \param armGpioSigMapRead pointer to a GPIO Signal Map. 
*        updated the GPIO pin assigned to the requested signal ID.
*
* \retval ADI_ADRV9001_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_ADRV9001_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_ADRV9001_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADI_ADRV9001_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_ArmGpioPinGet(adi_adrv9001_Device_t *device,
        adi_adrv9001_ArmGpioSignal_e armGpioSignalSel,
        adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMapRead);

#ifdef __cplusplus
}
#endif

#endif
