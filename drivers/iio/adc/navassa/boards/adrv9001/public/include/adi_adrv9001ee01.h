/**
* \file
* \brief Core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2018 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001EE01_H_
#define _ADI_ADRV9001EE01_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_adrv9001_utilities_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief This function creates a board instance for EE01 daughter card
* This function will create the device instances in the EE01 daughter card
* This function will also create the board instance of EE01
*
* \param adrv9001Ee01 Pointer to the ADRV9001 EE01 daughter card instance
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action returned in case of error (null pointer or unable to allocate structures)
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully
*/
int32_t adi_adrv9001Ee01_Create(adi_adrv9001Ee01_Board_t *adrv9001Ee01Inst);

/**
* \brief This function does the following
*  Closes Device drivers in use by the requested board
*  Deallocates memory consumed by device HAL instances
*  Removes the daughter board instance from the list of daughter cards instantiated
*  Deallocates memory for device instances in the board requested
*  Updates the list of daughter boards active
*
* \param adrv9001Ee01 Pointer to the ADRV9001 EE01 daughter card instance
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001Ee01_Destroy(adi_adrv9001Ee01_Board_t *adrv9001Ee01);

/**
* \brief This function programs the device using a profile file configure the ARM.
*
* \param adrv9001Ee01          Pointer to ADRV9001EE01 daughter card instance to be programmed
* \param profileFilename       String containing the absolute path of the profile file
* \param adrv9001RadioConfig   Pointer to ADRV9001 Radio Config structure
* \param adrv9001PlatformFiles Pointer to platform files on SD card (Arm and Stream bins, Rx Gain table, Tx Atten Table)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
*/
int32_t adi_adrv9001Ee01_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                               const char* profileFilename,
                                               adi_adrv9001_RadioConfig_t *adrv9001RadioConfig,
                                               adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles);

/**
* \brief This function programs the device using an init struct to configure the ARM.
*
* \param adrv9001Ee01          Pointer to ADRV9001EE01 daughter card instance to be programmed
* \param adrv9001Init          Pointer to ADRV9001 init structure
* \param adrv9001RadioConfig   Pointer to ADRV9001 Radio Config structure
* \param adrv9001PlatformFiles Pointer to platform files on SD card (ARM and Stream bins, Rx Gain table, Tx Atten Table)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
*/
int32_t adi_adrv9001Ee01_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                    adi_adrv9001_Init_t *adrv9001Init,
                                    adi_adrv9001_RadioConfig_t *adrv9001RadioConfig,
                                    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles);

#ifdef __cplusplus
}
#endif

#endif
