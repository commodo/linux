/**
* \file
* \brief Contains top level fpga9001 related function prototypes for
*        adi_fpga9001.c
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_H_
#define _ADI_FPGA9001_H_
#include "adi_platform.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_types.h"

#include "adi_fpga9001_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Performs a Hardware Initialization for FPGA Device.
*
* \pre This function may be called after device->common.devHalInfo has been initialized with
* user values
*
* \param device Pointer to FPGA device data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG     Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM    Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION          Function completed successfully, no action required
*/
int32_t adi_fpga9001_HwOpen(adi_fpga9001_Device_t *device);
    
/**
* \brief Performs a hardware shutdown  for FPGA Device.
*
* \pre This function may be called after device->common.devHalInfo has been initialized with
* user values
*
* \param device Pointer to FPGA device data structure
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM    Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION          Function completed successfully, no action required
*/
int32_t adi_fpga9001_HwClose(adi_fpga9001_Device_t *device);

// FIXME: This is an awful way to do this, but it should go away in the future
extern char fpgaBinDirectory[256];
/**
 * \brief Change the value of the fpgaBinDirectory global variable used by adi_fpga9001_SwitchBin()
 * 
 * \param[in] directory The new path to the directory containing FPGA binaries
 */
int32_t FpgaBinDirectorySet(const char* directory);
    
/**
 * \brief Reprogram the FPGA to the specified binary
 *
 * \param[in] device            Pointer to FPGA device data structure
 * \param[in] bin               The binary image to switch to
 * 
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
int32_t adi_fpga9001_SwitchBin(adi_fpga9001_Device_t *device, adi_fpga9001_Binary_e bin);
    
/**
 * \brief Get the version of the FPGA binary 
 * 
 * \param[in]  device          Pointer to FPGA device data structure
 * \param[out] version         The current version
 * 
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM    Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
    int32_t adi_fpga9001_VersionGet(adi_fpga9001_Device_t *device, adi_fpga9001_Version_t *version);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_H_ */