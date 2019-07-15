/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions
*        Analog Devices maintains and provides updates to this code layer.
*        The end user should not modify this file or any code in this directory.
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */
#ifndef _ADI_FPGA9001_HAL_H_
#define _ADI_FPGA9001_HAL_H_

#include "adi_fpga9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Performs a read of a single 32-bit register
*
* \param device Pointer to the FPGA9001 device data structure.
* \param addr Address of the register to be read. Must be on a word boundary
* \param data Pointer where the register value will be assigned
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RegisterRead(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t *data);

/**
* \brief Performs a write of a single 32-bit register
*
* \param device Pointer to the FPGA9001 device data structure.
* \param addr Address of the register to be written. Must be on a word boundary
* \param data The data to write
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RegisterWrite(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t data);

/**
* \brief Performs bit field read
*
* \param device Pointer to the FPGA9001 device data structure.
* \param addr Address of the register where the field resides
* \param fieldVal A pointer to store the value in the field
* \param mask A bit-wise mask that determines which bits are part of the field
* \param startBit The bit location in the addr where the field starts
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RegisterFieldRead(adi_fpga9001_Device_t *device, const uint32_t addr, uint32_t *fieldVal, uint32_t mask, uint8_t startBit);

/**
* \brief Performs bit field write
*
* \param device Pointer to the FPGA9001 device data structure.
* \param addr Address of the register where the field resides
* \param fieldVal The value to write to the field
* \param mask A bit-wise mask that determines which bits are part of the field
* \param startBit The bit location in the addr where the field starts
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RegisterFieldWrite(adi_fpga9001_Device_t *device, const uint32_t addr, uint32_t fieldVal, uint32_t mask, uint8_t startBit);

/**
* \brief Performs a RAM memory read
*
* \param device Pointer to the FPGA9001 device data structure.
* \param ramAddress Address to begin the read from
* \param data An array to hold the values read back from RAM. Must have enough space to store 'length' elements
* \param length The number of words to read starting from ramAddress
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RamRead(adi_fpga9001_Device_t *device, const uint32_t ramAddress, uint32_t data[], uint32_t length);

/**
* \brief Performs a RAM memory write
*
* \param device Pointer to the FPGA9001 device data structure.
* \param ramAddress Address to begin writing to
* \param data An array that holds the values to write to RAM
* \param length The number of words to write starting at ramAddress
*
* \retval ADI_FPGA9001_ACT_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_fpga9001_RamWrite(adi_fpga9001_Device_t *device, const uint32_t ramAddress, uint32_t data[], uint32_t length);


#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_HAL_H_ */
