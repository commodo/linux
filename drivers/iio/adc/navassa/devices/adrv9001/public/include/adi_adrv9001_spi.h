/**
 * \file
 * \brief Contains prototypes and macro definitions for Private ADI HAL wrapper
 *        functions implemented in adi_adrv9001_hal.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_HAL_H_
#define _ADI_ADRV9001_HAL_H_

#include "adi_adrv9001_spi_types.h"
#include "adi_platform.h"

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
/* TODO: Any user changeable #defines need to be moved to adi_adrv9001_user.h */
#define HAL_TIMEOUT_DEFAULT 100         /* 100ms */
#define HAL_TIMEOUT_NONE 0x0            /* Non-blocking */
#define HAL_TIMEOUT_INFINITE 0xFFFFFFFF /* Blocking */
#define HAL_TIMEOUT_MULT 2              /* HAL timeout worse-case factor */

#define MAXSPILOGMESSAGE 64

#define SPIARRAYSIZE 1024
#define SPIARRAYTRIPSIZE ((SPIARRAYSIZE / 3) * 3)

/* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
//TODO: add #defines
#define ADRV9001_HW_RMW_LO_ADDR     0x113        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_HI_ADDR     0x114        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_MASK        0x115        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_DATA        0x116        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_BYTES       0xC         /* Number of bytes required to use HW_RMW */
#define ADRV9001_SPI_BYTES          0x3          /* Number of bytes required to use non HW_RMW */
#define ADRV9001_SPI_WRITE_POLARITY 0x00         /* Write bit polarity for ADRV9001 */


/**
* \brief creates an array acceptable to the ADIHAL layer.
* SPI writes require 3 bytes two address and one data in order to change
* all 8 bits in register. if only a few bits are to be changed then a
* read Modify Write (RMW) operation is needed.
* The ADRV9001 provides a write only RMW which reduces spi transactions.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param wrData the resulting array to be set to the HAL layer.
* \param numWrBytes The number of elements filled in the wrData array
* in a zero indexed array this points to the next empty location.
* \param addr The address to be added to wrData.
* \param mask The mask to be added to wrData, this will be 0 if packing a full byte.
* \param data The data to be added to wrData.
* \param writeFlag The value to be bitwise or'd into the MSB of the 16-bit address
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_DataPack(adi_adrv9001_Device_t *device, uint8_t *wrData, uint16_t *numWrBytes, uint16_t addr, uint8_t mask, uint8_t data, uint8_t writeFlag);

/**
* \brief writes a byte of data to the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr the address of the register to write to.
* \param data the value to write to the register.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Byte_Write(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t data);

/**
* \brief writes an array of bytes of data to the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr[] the addresses of the registers to write to.
* \param data[] the values to write to the registers.
* \param count the number of register addresses to write data to.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Bytes_Write(adi_adrv9001_Device_t *device, const uint16_t addr[], const uint8_t data[], uint32_t count);

/**
* \brief reads a byte of data from the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr the address of the register to read from.
* \param readData a pointer to a location to write the register data to.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Byte_Read(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t *readData);

/**
* \brief reads an array of bytes of data from the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr[] the addresses of the registers to read from.
* \param readData[] the location to put the values to read from the registers.
* \param count the number of register addresses to read data from.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Bytes_Read(adi_adrv9001_Device_t *device, const uint16_t addr[], uint8_t readData[], uint32_t count);

/**
* \brief writes a field of data to the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr the address of the register to write to.
* \param fieldVal the value to write to the register.
* \param mask the mask to use when writing the value to the register.
* \param startBit the bit number where the field starts within the register.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Field_Write(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t  fieldVal, uint8_t mask, uint8_t startBit);

/**
* \brief reads a field of data from the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param addr the address of the register to read from.
* \param fieldVal pointer to a location to put the value read from the register.
* \param mask the mask to use when decoding the value to the register.
* \param startBit the bit number where the field starts within the register.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Field_Read(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t *fieldVal, uint8_t mask, uint8_t startBit);

/**
* \brief writes an array of bytes of data to the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param wrCache[] the values to write to the registers.
* \param count the number of register addresses to write data to.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Cache_Write(adi_adrv9001_Device_t *device, const uint32_t wrCache[], uint32_t count);

/**
* \brief reads an array of bytes of data from the part.
*
* \param device Pointer to the ADRV9001 device data structure.
* \param rdCache[] the addresses of the registers to read from.
* \param readData[] the location to put the values to read from the registers.
* \param count the number of register addresses to read data from.
*
* \retval ADRV9001_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADRV9001_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADRV9001_ACT_ERR_RESET_SPI Recovery action for SPI reset required
* \retval ADRV9001_ACT_NO_ACTION Function completed successfully, no action required
*
*/
int32_t adi_adrv9001_spi_Cache_Read(adi_adrv9001_Device_t *device, const uint32_t rdCache[], uint8_t readData[], uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_HAL_H_ */
