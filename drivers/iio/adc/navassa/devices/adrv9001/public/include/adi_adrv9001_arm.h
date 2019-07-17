/**
 * \file
 * \brief Contains ADRV9001 ARM related function prototypes for adi_adrv9001_arm.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_ARM_H_
#define _ADI_ADRV9001_ARM_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* ADI specific header files */
#include "adi_adrv9001_arm_types.h"
#include "adi_common_error_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_arm.h"

/* Header files related to libraries */


/* System header files */


/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/**
* \brief Enable the ARM processor
*
* Enables AHB SPI bridge
* Sets ARM Run = 1 to enable ARM
* Issue SW interrupt 4 to wake up ARM
* 
* \pre This function is called after the device has been initialized 
*
* \param device Pointer to the ADRV9001 device settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/	
int32_t adi_adrv9001_arm_Enable(adi_adrv9001_Device_t *device);
    
/**
* \brief Disable the ARM processor
*
* Sets ARM Run = 0.
* Disables AHB SPI bridge
* Polls ISR for ARM is READY_FOR_MCS
* 
* \pre This function is called after the device has been initialized 
*
* \param device Pointer to the ADRV9001 device settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_Disable(adi_adrv9001_Device_t *device);

/**
* \brief Enable AHB Spi Bridge
*
* Sets AHB Spi Bridge Enable = 1.
*
* 
* \pre This function is called after the device has been initialized
*
* \param device Pointer to the ADRV9001 device settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_AhbSpiBridge_Enable(adi_adrv9001_Device_t *device);

/**
* \brief Disable AHB Spi Bridge
*
* Sets AHB Spi Bridge Enable = 0.
*
* 
* \pre This function is called after the device has been initialized 
*
* \param device Pointer to the ADRV9001 device settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_AhbSpiBridge_Disable(adi_adrv9001_Device_t *device);

/**
* \brief Check the ARM processor Firmware Status.
*
* After ARM Run = 1, then wait and check for ARM FW Status. This function use a SPI read,
* and can not be use in a SPI write only (Broadcasting) mode.
* 
* \pre This function is called after the device has been initialized and before multichip-sync
* (MCS) has been completed
*
* \param device Pointer to the ADRV9001 device settings data structure
* \param timeout_us Timeout to stop waiting for ARM to boot up.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_StartStatus_Check(adi_adrv9001_Device_t *device, uint32_t timeout_us);

/**
* \brief Check the ARM processor Firmware Status.
*
* After ARM Run = 1, then wait and check for ARM FW Status. This function use a SPI read,
* and can not be use in a SPI write only (Broadcasting) mode.
* 
* \pre This function is called after the device has been initialized and before multichip-sync
* (MCS) has been completed
*
* \param device Pointer to the ADRV9001 device settings data structure
* \param timeout_us Timeout to stop waiting for ARM to boot up.
* \param fwCheckStatus FW status to stop waiting for ARM to change to.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_FwStatus_Check(adi_adrv9001_Device_t *device, uint32_t timeout_us, uint32_t fwCheckStatus);
    
/**
* \brief Writes the ADRV9001 ARM configuration settings
*
* \param device Pointer to the ADRV9001 device settings data structure
* \param init   Pointer to the ADRV9001 initialization settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function  Completed successfully, no action required
*/
int32_t adi_adrv9001_arm_Profile_Write(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init);

/**
* \brief Writes the ADRV9001 PFIR coefficients in ARM memory
*
* \param device Pointer to the ADRV9001 device settings data structure
* \param init   Pointer to the ADRV9001 initialization settings data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function  Completed successfully, no action required
*/
int32_t adi_adrv9001_arm_PfirProfiles_Write(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init);

/**
* \brief Loads binary array into ARM program memory
*
* This function sets the ARM DMA control register bits for an ARM memory write, auto-incrementing
* the address. Valid memory addresses are: Program Memory (0x01000000 - 0x01037FFF)
*
* The top level application reads the binary file, parsing it into any array, starting at the first data byte
* on the first line of the file. The array is passed to this function and writes it to ARM program memory. Any non-data
* bytes should be excluded from the binary array when parsing.
*
* \pre This function is called after the device has been initialized, PLL lock status has been verified, and
* the stream binary has been loaded
*
* \param device     Pointer to the ADRV9001 device data structure containing settings
* \param byteOffset Offset (starting from 0) of where to place the binary array in ARM memory (if loaded in multiple function calls)
* \param binary     Byte array containing all valid ARM file data bytes
* \param byteCount  The number of bytes in the binary array
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_Image_Write(adi_adrv9001_Device_t *device, uint32_t byteOffset, const uint8_t binary[], uint32_t byteCount);

/**
 * \brief Sets bit0 of SW_Interrupt_4 register, which issues wake up interrupt to ARM
 *
 * \param device is a pointer to the device settings structure
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
int32_t adi_adrv9001_arm_WakeupInterrupt_Set(adi_adrv9001_Device_t *device);

/****************************************************************************
* Helper functions
****************************************************************************
*/

/**
* \brief Read from the ADRV9001 ARM program or data memory
*
* Valid memory addresses are: Program Memory (0x01000000 - 0x0101C000),
* Data Memory (0x20000000 - 0x20014000).
*
* \pre This function is private and is not called directly by the user.
*
* \param device Structure pointer to the ADRV9001 data structure containing settings
* \param address The 32bit ARM address to read from.
* \param returnData Byte(uint8_t) array containing the data read from the ARM memory.
* \param byteCount Number of bytes in the returnData array.
* \param autoIncrement is boolean flag to enable or disable auto-increment of ARM register address
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_arm_Memory_Read(adi_adrv9001_Device_t *device, uint32_t address, uint8_t returnData[], uint32_t byteCount, uint8_t autoIncrement);

 /**
  * \brief Write to the ADRV9001 ARM program or data memory
  *
  * Valid memory addresses are: Program Memory (0x01000000 - 0x0101C000),
  * Data Memory (0x20000000 - 0x20014000).
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \param device Structure pointer to the ADRV9001 data structure containing settings
  * \param address The 32-bit ARM address to write
  * \param data Byte array (uint8_t) containing data to be written to ARM memory
  * \param byteCount Number of bytes in the data array to be written
  *
  * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
  * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
  * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
  */
 int32_t adi_adrv9001_arm_Memory_Write(adi_adrv9001_Device_t *device, uint32_t address, const uint8_t data[], uint32_t byteCount);

 /**
 * \brief Low level helper function used by ADRV9001 API to write the ARM memory config structures
 *
 * Normally this function should not be required to be used directly by the BBIC.  This is a helper
 * function used by other ADRV9001 API commands to write settings into the ARM memory.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param objectId ARM id of a particular structure or setting in ARM memory
 * \param byteOffset Byte offset from the start of the objectId's memory location in ARM memory
 * \param data A byte array containing data to write to the ARM memory buffer.
 * \param byteCount Number of bytes in the data array (Valid size = 1-255 bytes)
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_ADRV9001_ACT_ERR_RESET_ARM Recovery action for ARM reset required
 * \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 int32_t adi_adrv9001_arm_Config_Write(adi_adrv9001_Device_t *device, uint8_t objectId, uint16_t byteOffset, const uint8_t data[], uint32_t byteCount);

 /**
 * \brief Low level helper function used by ADRV9001 API to read the ARM memory config structures
 *
 * Normally this function should not be required to be used directly by the BBIC.  This is a helper
 * function used by other ADRV9001 API commands to read settings from the ARM memory.
 *
 * \pre This function is private and is not called directly by the user
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param objectId ARM id of a particular structure or setting in ARM memory
 * \param byteOffset Byte offset from the start of the objectId's memory location in ARM memory
 * \param returnData A byte array containing data read back from the ARM memory buffer
 * \param byteCount Number of bytes in the data array (Valid size = 1-255 bytes)
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval ADI_ADRV9001_ACT_ERR_RESET_ARM Recovery action for ARM reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 int32_t adi_adrv9001_arm_Config_Read(adi_adrv9001_Device_t *device, uint8_t objectId, uint16_t byteOffset, uint8_t returnData[], uint32_t byteCount);

/**
 * \brief Reads the ADRV9001 ARM System Error status
 * 
 * \param[in]  device       Pointer to the ADRV9001 device data structure containing settings
 * \param[out] objectId     The object ID for which the error occurred
 * \param[out] errorCode    The error type within the object
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_arm_SystemError_Get(adi_adrv9001_Device_t *device, uint8_t *objectId, uint8_t *errorCode);
    
 /**
 * \brief Reads the ADRV9001 ARM 64-bit command status register and returns an error and status word
 *
 * A 64-bit status register consisting of a pending bit and three-bit error type is read one byte at
 * a time for the first 16 even-numbered opcodes. The function parses the pending bits and error bits into
 * two (2) separate 16-bit words. statusWord contains the status pending bits. errorWord contains
 * a single error bit if the error type > 0 for any three-bit code.
 * Each word is weighted according to the first 16 even-numbered opcodes where,
 * 0x0001 = opcode '0', 0x0002 = opcode '2', 0x0004 = opcode '4', 0x0008 = opcode '6' and so on.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param errorWord 16-bit error word comprised of weighted bits according to first 16 even-numbered opcodes
 * The weighted bit is set if any three-bit error type > 0, where '0' = OK
 * \param statusWord 16-bit pending bits word comprised of weighted bits according to first 16 even-numbered opcodes
 * The weighted bit is set if an opcode is pending, where '0' = no pending opcode
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
 /* TODO: Need to review number of error and status bits in SPI regs */
 int32_t adi_adrv9001_arm_CmdStatus_Get(adi_adrv9001_Device_t *device, uint16_t *errorWord, uint16_t *statusWord);

 /**
 * \brief Isolated byte read of the ADRV9001 ARM 64-bit command status register based on the opcode
 *
 * A single byte read is performed on the 64-bit command status register according to
 * the opcode of interest. The pending bit and the error type are extracted from the status
 * register and returned as a single byte in the lower nibble.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param opCode Opcode of interest where only the first 16 even-numbered integers are valid
 * \param cmdStatByte Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
 /* TODO: Need to review number of bits SPI regs per opcode */
 int32_t adi_adrv9001_arm_CmdStatusOpcode_Get(adi_adrv9001_Device_t *device, uint8_t opCode, uint8_t *cmdStatByte);

 /**
 * \brief ADRV9001 ARM command status wait function polls command status register until opcode of interest completes
 *
 * \pre ARM firmware load and initialization must take place first before attempting to use this function
 *
 * \param device            Structure pointer to the ADRV9001 data structure containing settings
 * \param callback          Polling callback to execute while waiting for command to complete
 * \param opCode            Opcode of interest where only the first 16 even-numbered integers are valid
 * \param cmdStatusByte     Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
 * \param timeout_us        Command time-out period in microseconds
 * \param waitInterval_us   Wait interval time to thread sleep between each check of the ARM command status to prevent SPI read thrashing
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_ADRV9001_ACT_ERR_RESET_ARM Recovery action for ARM reset required
 * \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
 /* TODO: Need to review number of bits SPI regs per opcode */
int32_t adi_adrv9001_arm_CmdStatus_Wait(adi_adrv9001_Device_t *device, 
                                        int32_t(*callback)(void),
                                        uint8_t opCode,
                                        uint8_t *cmdStatusByte,
                                        uint32_t timeout_us,
                                        uint32_t waitInterval_us);

 /**
 * \brief Sends a command to the ADRV9001 ARM processor interface
 *
 * \pre This function can be called after initializing the ARM processor
 *
 * \param[in] device            Pointer to the ADRV9001 device data structure containing settings
 * \param[in] opCode            Opcode of interest where only the first 16 even-numbered integers are valid
 * \param[in] extendedData      A byte array containing extended data to write to the ARM command interface
 * \param[in] byteCount         Number of bytes in the extendedData array (Valid size = 0-4 bytes)
 *
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
 int32_t adi_adrv9001_arm_Cmd_Write(adi_adrv9001_Device_t *device, uint8_t opCode, const uint8_t extendedData[], uint32_t byteCount);

/**
 * \brief Sends a command SET DEVICE_PROFILE to the ADRV9001 ARM processor interface
 *
 * \pre This function must be called after initializing the ARM processor
 *
 * \param[in] device            Pointer to the ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
 int32_t adi_adrv9001_arm_Profile_Program(adi_adrv9001_Device_t *device);

/**
 * \brief Sends a command SET SYSTEM_CONFIG to the ADRV9001 ARM processor interface
 *
 * \pre This function must be called after initializing the ARM processor
 *
 * \param[in] device           Pointer to the ADRV9001 device data structure containing settings
 * \param[in] Mcs_Requested     Callback function to execute while waiting for init cals to run
 * \param[in] channelMask       The mask of Tx/Rx channels
 * \parblock              Bit position |  channelMask
 *                      ---------------|----------------
 *                           bit 0     | RX1
 *                           bit 1     | RX2
 *                           bit 2     | TX1
 *                           bit 3     | TX2
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_arm_System_Program(adi_adrv9001_Device_t *device,
                                        int32_t(*Mcs_Requested)(void),
                                        uint8_t channelMask);
    
/**
 * \brief Sends a command SET CLOCK_ENABLE to the ADRV9001 ARM processor interface
 *
 * \pre This function must be called after initializing the ARM processor
 *
 * \param device Structure pointer to the ADRV9001 data structure containing settings
 * \param armClock        pointer to adi_adrv9001_ArmClock_t data structure
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 int32_t adi_adrv9001_arm_Clocks_Program(adi_adrv9001_Device_t *device, adi_adrv9001_ArmClock_t *armClock);

 /****************************************************************************
 * Debug functions
 ****************************************************************************
 */

 /**
 * \brief Reads back the version of the ARM binary loaded into the ADRV9001 ARM memory
 *
 * This function reads the ARM memory to read back the major.minor.releaseCandidate
 * version for the ARM binary loaded into ARM memory.
 *
 * <B>Dependencies</B>
 * - device->spiSettings->chipSelectIndex
 *
 * \param device is structure pointer to the ADRV9001 data structure containing settings
 * \param armVersion Arm version will be populated here, it is of struct type adi_adrv9001_ArmVersion_t.
 *
 * \retval ADRV9001_ERR_OK Function completed successfully
 * \retval ADRV9001_ERR_GETARMVER_NULL_PARM One of the function parameters has a NULL pointer
 */
 int32_t adi_adrv9001_arm_Version(adi_adrv9001_Device_t *device, adi_adrv9001_ArmVersion_t *armVersion);

 /**
 * \brief Verifies the ARM checksum value
 *
 * The checksum which is written into the binary file is verified with the calculated
 * checksum in the ADRV9001 ARM after the binary file has been loaded.  This function
 * will wait for a timeout period for the checksum calculation to occur.  The
 * user can adjust the timeout period and SPI read interval in adi_adrv9001_user.c by
 * adjusting the macros VERIFY_ARM_CHKSUM_TIMEOUT_US and
 * VERIFY_ARM_CHKSUM_INTERVAL_US.
 *
 * \pre This function is called after the ARM binary file has been loaded to verify it's
 * checksum in ARM memory
 *
 * \param device is structure pointer to the ADRV9001 data structure containing settings
 * \param checksum Pointer to adi_adrv9001_ChecksumTable_t data structure containing settings
 * \param checksumValid Indicate if checksum is valid
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_ADRV9001_ACT_ERR_RESET_ARM Recovery action for ARM reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 int32_t adi_adrv9001_arm_ChecksumTable_Get(adi_adrv9001_Device_t *device, adi_adrv9001_ChecksumTable_t *checksum, uint8_t *checksumValid);

 /**
 * \brief Reads Arm Mailbox Command Status
 *
 * This function reads Arm Mailbox Command Status from 'arm_command_busy' bit
 * located in 'arm_command' register (0x00c3)
 *
 * \pre This function may be called any time after device initialization
 *
 * \param device is structure pointer to the ADRV9001 data structure containing settings
 * \param mailboxBusy Pointer to byte containing mailbox busy status, 0:Arm is ready to accept a new command, 1:Arm is busy and cannot accept a new command
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
 */
 int32_t adi_adrv9001_arm_MailboxBusy_Get(adi_adrv9001_Device_t *device, uint8_t *mailboxBusy);


#define ADRV9001_ARM_OPCODE_MASK      0xFFFF0000
#define ADRV9001_ARM_OPCODE_SHIFT     16
#define ADRV9001_ARM_OBJ_ID_MASK      0x0000FF00
#define ADRV9001_ARM_OBJ_ID_SHIFT     8
#define ADRV9001_ARM_ERROR_MASK       0x000000FF
#define ADRV9001_ARM_ERROR_SHIFT      0

#define ADRV9001_ARMCMD_ERRCODE(armOpCode, armObjId, armErrorFlag) ((armOpCode << ADRV9001_ARM_OPCODE_SHIFT) | (armObjId << ADRV9001_ARM_OBJ_ID_SHIFT) | armErrorFlag)

#define ADRV9001_OPCODE_VALID(a) (((a) != 0) && (((a) % 2) || ((a) > 30)))

#define ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(devicePtr, opcode, objid, time1, time2) \
{\
    int32_t _recoveryAction = ADI_COMMON_ACT_NO_ACTION; \
    uint8_t _cmdStatusByte = 0; \
    _recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(devicePtr, \
                                                      NULL, \
                                                      (uint8_t)opcode, \
                                                      &_cmdStatusByte, \
                                                      (uint32_t)time1, \
                                                      (uint32_t)time2); \
    /* If cmdStatusByte is non-zero then flag an ARM error */ \
    if((_cmdStatusByte >> 1) > 0) \
    { \
        ADI_EXPECT(adrv9001_ArmCmdErrorHandler, devicePtr, ADRV9001_ARMCMD_ERRCODE(opcode, objid, _cmdStatusByte)); \
    } \
    ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, devicePtr->common.error.errCode, _recoveryAction, NULL, devicePtr->common.error.errormessage); \
    ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
}

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_ARM_H_ */
