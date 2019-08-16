/**
* \file
* \brief Contains ADRV9001 utility functions to load ARM binaries
*        load stream binaries, load Rx Gain Table, load Tx Atten Table.
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_UTILITIES_H_
#define _ADI_ADRV9001_UTILITIES_H_

#include "adi_adrv9001_utilities_types.h"
#include "jsmn.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief This utility function reads and parses a JSON profile file, loading the contents into an init struct.
*
* \pre The parameter init must have memory fully allocated.
*
* \param[in]  device Pointer to the ADRV9001 device data structure
* \param[in]  profileFilename String containing the absolute path of the profile file
* \param[out] init is an init struct where the contents of the profile will be written
*/
int32_t adi_adrv9001_Utilities_DeviceProfile_Load(adi_adrv9001_Device_t *device, const char *profileFilename, adi_adrv9001_Init_t *init);


int32_t adi_adrv9001_Utilities_DeviceProfile_Parse(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, char *jsonBuffer, uint32_t length);


/**
* \brief This utility function loads ADRV9001 ARM binary image through ArmImageWrite() API
*
* This function reads the ARM binary file from a specified location
* (typically in an SD card) and programs the ARM with the specified binary image
*
* Please note that a large chunk size defined by ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES
* in adi_adrv9001_user.h could potentially cause the stack to crash. Please optimize the chunk size
* in accordance with the stack space available
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param armImagePath is a string containing absolute path of the ARM image to be programmed
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_Utilities_ArmImage_Load(adi_adrv9001_Device_t *device, const char *armImagePath);

/**
* \brief This utility function loads ADRV9001 Stream binary image through StreamImageWrite() API
*
* This function reads the ADRV9001 stream binary file from a specified location
* (typically in an SD card) and programs the stream processor with the specified binary image
*
* Please note that a large chunk size defined by ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES
* in adi_adrv9001_user.h could potentially cause the stack to crash. Please optimize the chunk size
* in accordance with the stack space available.
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param streamImagePath is a string containing absolute path of the stream image to be programmed
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_Utilities_StreamImage_Load(adi_adrv9001_Device_t *device, const char *streamImagePath);

/**
* \brief This utility function load Arm files after Digital init sequence
*
* This function executes the following set of sub-functions required to be
* complete after digital init sequence
*
* - Continue Digital Init (adi_adrv9001_InitDigital)
* - Load ARM binary image from a .bin file
* - Load Stream binary image from a .bin file
* - Load Requested Rx gain tables from .csv files for initialized channels
* - Load Reqeuested Tx atten tables from .csv files for initialized channels
* - Load ARM profile
* - ARM bootup
*
* \param device Pointer to the ADRV9001 device data structure containing settings*
* \param resourceCfg Pointer to ADRV9001 resource config settings structure
*
* Rx Gain Table columns should be arranged in the following order
*
*  --------------------------------------------------------------------------------------------------------
*  | Gain Index | FE Control Word | TIA Control | ADC Control | Ext Control | Phase Offset | Digital Gain |
*  --------------------------------------------------------------------------------------------------------
*
* Rx Gain Indices should be arranged in ascending order for row entries starting with the lowest gain index and
* and progressing to highest gain Index
*
* Eg: If the gain table contains entries from index 192 to index 255, the table has to be arranged such that
*     first line should contain row entries for index 192 and the last line should contain row entries to index 255
*     with a total of 64 entries.
*
* Tx Atten Table columns should be arranged in the following order
*
*  ------------------------------------------------
*  | Tx Atten Index | Tx Atten Hp | Tx Atten Mult |
*  ------------------------------------------------
*
* Tx Atten Indices should be arranged in ascending order for row entries starting with the lowest tx atten index and
* and progressing to highest tx atten Index (Eg: Starting from index 0 and progressing to index 1023)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_Utilities_Resources_Load(adi_adrv9001_Device_t *device,
                                   adi_adrv9001_ResourceCfg_t *resourceCfg);

/**
* \brief This utility function load Digital init sequence and Arm files
*
* This function executes the following set of sub-functions required to be
* complete after analog init sequence
*
* - Run Digital Init (adi_adrv9001_InitDigital)
* - Load ARM binary files (adi_adrv9001_Utilities_Resources_Load)
* - Run Full Digital External MCS Protocol (adi_adrv9001_FullMcsDigitalExt)
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param resourceCfg Pointer to ADRV9001 resource config settings structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_InitDigitalLoad(adi_adrv9001_Device_t *device, adi_adrv9001_ResourceCfg_t *resourceCfg);

/**
* \brief This utility function loads ADRV9001 Rx Gain table file in csv format to ADRV9001 gain table SRAM
*
* This function reads the ADRV9001 Rx Gain Table file in csv format from a specified location
* (typically in an SD card) and programs the ADRV9001 SRAM with the gain table for the requested channels
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param rxGainTablePath is a string containing path to the Rx gain table in csv format to be programmed
* \param rxChannelMask Rx channels to be programmed. Valid masks include
*        ADI_ADRV9001_RX1 - ADI_ADRV9001_RX2.
*
* Rx Gain Table columns should be arranged in the following order
*
*  --------------------------------------------------------------------------------------------------------
*  | Gain Index | FE Control Word | TIA Control | ADC Control | Ext Control | Phase Offset | Digital Gain |
*  --------------------------------------------------------------------------------------------------------
*
* Rx Gain Indices should be arranged in ascending order for row entries starting with the lowest gain index and
* and progressing to highest gain Index
*
* Eg: If the gain table contains entries from index 192 to index 255, the table has to be arranged such that
*     first line should contain row entries for index 192 and the last line should contain row entries to index 255
*     with a total of 64 entries.
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_Utilities_RxGainTable_Load(adi_adrv9001_Device_t *device, const char *rxGainTablePath, uint32_t rxChannelMask);

/**
* \brief This utility function loads ADRV9001 Tx Attenuation table file in csv format to ADRV9001 Tx Atten table SRAM
*
* This function reads the ADRV9001 Tx Atten Table file in csv format from a specified location
* (typically in an SD card) and programs the ADRV9001 SRAM with the atten table for the requested Tx channels
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param txAttenTablePath is a string containing path to the Tx atten table in csv format to be programmed
* \param txChannelMask Tx channels to be programmed. Valid masks include
*        ADI_CHANNEL_1 - ADI_CHANNEL_2.
*
* Tx Atten Table columns should be arranged in the following order
*
*  ------------------------------------------------
*  | Tx Atten Index | Tx Atten Hp | Tx Atten Mult |
*  ------------------------------------------------
*
* Tx atten indices should be arranged in ascending order for row entries starting with the lowest tx atten index and
* and progressing to highest tx atten Index (Eg: Starting from index 0 and progressing to index 1023)
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_Utilities_TxAttenTable_Load(adi_adrv9001_Device_t *device, const char *txAttenTablePath, uint32_t txChannelMask);

/*
* \brief This utility function dumps the ADRV9001 ARM program and data memory through ArmMemRead() API
*
* This function reads the ARM Memory and writes the binary byte array directly to a binary file. First section is program memory followed
* by data memory. The binaryFilename is opened before reading the ARM memory to verify that the filepath is valid and has write access
* before reading ARM memory. A file IO error message will be thrown if write access is not valid for the binaryFilename path.
*
* \param device Pointer to the ADRV9001 device data structure containing settings
* \param binaryFilename is a string containing absolute path of the ARM memory to be dumped
*/
int32_t adi_adrv9001_Utilities_ArmMemory_Dump(adi_adrv9001_Device_t *device, const char *binaryFilename);

/**
* \brief This utility function performs initialization of Radio Controls
*
* This function attempts to place Radio Controls into a required init state
*
* \param device            Pointer to the ADRV9001 device data structure containing settings
* \param initConfig        Pointer to ADRV9001 resource config settings structure
* \param ssiConfigSettings Pointer to SSI calibration configuration parameters
* \param channelMask       The mask of Tx/Rx channels
*
* \parblock              Bit position |  channelMask
*                      ---------------|----------------
*                           bit 0     | RX1
*                           bit 1     | RX2
*                           bit 2     | TX1
*                           bit 3     | TX2
* \endparblock
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adi_adrv9001_InitRadioLoad(adi_adrv9001_Device_t *device, 
                                   int32_t(*Mcs_Requested)(void),
                                   adi_adrv9001_ResourceCfg_t *initConfig,
                                   adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings,
                                   uint8_t channelMask);

/**
* \brief This function programs the SSI delay configuration in ADRV9001 device through SPI.
*
* \param[in] device            Pointer to the ADRV9001 device data structure containing settings
* \param[in] ssiConfigSettings Pointer to SSI calibration configuration parameters
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
*/

int32_t adi_adrv9001_Utilities_SsiDelay_Configure(adi_adrv9001_Device_t *device,
                                       adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings);

/**
* \brief This function gets the SSI delay configuration from ADRV9001 device through SPI.
*
* \param[in] device             Pointer to the ADRV9001 device data structure containing settings
* \param[out] ssiConfigSettings Pointer to SSI calibration configuration parameters
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
* \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
*/

int32_t adi_adrv9001_Utilities_SsiDelay_Inspect(adi_adrv9001_Device_t *device,
                                       adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_UTILITIES_H_ */
