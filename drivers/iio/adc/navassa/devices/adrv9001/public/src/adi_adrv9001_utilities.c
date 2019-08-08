/**
* \file
* \brief Contains Utility features related function implementation defined in
* adi_adrv9001_utilities.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#ifndef free
#define free kfree
#endif

#ifndef calloc
#define calloc(n, s) kcalloc(n, s, GFP_KERNEL)
#endif

#else
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#endif

#include "adi_adrv9001_user.h"
#include "adi_common_macros.h"
#include "adi_adrv9001_utilities.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_stream.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_mcs.h"
#include "adrv9001_Init_t_parser.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_bf_hal.h"
#include "adrv9001_init.h"
#include "adrv9001_utilities.h"

#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_utilities.c"
#endif

#define ADI_ADRV9001_ARM_BINARY_IMAGE_FILE_SIZE_BYTES (256*1024)
#define ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES (32*1024)
#define ADI_ADRV9001_RX_GAIN_TABLE_SIZE_ROWS 256
#define ADI_ADRV9001_TX_ATTEN_TABLE_SIZE_ROWS 1024

#define ADI_ADRV9001_LINE_BUFFER_SIZE 128
#define ADI_ADRV9001_HEADER_BUFFER_SIZE 16

#if (OS_Windows == 64) && (_LABVIEW_INSTALLED != 0)
int32_t adi_adrv9001_Utilities_ArmImage_Load(adi_adrv9001_Device_t *device, const char *armImagePath)
{
    static const size_t BIN_ELEMENT_SIZE = 1;

    int32_t retVal = ADI_COMMON_ACT_NO_ACTION;
    FILE *armImageFilePointer = NULL;
    uint32_t fileSize = 0;
    //TODO: Load in smaller chunks
    static uint8_t armBinaryImageBuffer[ADI_ADRV9001_ARM_BINARY_IMAGE_FILE_SIZE_BYTES];

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /*Open ARM binary file*/
#ifdef __GNUC__
    armImageFilePointer = fopen(armImagePath, "rb");
#else
    if (fopen_s(&armImageFilePointer, armImagePath, "rb") != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Unable to open ARM binary image file. Please check if the path is correct");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    if (armImageFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Invalid ARM binary image path encountered while attempting to load ARM binary image");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Determine file size*/
    if (fseek(armImageFilePointer, 0, SEEK_END) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImageFilePointer,
            "Unable to move file descriptor to the end of the ARM binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    fileSize = ftell(armImageFilePointer);

    /*Check that ARM binary file is not empty*/
    if (fileSize == 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Empty ARM binary image file encountered while attempting to load ARM binary image");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Check that ARM binary file size does not exceed maximum size*/
    if (fileSize > ADI_ADRV9001_ARM_BINARY_IMAGE_FILE_SIZE_BYTES)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "ARM binary image file exceeds maximum ARM binary image size");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Check that size of the file is a multiple of 4*/
    if ((fileSize % 4) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "ARM binary image file is expected to be a multiple of 4");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Rewind the file pointer to beginning of the file*/
    if (fseek(armImageFilePointer, 0, SEEK_SET) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImageFilePointer,
            "Unable to move file descriptor to the beginning of the ARM binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Read ARM binary file*/
    if (fread(&armBinaryImageBuffer[0], BIN_ELEMENT_SIZE, fileSize, armImageFilePointer) < fileSize)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Fatal error while reading ARM binary file. Possible memory shortage");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Write the ARM binary*/
    retVal = adi_adrv9001_arm_Image_Write(device, 0, &armBinaryImageBuffer[0], fileSize);

    /*Close ARM binary file*/
    if (fclose(armImageFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Fatal error while trying to close ARM binary file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return retVal;
}

int32_t adi_adrv9001_Utilities_StreamImage_Load(adi_adrv9001_Device_t *device, const char *streamImagePath)
{
    static const size_t BIN_ELEMENT_SIZE = 1;

    int32_t retVal = ADI_COMMON_ACT_NO_ACTION;
    FILE *streamImageFilePointer = NULL;
    uint32_t fileSize = 0;
    //TODO: Load in smaller chunks
    static uint8_t streamBinaryImageBuffer[ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES];

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /*Open ARM binary file*/
#ifdef __GNUC__
    streamImageFilePointer = fopen(streamImagePath, "rb");
#else
    if (fopen_s(&streamImageFilePointer, streamImagePath, "rb") != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Unable to open stream binary image file. Please check if the path is correct");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    if (streamImageFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Invalid Stream binary image path encountered while attempting to load Stream processor");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Determine file size*/
    if (fseek(streamImageFilePointer, 0, SEEK_END) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImageFilePointer,
            "Unable to move file descriptor to the end of the stream binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    fileSize = ftell(streamImageFilePointer);

    /*Check that Stream binary file is not empty*/
    if (fileSize == 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Empty Stream binary image file encountered while attempting to load the Stream processor");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /*Check that Stream binary file size does not exceed maximum size*/
    if (fileSize > ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Stream binary image file exceeds maximum Stream binary image size");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /*Check that size of the file is a multiple of 4*/
    if ((fileSize % 4) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Stream binary image file is expected to be a multiple of 4");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /*Rewind the file pointer to beginning of the file*/
    if (fseek(streamImageFilePointer, 0, SEEK_SET) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImageFilePointer,
            "Unable to move file descriptor to the beginning of the stream binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /*Read Stream binary file*/
    if (fread(&streamBinaryImageBuffer[0], BIN_ELEMENT_SIZE, fileSize, streamImageFilePointer) < fileSize)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Fatal error while reading stream binary file. Possible memory shortage");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /*Write Stream binary*/
    retVal = adi_adrv9001_Stream_Image_Write(device, 0, &streamBinaryImageBuffer[0], fileSize);

    /*Close Stream binary file*/
    if (fclose(streamImageFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Fatal error while trying to close stream binary file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }


    return retVal;
}

#else

int32_t adi_adrv9001_Utilities_DeviceProfile_Load(adi_adrv9001_Device_t *device, const char *profileFilename, adi_adrv9001_Init_t *init)
{
    static const int16_t ADI_ADRV9001_TOKEN_MAX_LENGTH = 32;

    uint16_t ii = 0;
    int16_t numTokens = 0;
    uint32_t length = 0;
    char * jsonBuffer = 0;
    jsmn_parser parser = { 0 };
    jsmntok_t * tokens = NULL;
    char parsingBuffer[ADI_ADRV9001_TOKEN_MAX_LENGTH]; /* This buffer only needs to hold a stringified number like '123.4567'. */

    ADI_EXPECT(adrv9001_SafeFileLoad, device, profileFilename, &jsonBuffer, &length);

    /* initialize the JSMN parser and determine the number of JSON tokens */
    jsmn_init(&parser);
    numTokens = jsmn_parse(&parser, jsonBuffer, length, NULL, 0);

    /* The JSON file must be tokenized successfully. */
    if (numTokens < 1)
    {
        free(jsonBuffer);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            NULL,
            "Fatal error while parsing profile file. The JSON may be invalid, or the token buffer may be too small.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* allocate space for tokens */
    tokens = (jsmntok_t*)calloc(numTokens, sizeof(jsmntok_t));

    if (NULL == tokens)
    {
        free(jsonBuffer);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_MEM_ALLOC_FAIL,
            ADI_COMMON_ACT_ERR_RESET_FULL,
            NULL,
            "Fatal error while reading profile file. Possible memory shortage.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* initialize the JSMN parser and parse the profile file into the tokens array */
    jsmn_init(&parser);
    numTokens = jsmn_parse(&parser, jsonBuffer, length, tokens, numTokens);

    /* The top-level element must be an object. */
    if (numTokens < 1 || tokens[0].type != JSMN_OBJECT)
    {
        free(tokens);
        free(jsonBuffer);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            NULL,
            "Fatal error while parsing profile file. The JSON may be invalid, or the token buffer may be too small.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Loop over all keys of the root object, searching for matching fields. */
    for (ii = 1; ii < numTokens; ii++)
    {
        ADI_ADRV9001_INIT_T(tokens, ii, jsonBuffer, parsingBuffer, (*init));
    }

    free(tokens);
    free(jsonBuffer);
    tokens = NULL;
    jsonBuffer = NULL;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Utilities_ArmImage_Load(adi_adrv9001_Device_t *device, const char *armImagePath)
{
#ifdef __KERNEL__
	adi_hal_Cfg_t *hal = device->common.devHalInfo;
	struct spi_device *spi = hal->spi;
	uint32_t i,numFileChunks = 0;
	int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, armImagePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev,
			"request_firmware(%s) failed with %d\n", armImagePath, ret);
	}

	numFileChunks = fw->size / ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES;

	/*Check that ARM binary file is not empty*/
	if (ret || (fw->size == 0))
	{
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
				 "Empty ARM binary image file encountered while attempting to load ARM binary image");
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	/*Check that ARM binary file size does not exceed maximum size*/
	if (fw->size > ADI_ADRV9001_ARM_BINARY_IMAGE_FILE_SIZE_BYTES)
	{
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
				 "ARM binary image file exceeds maximum ARM binary image size");
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	/*Check that size of the file is a multiple of 4*/
	if ((fw->size % 4) != 0)
	{
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
				 "ARM binary image file is expected to be a multiple of 4");
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	/*Check that size of the file is divisible into equal sized chunks*/
	if ((fw->size % numFileChunks) != 0)
	{
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
				 "ARM binary image chunk size is expected to divide the stream file into equal chunks");
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	/*Read ARM binary file*/
	for (i = 0; i < numFileChunks; i++)
	{
		/*Write the ARM binary chunk*/
		if ((recoveryAction = adi_adrv9001_arm_Image_Write(device,
			(i * ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES),
			&fw->data[(i * ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES)],
			ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES)) != ADI_COMMON_ACT_NO_ACTION)
		{
			ADI_ERROR_RETURN(device->common.error.newAction);
		}
	}

	release_firmware(fw);

	return recoveryAction;
#else
    static const size_t BIN_ELEMENT_SIZE = 1;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    FILE *armImageFilePointer = NULL;
    uint32_t fileSize = 0;
    uint32_t numFileChunks = 0;
    uint32_t i = 0;
    uint8_t armBinaryImageBuffer[ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES];

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /*Open ARM binary file*/
#ifdef __GNUC__
    armImageFilePointer = fopen(armImagePath, "rb");
#else
    if (fopen_s(&armImageFilePointer, armImagePath, "rb") != 0)
    {
        ADI_ERROR_REPORT(&device->common,  ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                     "Unable to open ARM binary image file. Please check if the path is correct");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    if (armImageFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                     "Invalid ARM binary image path encountered while attempting to load ARM binary image");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Determine file size*/
    if (fseek(armImageFilePointer, 0, SEEK_END) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImageFilePointer,
                     "Unable to move file descriptor to the end of the ARM binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    fileSize = ftell(armImageFilePointer);
    numFileChunks = (uint32_t)(fileSize / ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES);

    /*Check that ARM binary file is not empty*/
    if (fileSize == 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                     "Empty ARM binary image file encountered while attempting to load ARM binary image");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Check that ARM binary file size does not exceed maximum size*/
    if (fileSize > ADI_ADRV9001_ARM_BINARY_IMAGE_FILE_SIZE_BYTES)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                     "ARM binary image file exceeds maximum ARM binary image size");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Check that size of the file is a multiple of 4*/
    if ((fileSize % 4) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                     "ARM binary image file is expected to be a multiple of 4");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Check that size of the file is divisible into equal sized chunks*/
    if ((fileSize % numFileChunks) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "ARM binary image chunk size is expected to divide the stream file into equal chunks");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Rewind the file pointer to beginning of the file*/
    if (fseek(armImageFilePointer, 0, SEEK_SET) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImageFilePointer,
                     "Unable to move file descriptor to the beginning of the ARM binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
    }

    /*Read ARM binary file*/
    for (i = 0; i < numFileChunks; i++)
    {
        if (fread(&armBinaryImageBuffer[0], BIN_ELEMENT_SIZE, ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES, armImageFilePointer) < ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                "Fatal error while reading ARM binary file. Possible memory shortage");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
        }

        /*Write the ARM binary chunk*/
        if ((recoveryAction = adi_adrv9001_arm_Image_Write(device, (i*ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES), &armBinaryImageBuffer[0],
                                                ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES)) != ADI_COMMON_ACT_NO_ACTION)
        {
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, armImageFilePointer);
        }
    }


    /*Close ARM binary file*/
    if (fclose(armImageFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
                         "Fatal error while trying to close ARM binary file");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return recoveryAction;
#endif
}

int32_t adi_adrv9001_Utilities_StreamImage_Load(adi_adrv9001_Device_t *device, const char *streamImagePath)
{
#ifdef __KERNEL__ /* FIXME: Later */
	BUG();
	return 0;
#else
    static const size_t BIN_ELEMENT_SIZE = 1;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    FILE *streamImageFilePointer = NULL;
    uint32_t fileSize = 0;
    uint32_t numFileChunks = 0;
    uint32_t i = 0;
    uint8_t streamBinaryImageBuffer[ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES];

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Open ARM binary file */
#ifdef __GNUC__
    streamImageFilePointer = fopen(streamImagePath, "rb");
#else
    if (fopen_s(&streamImageFilePointer, streamImagePath, "rb") != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, armImagePath,
            "Unable to open stream binary image file. Please check if the path is correct");
        ADI_ERROR_RETURN(device->common.error.newAction);
}
#endif

    if (streamImageFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Invalid Stream binary image path encountered while attempting to load Stream processor");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Determine file size*/
    if (fseek(streamImageFilePointer, 0, SEEK_END) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImageFilePointer,
            "Unable to move file descriptor to the end of the stream binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    fileSize = ftell(streamImageFilePointer);
    numFileChunks = (uint32_t)(fileSize / ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES);

    /* Check that Stream binary file is not empty */
    if (fileSize == 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Empty Stream binary image file encountered while attempting to load the Stream processor");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /* Check that Stream binary file size does not exceed maximum size */
    if (fileSize > ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Stream binary image file exceeds maximum Stream binary image size");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /* Check that size of the file is divisible into equal sized chunks */
    if ((fileSize % numFileChunks) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
           "Stream binary image chunk size is expected to divide the stream file into equal chunks");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /* Check that size of the file is a multiple of 4 */
    if ((fileSize % 4) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Stream binary image file is expected to be a multiple of 4");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /* Rewind the file pointer to beginning of the file */
    if (fseek(streamImageFilePointer, 0, SEEK_SET) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImageFilePointer,
            "Unable to move file descriptor to the beginning of the stream binary image file");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
    }

    /* Read Stream binary file */
    for (i = 0; i < numFileChunks; i++)
    {
        if (fread(&streamBinaryImageBuffer[0], BIN_ELEMENT_SIZE, ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES, streamImageFilePointer) < ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
                "Fatal error while reading stream binary file. Possible memory shortage");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, streamImageFilePointer);
        }

        /* Write Stream binary */
        recoveryAction = adi_adrv9001_Stream_Image_Write(device, (i*ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES), &streamBinaryImageBuffer[0], ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES);
    }

    /* Close Stream binary file */
    if (fclose(streamImageFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, streamImagePath,
            "Fatal error while trying to close stream binary file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return recoveryAction;
#endif
}
#endif

int32_t adi_adrv9001_Utilities_Resources_Load(adi_adrv9001_Device_t *device, adi_adrv9001_ResourceCfg_t *resourceCfg)
{
    int32_t origLogLevel = 0;
    int32_t logLevelNoSpi = 0;
    adi_adrv9001_Init_t *init = NULL;
    const char *armImagePath = NULL;
    //const char *streamImagePath = NULL;

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, resourceCfg);
    ADI_NULL_PTR_RETURN(&device->common, resourceCfg->adrv9001Init);
    ADI_NULL_PTR_RETURN(&device->common, resourceCfg->adrv9001PlatformFiles);
    ADI_NULL_PTR_RETURN(&device->common, resourceCfg->adrv9001PlatformFiles->armImageFile);
    ADI_NULL_PTR_RETURN(&device->common, resourceCfg->adrv9001PlatformFiles->streamImageFile);

    init = resourceCfg->adrv9001Init;
    armImagePath = (char *)resourceCfg->adrv9001PlatformFiles->armImageFile;
    //streamImagePath = (char *)resourceCfg->adrv9001PlatformFiles->streamImageFile;

    ADI_EXPECT(adi_adrv9001_arm_AhbSpiBridge_Enable, device);

    /* Disable SPI Logging */
    adi_common_LogLevelGet(&device->common, &origLogLevel);
    ADI_ERROR_RETURN(device->common.error.newAction);

    logLevelNoSpi = ADI_COMMON_LOG_MSG | ADI_COMMON_LOG_WARN | ADI_COMMON_LOG_ERR | ADI_COMMON_LOG_API; /*Disable SPI logging*/

    adi_common_LogLevelSet(&device->common, logLevelNoSpi);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Load Stream binary image from a .bin file */
    //ADI_EXPECT(adi_adrv9001_Utilities_StreamImage_Load, device, streamImagePath);
    //ADI_ERROR_RETURN(device->common.error.newAction);

    /* Load ARM binary image from a.bin file */
    ADI_EXPECT(adi_adrv9001_Utilities_ArmImage_Load, device, armImagePath);

    /* Load ARM profile */
    ADI_EXPECT(adi_adrv9001_arm_Profile_Write, device, init);

    /* Load PFIR coefficients */
    ADI_EXPECT(adi_adrv9001_arm_PfirProfiles_Write, device, init);

    /* ARM Bootup */
    ADI_EXPECT(adrv9001_ArmStart, device, init);

    /* ARM Bootup Status Check (Non Broadcast mode) */
#if ADI_ADRV9001_PRE_MCS_BROADCAST_DISABLE > 0
    adi_adrv9001_arm_StartStatus_Check(device, ADI_ADRV9001_GETARMBOOTUP_TIMEOUT_US);
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED > 0
    adi_common_ErrorClear(&device->common);
#endif
    ADI_API_RETURN(device);
#endif

    /* Enable SPI logging */
    adi_common_LogLevelSet(&device->common, origLogLevel);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_InitDigitalLoad(adi_adrv9001_Device_t *device, adi_adrv9001_ResourceCfg_t *resourceCfg)
{
    static const uint16_t DEFAULT_ATTENUATION_MDB = 10000;

    static const adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };
    uint8_t i = 0;
    adi_adrv9001_Init_t *init = NULL;

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, resourceCfg);
    ADI_API_ENTRY_PTR_EXPECT(device, resourceCfg->adrv9001Init);

    init = resourceCfg->adrv9001Init;

    /* Setup SPI controller, master bias, enable pin pads, Load PFIRs */
    ADI_EXPECT(adi_adrv9001_InitDigital, device, init);

    ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, device, 2);
    
    ADI_EXPECT(adi_adrv9001_Utilities_Resources_Load, device, resourceCfg);
    
    if (device->devStateInfo.profilesValid & ADI_ADRV9001_TX_PROFILE_VALID) 
    {
        device->devStateInfo.outputSignaling[0] = resourceCfg->adrv9001Init->tx.txProfile[0].outputSignaling;
        device->devStateInfo.outputSignaling[1] = resourceCfg->adrv9001Init->tx.txProfile[1].outputSignaling;
        device->devStateInfo.txInputRate_kHz[0] = resourceCfg->adrv9001Init->tx.txProfile[0].txInputRate_Hz / 1000;
        device->devStateInfo.txInputRate_kHz[1] = resourceCfg->adrv9001Init->tx.txProfile[1].txInputRate_Hz / 1000;
        for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
        {
            /* For each tx channel enabled */
            if ((init->tx.txInitChannelMask & channels[i]) > 0)
            {
                ADI_EXPECT(adi_adrv9001_Tx_Attenuation_Configure,
                               device,
                               channels[i],
                               &resourceCfg->adrv9001RadioConfig->radioCtrlInit.txAttenConfig);

                ADI_EXPECT(adi_adrv9001_Tx_Attenuation_Set, device, channels[i], DEFAULT_ATTENUATION_MDB);
            }
        }
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Utilities_RxGainTable_Load(adi_adrv9001_Device_t *device, const char *rxGainTablePath, uint32_t rxChannelMask)
{
#ifdef __KERNEL__
	adi_hal_Cfg_t *hal = device->common.devHalInfo;
	struct spi_device *spi = hal->spi;
	const struct firmware *fw;
	int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
	uint8_t minGainIndex = 0;
	uint8_t maxGainIndex = 0;
	uint8_t prevGainIndex = 0;
	uint8_t gainIndex = 0;
	uint8_t tiaControl = 0;
	uint8_t adcControl = 0;
	uint16_t lineCount = 0;
	static adi_adrv9001_RxGainTableRow_t rxGainTableRowBuffer[ADI_ADRV9001_RX_GAIN_TABLE_SIZE_ROWS];
	static const uint8_t NUM_COLUMNS = 7;
	int ret;
	const char header[] = {"Gain Index,FE Control Word,TIA Control,ADC Control,Ext Control,Phase Offset,Digital Gain"};
	char *line, *ptr;

	/* Check device pointer is not null */
	ADI_API_ENTRY_PTR_EXPECT(device, rxGainTablePath);

	ret = request_firmware(&fw, rxGainTablePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev,
			"request_firmware(%s) failed with %d\n",rxGainTablePath, ret);
		return ret;
	}

	ptr = (char *) fw->data;

	while ((line = strsep(&ptr, "\n"))  && (lineCount <  ADI_ADRV9001_RX_GAIN_TABLE_SIZE_ROWS)) {
		if (line >= (char *)fw->data + fw->size)
			break;

		line = skip_spaces(line);

		if (sscanf(line,
			"%hhu,%hhu,%hhu,%hhu,%hhu,%hu,%hd",
			&gainIndex,
			&rxGainTableRowBuffer[lineCount].rxFeGain,
			&tiaControl,
			&adcControl,
			&rxGainTableRowBuffer[lineCount].extControl,
			&rxGainTableRowBuffer[lineCount].phaseOffset,
			&rxGainTableRowBuffer[lineCount].digGain) != NUM_COLUMNS) {

			ret = strncmp(line, header , strlen(header));
			if (ret == 0)
				continue;

			ADI_ERROR_REPORT(&device->common,
					 ADI_COMMON_ERRSRC_API,
					ADI_COMMON_ERR_INV_PARAM,
					ADI_COMMON_ACT_ERR_CHECK_PARAM,
					lineCount,
					"Insufficient entries in Rx gain table row entry");
			release_firmware(fw);
			ADI_ERROR_RETURN(device->common.error.newAction);
		}

		rxGainTableRowBuffer[lineCount].adcTiaGain = ((adcControl << 1) | tiaControl);

		if (lineCount == 0)
		{
			minGainIndex = gainIndex;
		}
		else
		{
			/*Check that gain indices are arranged in ascending order*/
			if (prevGainIndex != (gainIndex - 1))
			{
				ADI_ERROR_REPORT(&device->common,
						 ADI_COMMON_ERRSRC_API,
		     ADI_COMMON_ERR_INV_PARAM,
		     ADI_COMMON_ACT_ERR_CHECK_PARAM,
		     gainIndex,
		     "Gain indices not arranged in ascending order in Rx Gain Table file");
				release_firmware(fw);
				ADI_ERROR_RETURN(device->common.error.newAction);
			}
		}

		prevGainIndex = gainIndex;
		lineCount++;
	}

	maxGainIndex = gainIndex;
	recoveryAction = adi_adrv9001_Rx_GainTable_Write(device, rxChannelMask, maxGainIndex, &rxGainTableRowBuffer[0], lineCount);
	if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
	{
		ADI_ERROR_REPORT(&device->common,
				 ADI_COMMON_ERRSRC_API,
		   ADI_COMMON_ERR_INV_PARAM,
		   ADI_COMMON_ACT_ERR_CHECK_PARAM,
		   rxGainTablePath,
		   "Unable to Write Rx gain table");
		release_firmware(fw);
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	recoveryAction = adi_adrv9001_Rx_MinMaxGainIndex_Set(device, rxChannelMask, minGainIndex, maxGainIndex);
	if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
	{
		ADI_ERROR_REPORT(&device->common,
				 ADI_COMMON_ERRSRC_API,
		   ADI_COMMON_ERR_INV_PARAM,
		   ADI_COMMON_ACT_ERR_CHECK_PARAM,
		   rxGainTablePath,
		   "Unable to set Rx gain table min/max gain indices.");
		release_firmware(fw);
		ADI_ERROR_RETURN(device->common.error.newAction);
	}


	release_firmware(fw);

	return recoveryAction;


#else
    static const uint8_t NUM_COLUMNS = 7;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t minGainIndex = 0;
    uint8_t maxGainIndex = 0;
    uint8_t prevGainIndex = 0;
    uint8_t gainIndex = 0;
    uint8_t tiaControl = 0;
    uint8_t adcControl = 0;
    uint16_t lineCount = 0;
    FILE *rxGainTableFilePointer = NULL;
    char rxGainTableLineBuffer[ADI_ADRV9001_LINE_BUFFER_SIZE];
    char headerStr1[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr2[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr3[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr4[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr5[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr6[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr7[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    static adi_adrv9001_RxGainTableRow_t rxGainTableRowBuffer[ADI_ADRV9001_RX_GAIN_TABLE_SIZE_ROWS];

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, rxGainTablePath);

    /*Open Rx Gain Table csv file*/
#ifdef __GNUC__
    rxGainTableFilePointer = fopen(rxGainTablePath, "r");

    if (rxGainTableFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            rxGainTablePath,
            "Invalid Rx Gain Table csv file path encountered while attempting to load Rx Gain Table");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#else
    if (fopen_s(&rxGainTableFilePointer, rxGainTablePath, "r") != 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            rxGainTablePath,
            "Unable to open Rx Gain Table csv file. Please check if the path is correct or the file is open in another program");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /*Check for empty Rx Gain Table*/
    if (fgets(rxGainTableLineBuffer, sizeof(rxGainTableLineBuffer), rxGainTableFilePointer) != NULL)
    {
#ifdef __GNUC__
        if (sscanf(rxGainTableLineBuffer,
            "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^\n]",
            headerStr1,
            headerStr2,
            headerStr3,
            headerStr4,
            headerStr5,
            headerStr6,
            headerStr7) != NUM_COLUMNS)
#else
        if (sscanf_s(rxGainTableLineBuffer,
            "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^\n]",
             headerStr1,
             (uint32_t)sizeof(headerStr1),
             headerStr2,
             (uint32_t)sizeof(headerStr2),
             headerStr3,
             (uint32_t)sizeof(headerStr3),
             headerStr4,
             (uint32_t)sizeof(headerStr4),
             headerStr5,
             (uint32_t)sizeof(headerStr5),
             headerStr6,
             (uint32_t)sizeof(headerStr6),
             headerStr7,
             (uint32_t)sizeof(headerStr7)) != NUM_COLUMNS)
#endif
        {
             ADI_ERROR_REPORT(&device->common,
                 ADI_COMMON_ERRSRC_API,
                 ADI_COMMON_ERR_INV_PARAM,
                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                 rxGainTablePath,
                 "Invalid Rx Gain Table format encountered while attempting to load Rx Gain Table");
                ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        /*Verify that Gain Table Format is correct*/
        if (strstr(headerStr1, "Gain Index") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'Gain Index' to be the first column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr2, "FE Control Word") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'FE Control Word' to be the second column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr3, "TIA Control") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'TIA Control' to be the third column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr4, "ADC Control") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'ADC Control' to be the fourth column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr5, "Ext Control") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'Ext Control' to be the fifth column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr6, "Phase Offset") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected 'Phase Offset' to be the sixth column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        if (strstr(headerStr7, "Digital Gain") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Expected Digital Gain to be the seventh column in Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        /*Loop until the gain table end is reached or no. of lines scanned exceeds maximum*/
        while ((fgets(rxGainTableLineBuffer, sizeof(rxGainTableLineBuffer), rxGainTableFilePointer) != NULL) &&
               (lineCount <  ADI_ADRV9001_RX_GAIN_TABLE_SIZE_ROWS))
        {
#ifdef __GNUC__
            if (sscanf(rxGainTableLineBuffer,
                "%hhu,%hhu,%hhu,%hhu,%hhu,%hu,%hd",
                &gainIndex,
                &rxGainTableRowBuffer[lineCount].rxFeGain,
                &tiaControl,
                &adcControl,
                &rxGainTableRowBuffer[lineCount].extControl,
                &rxGainTableRowBuffer[lineCount].phaseOffset,
                &rxGainTableRowBuffer[lineCount].digGain) != NUM_COLUMNS)
#else
            if (sscanf_s(rxGainTableLineBuffer,
                "%hhu,%hhu,%hhu,%hhu,%hhu,%hu,%hd",
                 &gainIndex,
                 &rxGainTableRowBuffer[lineCount].rxFeGain,
                 &tiaControl,
                 &adcControl,
                 &rxGainTableRowBuffer[lineCount].extControl,
                 &rxGainTableRowBuffer[lineCount].phaseOffset,
                 &rxGainTableRowBuffer[lineCount].digGain) != NUM_COLUMNS)
#endif
            {
                ADI_ERROR_REPORT(&device->common,
                    ADI_COMMON_ERRSRC_API,
                    ADI_COMMON_ERR_INV_PARAM,
                    ADI_COMMON_ACT_ERR_CHECK_PARAM,
                    lineCount,
                    "Insufficient entries in Rx gain table row entry");
                    ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
            }

            rxGainTableRowBuffer[lineCount].adcTiaGain = ((adcControl << 1) | tiaControl);

            if (lineCount == 0)
            {
                minGainIndex = gainIndex;
            }
            else
            {
                /*Check that gain indices are arranged in ascending order*/
                if (prevGainIndex != (gainIndex - 1))
                {
                    ADI_ERROR_REPORT(&device->common,
                        ADI_COMMON_ERRSRC_API,
                        ADI_COMMON_ERR_INV_PARAM,
                        ADI_COMMON_ACT_ERR_CHECK_PARAM,
                        gainIndex,
                        "Gain indices not arranged in ascending order in Rx Gain Table file");
                    ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
                }
            }

            prevGainIndex = gainIndex;
            lineCount++;
        }

        maxGainIndex = gainIndex;
        recoveryAction = adi_adrv9001_Rx_GainTable_Write(device, rxChannelMask, maxGainIndex, &rxGainTableRowBuffer[0], lineCount);
        if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Unable to Write Rx gain table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }

        recoveryAction = adi_adrv9001_Rx_MinMaxGainIndex_Set(device, rxChannelMask, minGainIndex, maxGainIndex);
        if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                rxGainTablePath,
                "Unable to set Rx gain table min/max gain indices.");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, rxGainTableFilePointer);
        }
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            rxGainTablePath,
            "Empty Rx Gain Table encountered");
        /* no need for ADI_ERROR_CLOSE_RETURN here as the file will be closed below anyway */
    }

    /* Close Rx Gain Table csv file */
    if (fclose(rxGainTableFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            rxGainTablePath,
            "Fatal error while trying to close Rx Gain Table csv file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return recoveryAction;
#endif
}

int32_t adi_adrv9001_Utilities_TxAttenTable_Load(adi_adrv9001_Device_t *device, const char *txAttenTablePath, uint32_t txChannelMask)
{
#ifdef __KERNEL__
	adi_hal_Cfg_t *hal = device->common.devHalInfo;
	struct spi_device *spi = hal->spi;
	const struct firmware *fw;
	static const uint8_t NUM_COLUMNS = 3;
	int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
	uint16_t prevAttenIndex = 0;
	uint16_t attenIndex = 0;
	uint16_t minAttenIndex = 0;
	uint16_t maxAttenIndex = ADRV9001_TX_ATTEN_TABLE_MAX;
	uint16_t lineCount = 0;
	uint16_t tableSize = 0;
	static adi_adrv9001_TxAttenTableRow_t txAttenTableRowBuffer[ADI_ADRV9001_TX_ATTEN_TABLE_SIZE_ROWS];
	int ret;
	const char header[] = {"Tx Atten Index,Tx Atten Hp,Tx Atten Mult"};
	char *line, *ptr;

	/* Check device pointer is not null */
	ADI_API_ENTRY_PTR_EXPECT(device, txAttenTablePath);

	ret = request_firmware(&fw, txAttenTablePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev,
			"request_firmware(%s) failed with %d\n", txAttenTablePath, ret);
		return ret;
	}

	ptr = (char *) fw->data;

	while ((line = strsep(&ptr, "\n"))  && (lineCount <  maxAttenIndex)) {
		if (line >= (char *)fw->data + fw->size)
			break;

		line = skip_spaces(line);

		if (sscanf(line,
			"%hu,%hhu,%hu",
			&attenIndex,
			&txAttenTableRowBuffer[lineCount].txAttenHp,
			&txAttenTableRowBuffer[lineCount].txAttenMult) != NUM_COLUMNS) {

			ret = strncmp(line, header , strlen(header));
			if (ret == 0)
				continue;

			ADI_ERROR_REPORT(&device->common,
					 ADI_COMMON_ERRSRC_API,
					ADI_COMMON_ERR_INV_PARAM,
					ADI_COMMON_ACT_ERR_CHECK_PARAM,
					lineCount,
					"Insufficient entries in Tx atten table row entry");
			release_firmware(fw);
			ADI_ERROR_RETURN(device->common.error.newAction);
		}

		if (lineCount == 0) {
			minAttenIndex = attenIndex;
		} else {
			/*Check that atten indices are arranged in ascending order*/
			if (prevAttenIndex != (attenIndex - 1)) {
				ADI_ERROR_REPORT(&device->common,
						 ADI_COMMON_ERRSRC_API,
		     				ADI_COMMON_ERR_INV_PARAM,
						ADI_COMMON_ACT_ERR_CHECK_PARAM,
						attenIndex,
						"Atten indices not arranged in ascending order in Tx Atten Table file");
				release_firmware(fw);
				ADI_ERROR_RETURN(device->common.error.newAction);
			}
		}

		prevAttenIndex = attenIndex;
		lineCount++;
	}

	tableSize = attenIndex - minAttenIndex + 1;

	recoveryAction = adi_adrv9001_Tx_AttenuationTable_Write(device, txChannelMask,
								minAttenIndex, &txAttenTableRowBuffer[0], tableSize);
	if (recoveryAction != ADI_COMMON_ACT_NO_ACTION) {
		ADI_ERROR_REPORT(&device->common,
				 ADI_COMMON_ERRSRC_API,
				ADI_COMMON_ERR_INV_PARAM,
				ADI_COMMON_ACT_ERR_CHECK_PARAM,
				txAttenTablePath,
				"Unable to write Tx Atten Table");
		release_firmware(fw);
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	release_firmware(fw);

	return recoveryAction;
#else
    static const uint8_t NUM_COLUMNS = 3;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint16_t prevAttenIndex = 0;
    uint16_t attenIndex = 0;
    uint16_t minAttenIndex = 0;
    uint16_t maxAttenIndex = 0;
    uint16_t lineCount = 0;
    uint16_t tableSize = 0;
    FILE *txAttenTableFilePointer = NULL;
    char txAttenTableLineBuffer[ADI_ADRV9001_LINE_BUFFER_SIZE];
    char headerStr1[ADI_ADRV9001_HEADER_BUFFER_SIZE] ;
    char headerStr2[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    char headerStr3[ADI_ADRV9001_HEADER_BUFFER_SIZE];
    static adi_adrv9001_TxAttenTableRow_t txAttenTableRowBuffer[ADI_ADRV9001_TX_ATTEN_TABLE_SIZE_ROWS];

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, txAttenTablePath);

    maxAttenIndex = ADRV9001_TX_ATTEN_TABLE_MAX;

    /*Open Tx Atten Table csv file*/
#ifdef __GNUC__
    txAttenTableFilePointer = fopen(txAttenTablePath, "r");

    if (txAttenTableFilePointer == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txAttenTablePath,
            "Invalid Tx Atten Table csv file path encountered while attempting to load Tx Atten Table");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#else
    if (fopen_s(&txAttenTableFilePointer, txAttenTablePath, "r") != 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txAttenTablePath,
            "Unable to open Tx Atten Table csv file. Please check if the path is correct or the file is open in another program");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /*Check for empty Tx Atten Table*/
    if (fgets(txAttenTableLineBuffer, sizeof(txAttenTableLineBuffer), txAttenTableFilePointer) != NULL)
    {
#ifdef __GNUC__
        if (sscanf(txAttenTableLineBuffer, "%[^,],%[^,],%[^\n]", headerStr1, headerStr2, headerStr3) != NUM_COLUMNS)
#else
        if (sscanf_s(txAttenTableLineBuffer,
                "%[^,],%[^,],%[^\n]",
                headerStr1,
                (uint32_t)sizeof(headerStr1),
                headerStr2,
                (uint32_t)sizeof(headerStr2),
                headerStr3,
                (uint32_t)sizeof(headerStr3)) != NUM_COLUMNS)
#endif
        {
            ADI_ERROR_REPORT(&device->common,
                 ADI_COMMON_ERRSRC_API,
                 ADI_COMMON_ERR_INV_PARAM,
                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                 txAttenTablePath,
                 "Invalid Tx Atten Table format encountered while attempting to load Tx Atten Table");
             ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
        }

        /*Verify that Gain Table Format is correct*/
        if (strstr(headerStr1, "Tx Atten Index") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                txAttenTablePath,
                "Expected 'Tx Atten Index' to be the first column in Tx Atten table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
        }

        if (strstr(headerStr2, "Tx Atten Hp") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                txAttenTablePath,
                "Expected 'Tx Atten Hp' to be the second column in Tx Atten table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
        }

        if (strstr(headerStr3, "Tx Atten Mult") == NULL)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                txAttenTablePath,
                "Expected 'Tx Atten Mult' to be the third column in Tx Atten table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
        }

        /*Loop until the atten table end is reached or no. of lines scanned exceeds maximum*/
        while ((fgets(txAttenTableLineBuffer, sizeof(txAttenTableLineBuffer), txAttenTableFilePointer) != NULL) &&
               (lineCount < maxAttenIndex))
        {
#ifdef __GNUC__
            if (sscanf(txAttenTableLineBuffer,
                "%hu,%hhu,%hu",
                &attenIndex,
                &txAttenTableRowBuffer[lineCount].txAttenHp,
                &txAttenTableRowBuffer[lineCount].txAttenMult) != NUM_COLUMNS)
#else
            if (sscanf_s(txAttenTableLineBuffer,
                    "%hu,%hhu,%hu",
                    &attenIndex,
                    &txAttenTableRowBuffer[lineCount].txAttenHp,
                    &txAttenTableRowBuffer[lineCount].txAttenMult) != NUM_COLUMNS)
#endif
            {
                ADI_ERROR_REPORT(&device->common,
                    ADI_COMMON_ERRSRC_API,
                    ADI_COMMON_ERR_INV_PARAM,
                    ADI_COMMON_ACT_ERR_CHECK_PARAM,
                    lineCount,
                    "Insufficient entries in Tx atten table row entry");
                    ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
            }

            if (lineCount == 0)
            {
                minAttenIndex = attenIndex;
            }
            else
            {
                /*Check that atten indices are arranged in ascending order*/
                if (prevAttenIndex != (attenIndex - 1))
                {
                    ADI_ERROR_REPORT(&device->common,
                        ADI_COMMON_ERRSRC_API,
                        ADI_COMMON_ERR_INV_PARAM,
                        ADI_COMMON_ACT_ERR_CHECK_PARAM,
                        attenIndex,
                        "Atten indices not arranged in ascending order in Tx Atten Table file");
                    ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
                }
            }

            prevAttenIndex = attenIndex;
            lineCount++;
        }

        tableSize = attenIndex - minAttenIndex + 1;

        recoveryAction = adi_adrv9001_Tx_AttenuationTable_Write(device, txChannelMask, minAttenIndex, &txAttenTableRowBuffer[0], tableSize);
        if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                txAttenTablePath,
                "Unable to write Tx Atten Table");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, txAttenTableFilePointer);
        }

    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txAttenTablePath,
            "Empty Tx Atten Table encountered");
        /* no need for ADI_ERROR_CLOSE_RETURN here as the file will be closed below anyway */
    }

    /* Close Tx Atten Table csv file */
    if (fclose(txAttenTableFilePointer) < 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            txAttenTablePath,
            "Fatal error while trying to close Tx Atten Table csv file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return recoveryAction;
#endif
}

int32_t adi_adrv9001_Utilities_ArmMemory_Dump(adi_adrv9001_Device_t *device, const char *binaryFilename)
{
#ifdef __KERNEL__ /* FIXME: Later */
	BUG();
	return 0;
#else
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    FILE *ofp;
    uint32_t byteCount = 0;
    uint32_t offset = 0;
    uint8_t armMailboxBusy = 0;
    uint8_t cmdStatusByte = 0;
    uint8_t exceptionArray[4] = { 0 };
    uint8_t extArray[4] = { 0 };
    uint32_t exceptionValue = 0;
    uint8_t armException = 0;

    uint8_t binaryRead[ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE + 10] = { 0 };

    static const uint32_t armExceptionAddr = 0x20000214; /* Exception Flag Memory */

    ADI_API_ENTRY_PTR_EXPECT(device, binaryFilename);

#ifdef ADRV9001_INIT_DEBUG
    printf("open %s (%s) \n", binaryFilename, "wb");
#endif

#ifdef __GNUC__
    ofp = fopen(binaryFilename, "wb");
#else
    fopen_s(&ofp, binaryFilename, "wb");
#endif

    if (ofp == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            ofp,
            "Unable to open binary image file. Please check if the path is correct");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check if exception has occurred */
    recoveryAction = adi_adrv9001_arm_Memory_Read(device, armExceptionAddr, &exceptionArray[0], 4, 1);
    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        ADI_COMMON_ERR_API_FAIL,
        recoveryAction,
        NULL,
        "Unable to Read ARM memory");
    ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

    exceptionValue = (uint32_t)(exceptionArray[0] | (exceptionArray[1] << 8) | (exceptionArray[2] << 16) | (exceptionArray[3] << 24));

    if (exceptionValue == 0)
    {
        /* Force an exception during ARM MEM dump for more useful information        */

        recoveryAction = adi_adrv9001_arm_MailboxBusy_Get(device, &armMailboxBusy);
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Invalid Get for armMailboxBusy");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

        if (armMailboxBusy == false)
        {
            extArray[0] = ADRV9001_ARM_OBJECTID_ARM_EXCEPTION;
            recoveryAction = adi_adrv9001_arm_Cmd_Write(device, ADRV9001_ARM_SET_OPCODE, &extArray[0], 1);
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_API_FAIL,
                recoveryAction,
                NULL,
                "Unable to Process ARM command");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

            recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(device,
                                                             NULL,
                                                             ADRV9001_ARM_SET_OPCODE,
                                                             &cmdStatusByte,
                                                             ADI_ADRV9001_WRITEARMEXCEPTION_TIMEOUT_US,
                                                             ADI_ADRV9001_WRITEARMEXCEPTION_INTERVAL_US);
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             recoveryAction,
                             NULL,
                             "Unable to Proccess ARM command");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

            recoveryAction = adi_adrv9001_arm_Memory_Read(device, armExceptionAddr, &exceptionArray[0], 4, 1);
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             recoveryAction,
                             NULL,
                             "Unable to Read ARM memory");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

            exceptionValue = (uint32_t)(exceptionArray[0] | (exceptionArray[1] << 8) | (exceptionArray[2] << 16) | (exceptionArray[3] << 24));
            if (exceptionValue > 0)
            {
                armException = 1;
            }
        }
    }

    /* Program Binary */
    for (offset = ADRV9001_ADDR_ARM_START_PROG; offset < ADRV9001_ADDR_ARM_END_PROG; offset += ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE)
    {
        if (offset < (ADRV9001_ADDR_ARM_END_PROG - ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE))
        {
            byteCount = ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE;
        }
        else
        {
            byteCount = ADRV9001_ADDR_ARM_END_PROG + 1 - offset;
        }

        recoveryAction = adi_adrv9001_arm_Memory_Read(device, offset, binaryRead, byteCount, 1);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            recoveryAction,
            NULL,
            "Unable to Read ARM memory");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

        if (fwrite(binaryRead, 1, byteCount, ofp) != byteCount)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                ofp,
                "Fatal error while trying to write a binary file.");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);
        }
    }

    /* Data Binary */
    for (offset = ADRV9001_ADDR_ARM_START_DATA; offset < ADRV9001_ADDR_ARM_END_DATA; offset += ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE)
    {
        if (offset < (ADRV9001_ADDR_ARM_END_DATA - ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE))
        {
            byteCount = ADI_ADRV9001_MEM_DUMP_CHUNK_SIZE;
        }
        else
        {
            byteCount = ADRV9001_ADDR_ARM_END_DATA + 1 - offset;
        }

        recoveryAction = adi_adrv9001_arm_Memory_Read(device, offset, binaryRead, byteCount, 1);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            recoveryAction,
            NULL,
            "Unable to Read ARM memory");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);

        if (fwrite(binaryRead, 1, byteCount, ofp) != byteCount)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                ofp,
                "Fatal error while trying to write a binary file.");
            ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, ofp);
        }
    }

    if (armException > 0)
    {
        /*if we forced an exception, clear the exception so the ARM will continue to run*/
        exceptionArray[0] = 0;
        exceptionArray[1] = 0;
        exceptionArray[2] = 0;
        exceptionArray[3] = 0;

        recoveryAction = adi_adrv9001_arm_Memory_Write(device, armExceptionAddr, &exceptionArray[0], 4);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            recoveryAction,
            NULL,
            "Unable to Write ARM memory");
        /* no need for ADI_ERROR_CLOSE_RETURN here as the file will be closed below anyway */
    }

    /*Close ARM binary file*/
    if (fclose(ofp) < 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            ofp,
            "Fatal error while trying to close a binary file. Possible memory shortage while flushing / other I/O errors.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
#endif
}

int32_t adi_adrv9001_InitRadioLoad(adi_adrv9001_Device_t *device, 
                                   int32_t(*Mcs_Requested)(void),
                                   adi_adrv9001_ResourceCfg_t *initConfig,
                                   adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings,
                                   uint8_t channelMask)
{
    uint8_t allChannels = (uint8_t)(ADI_CHANNEL_1 | ADI_CHANNEL_2);
    adi_adrv9001_ArmClock_t armClock = { 0x0F, true, ADI_ADRV9001_ARMCLOCK_DATAPATH };

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, initConfig);
    ADI_API_ENTRY_PTR_EXPECT(device, initConfig->adrv9001RadioConfig);
    ADI_API_ENTRY_PTR_EXPECT(device, initConfig->adrv9001PlatformFiles);

    ADI_EXPECT(adi_adrv9001_arm_Disable, device);

    ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, device);
    ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, device, 2);  

    ADI_EXPECT(adi_adrv9001_arm_Enable, device);

    ADI_EXPECT(adi_adrv9001_arm_Clocks_Program, device, &armClock);

    if (device->devStateInfo.profilesValid & ADI_ADRV9001_RX_PROFILE_VALID)
    {
        /* Load Rx gain table for requested channels */
        ADI_EXPECT(adi_adrv9001_Utilities_RxGainTable_Load,
                       device,
                       (const char*)initConfig->adrv9001PlatformFiles->rxGainTableFile,
                       (allChannels & device->devStateInfo.initializedChannels));
    }

    if (device->devStateInfo.profilesValid & ADI_ADRV9001_TX_PROFILE_VALID)
    {
        /* Load Tx atten tables from a .csv file for requested channels */
        ADI_EXPECT(adi_adrv9001_Utilities_TxAttenTable_Load,
                       device,
                       (const char*)initConfig->adrv9001PlatformFiles->txAttenTableFile,
                       (allChannels & (device->devStateInfo.initializedChannels >> ADI_ADRV9001_TX_INITIALIZED_CH_OFFSET)));
    }

    armClock.clockEnable = false;
    ADI_EXPECT(adi_adrv9001_arm_Clocks_Program, device, &armClock);

    /* LVDS forced mode */
    ADI_MSG_EXPECT("Error programming SSI delay configuration", adi_adrv9001_Utilities_SsiDelay_Configure, device, ssiConfigSettings);

    ADI_EXPECT(adi_adrv9001_arm_Disable, device);

    ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, device);
    ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, device, 2);  

    ADI_EXPECT(adi_adrv9001_arm_Enable, device);

#if ADI_ADRV9001_FRONT_END_GPIO_DOWNLOAD > 0
    ADI_EXPECT(adi_adrv9001_FrontEndGpioConfigSet,
        device,
        initConfig->adrv9001RadioConfig->radioCtrlInit.armGpioSigMap,
        initConfig->adrv9001RadioConfig->radioCtrlInit.armGpioSigMapSize);
#endif

    /*Initialize radio control. This is required to run before running init cals*/
    ADI_EXPECT(adrv9001_RadioCtrlInit, device, &initConfig->adrv9001RadioConfig->radioCtrlInit, channelMask);

    /* Send SYSTEM_CONFIG mailbox command to ARM */
    ADI_EXPECT(adi_adrv9001_arm_System_Program, device, Mcs_Requested, channelMask);

    ADI_API_RETURN(device);
}

/* TODO: Move all LVDS related APIs to a new file when LVDS calibration is implemented */
static uint32_t adi_adrv9001_RxLvdsDelayConfigSet(adi_adrv9001_Device_t *adrv9001Device,
                                                  adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg,
                                                  uint8_t channelCount)
{
    uint8_t lvdsDelay = 0;

    uint16_t rxLvdsDelayAddr[ADI_ADRV9001_MAX_RX_ONLY][4] =
    {
        {
            ADRV9001_ADDR_RX1_CLK_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX1_STROBE_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX1_IDATA_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX1_QDATA_LVDS_MODE_DELAY
        },
        {
            ADRV9001_ADDR_RX2_CLK_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX2_STROBE_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX2_IDATA_LVDS_MODE_DELAY,
            ADRV9001_ADDR_RX2_QDATA_LVDS_MODE_DELAY
        }
    };

    /* CLK LVDS delay for RX1/RX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_CLK_LVDS_DELAY", rxLvdsDelayAddr[channelCount][0], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsRxClkDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "RX_CLK_LVDS_DELAY", rxLvdsDelayAddr[channelCount][0], lvdsDelay);
    lvdsDelay = 0;

    /* Strobe LVDS delay for RX1/RX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_STROBE_LVDS_DELAY", rxLvdsDelayAddr[channelCount][1], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsRxStrobeDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "RX_STROBE_LVDS_DELAY", rxLvdsDelayAddr[channelCount][1], lvdsDelay);
    lvdsDelay = 0;

    /* I data LVDS delay for RX1/RX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_IDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][2], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsRxIDataDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "RX_IDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][2], lvdsDelay);
    lvdsDelay = 0;

    /* Q data LVDS delay for RX1/RX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_QDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][3], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsRxQDataDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "RX_QDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][3], lvdsDelay);
    lvdsDelay = 0;

    ADI_API_RETURN(adrv9001Device);
}

static uint32_t adi_adrv9001_TxLvdsDelayConfigSet(adi_adrv9001_Device_t *adrv9001Device,
                                                  adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg,
                                                  uint8_t channelCount)
{
    uint8_t lvdsDelay = 0;

    uint16_t txLvdsDelayAddr[ADI_ADRV9001_MAX_RX_ONLY][4] =
    {
    {
        ADRV9001_ADDR_TX1_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_QDATA_LVDS_MODE_DELAY
    },
    {
        ADRV9001_ADDR_TX2_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_QDATA_LVDS_MODE_DELAY
    }
    };

    /* CLK LVDS delay for TX1/TX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_CLK_LVDS_DELAY", txLvdsDelayAddr[channelCount][0], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsTxClkDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "TX_CLK_LVDS_DELAY", txLvdsDelayAddr[channelCount][0], lvdsDelay);
    lvdsDelay = 0;

    /* Strobe LVDS delay for TX1/TX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_STROBE_LVDS_DELAY", txLvdsDelayAddr[channelCount][1], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsTxStrobeDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "TX_STROBE_LVDS_DELAY", txLvdsDelayAddr[channelCount][1], lvdsDelay);
    lvdsDelay = 0;

    /* I data LVDS delay for TX1/TX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_IDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][2], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsTxIDataDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "TX_IDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][2], lvdsDelay);
    lvdsDelay = 0;

    /* Q data LVDS delay for TX1/TX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_QDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][3], &lvdsDelay);

    lvdsDelay = lvdsDelay | ((lvdsSsiCalibrationCfg->lvdsTxQDataDelay[channelCount] & 0x07) << 1);
    ADRV9001_SPIWRITEBYTE(adrv9001Device, "TX_QDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][3], lvdsDelay);
    lvdsDelay = 0;

    ADI_API_RETURN(adrv9001Device);
}

static uint32_t adi_adrv9001_LvdsDelayConfigSet(adi_adrv9001_Device_t *adrv9001Device,
                                                adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg)
{
    uint8_t i = 0;

    for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
    {
        ADI_EXPECT(adi_adrv9001_RxLvdsDelayConfigSet, adrv9001Device, lvdsSsiCalibrationCfg, i);
    }

    for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
    {
        ADI_EXPECT(adi_adrv9001_TxLvdsDelayConfigSet, adrv9001Device, lvdsSsiCalibrationCfg, i);
    }

    ADI_API_RETURN(adrv9001Device);
}

int32_t adi_adrv9001_Utilities_SsiDelay_Configure(adi_adrv9001_Device_t *adrv9001Device,
                                       adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings)
{
    ADI_API_ENTRY_PTR_EXPECT(adrv9001Device, ssiConfigSettings);

    ADI_RANGE_CHECK(adrv9001Device, ssiConfigSettings->ssiMode, ADI_ADRV9001_CMOS_1LANE_16I16Q, ADI_ADRV9001_LVDS_2LANE_16I16Q);

    /* TODO: Range check to be done for all delay values */

    /* LVDS delay configuration */
    if (ssiConfigSettings->ssiMode == ADI_ADRV9001_LVDS_2LANE_16I16Q)
    {
        ADI_EXPECT(adi_adrv9001_LvdsDelayConfigSet, adrv9001Device, &(ssiConfigSettings->ssiCalibrationCfg.lvdsSsiCalibrationCfg));
    }

    /* TODO: Delay configuration for other LVDS modes and CMOS in future */

    ADI_API_RETURN(adrv9001Device);
}

static uint32_t adi_adrv9001_RxLvdsDelayConfigGet(adi_adrv9001_Device_t *adrv9001Device,
                                                  adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg,
                                                  uint8_t channelCount)
{
    uint8_t lvdsDelay = 0;

    uint16_t rxLvdsDelayAddr[ADI_ADRV9001_MAX_RX_ONLY][4] =
    {
    {
        ADRV9001_ADDR_RX1_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX1_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX1_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX1_QDATA_LVDS_MODE_DELAY
    },
    {
        ADRV9001_ADDR_RX2_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX2_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX2_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_RX2_QDATA_LVDS_MODE_DELAY
    }
    };

    /* CLK LVDS delay for RX1/RX2*/
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_CLK_LVDS_DELAY", rxLvdsDelayAddr[channelCount][0], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsRxClkDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* Strobe LVDS delay for RX1/RX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_STROBE_LVDS_DELAY", rxLvdsDelayAddr[channelCount][1], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsRxStrobeDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* I data LVDS delay for RX1/RX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_IDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][2], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsRxIDataDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* Q data LVDS delay for RX1/RX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "RX_QDATA_LVDS_DELAY", rxLvdsDelayAddr[channelCount][3], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsRxQDataDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    ADI_API_RETURN(adrv9001Device);
}

static uint32_t adi_adrv9001_TxLvdsDelayConfigGet(adi_adrv9001_Device_t *adrv9001Device,
                                                  adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg,
                                                  uint8_t channelCount)
{
    uint8_t lvdsDelay = 0;

    uint16_t txLvdsDelayAddr[ADI_ADRV9001_MAX_RX_ONLY][4] =
    {
    {
        ADRV9001_ADDR_TX1_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX1_QDATA_LVDS_MODE_DELAY
    },
    {
        ADRV9001_ADDR_TX2_CLK_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_STROBE_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_IDATA_LVDS_MODE_DELAY,
        ADRV9001_ADDR_TX2_QDATA_LVDS_MODE_DELAY
    }
    };

    /* CLK LVDS delay for TX1/TX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_CLK_LVDS_DELAY", txLvdsDelayAddr[channelCount][0], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsTxClkDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* Strobe LVDS delay for TX1/TX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_STROBE_LVDS_DELAY", txLvdsDelayAddr[channelCount][1], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsTxStrobeDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* I data LVDS delay for TX1/TX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_IDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][2], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsTxIDataDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    /* Q data LVDS delay for TX1/TX2*/
    lvdsDelay = 0;
    ADRV9001_SPIREADBYTE(adrv9001Device, "TX_QDATA_LVDS_DELAY", txLvdsDelayAddr[channelCount][3], &lvdsDelay);
    lvdsSsiCalibrationCfg->lvdsTxQDataDelay[channelCount] = (lvdsDelay >> 1) & 0x07;

    ADI_API_RETURN(adrv9001Device);
}

static uint32_t adi_adrv9001_LvdsDelayConfigGet(adi_adrv9001_Device_t *adrv9001Device,
                                                adi_adrv9001_LvdsSsiCalibrationCfg_t *lvdsSsiCalibrationCfg)
{
    uint8_t i = 0;

    for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
    {
        ADI_EXPECT(adi_adrv9001_RxLvdsDelayConfigGet, adrv9001Device, lvdsSsiCalibrationCfg, i);
    }

    for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
    {
        ADI_EXPECT(adi_adrv9001_TxLvdsDelayConfigGet, adrv9001Device, lvdsSsiCalibrationCfg, i);
    }

    ADI_API_RETURN(adrv9001Device);
}

int32_t adi_adrv9001_Utilities_SsiDelay_Inspect(adi_adrv9001_Device_t *adrv9001Device,
                                       adi_adrv9001_SsiConfigSettings_t *ssiConfigSettings)
{
    ADI_API_ENTRY_PTR_EXPECT(adrv9001Device, ssiConfigSettings);

    ADI_RANGE_CHECK(adrv9001Device, ssiConfigSettings->ssiMode, ADI_ADRV9001_CMOS_1LANE_16I16Q, ADI_ADRV9001_LVDS_2LANE_16I16Q);

    /* TODO: Range check to be done for all delay values */

    /* LVDS delay configuration */
    if (ssiConfigSettings->ssiMode == ADI_ADRV9001_LVDS_2LANE_16I16Q)
    {
        ADI_EXPECT(adi_adrv9001_LvdsDelayConfigGet, adrv9001Device, &(ssiConfigSettings->ssiCalibrationCfg.lvdsSsiCalibrationCfg));
    }

    /* TODO: Delay configuration for other LVDS modes and CMOS in future */

    ADI_API_RETURN(adrv9001Device);
}
