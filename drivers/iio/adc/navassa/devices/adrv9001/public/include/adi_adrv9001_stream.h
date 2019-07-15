/**
 * \file
 * \brief Contains ADRV9001 related function prototypes for adi_adrv9001_stream.c
 *
 *  ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_STREAM_H_
#define _ADI_ADRV9001_STREAM_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_stream_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Loads binary array into stream processor data memory
 *
 * FIXME PTN Navassa may not be 20K, check when into available
 *
 * A 20K element byte array is passed into this function.  The byte array is
 * obtained by reading the binary stream processor file provided by Analog
 * Devices.  The stream processor uses the information in the stream file to
 * properly power up and down the various signal chains.
 *
 * \pre This function is called after adi_adrv9001_Initialize, and before ARM is loaded.
 *
 * \param device     Pointer to the ADRV9001 device data structure containing settings
 * \param byteOffset Offset (starting from 0) of where to place the binary
 *                   array (if loaded in multiple function calls)
 * \param binary     Byte array containing all valid ARM file data bytes
 * \param byteCount  The number of bytes in the binary array file
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Stream_Image_Write(adi_adrv9001_Device_t *device, uint32_t byteOffset, uint8_t binary[], uint32_t byteCount);


#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_STREAM_H_ */
