/**
* \file
* \brief Contains Utility features related private function implementations
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adrv9001_utilities.h"

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#include "adi_adrv9001_error.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001.h"
#include "adrv9001_reg_addr_macros.h"

#ifdef __KERNEL__
int32_t adrv9001_SafeFileLoad(adi_adrv9001_Device_t *device, const char *filename, char **buffer, uint32_t *filelength)
{
	/* FIXME MH */
	BUG();
	ADI_API_RETURN(device);
}
#else
int32_t adrv9001_SafeFileLoad(adi_adrv9001_Device_t *device, const char *filename, char **buffer, uint32_t *filelength) 
{
    FILE * fp = NULL;
    
    /* Try to open the file. */
    fp = fopen(filename, "rb");

    /* Check that the file was opened. */
    if (fp == NULL)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            filename,
            "Invalid file name or path encountered.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    
    /* Check that we can move the file pointer from beginning to end. */
    if (fseek(fp, 0, SEEK_END) < 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            filename,
            "Unable to move file descriptor to the end of the file.");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, fp);
    }
    
    *filelength = ftell(fp);
    
    /* Check that the file is non-empty. */
    if (*filelength <= 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            filename,
            "Empty file encountered.");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, fp);
    }
    
    /* Rewind the file pointer to beginning of the file. */
    if (fseek(fp, 0, SEEK_SET) < 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            filename,
            "Unable to move file descriptor to the beginning of the file.");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, fp);
    }

    /* Allocate space in the buffer. */
    *buffer = (char *)calloc(*filelength, sizeof(char*));
    
    /* Check that buffer memory allocation was successful. */
    if (NULL == *buffer)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_MEM_ALLOC_FAIL,
            ADI_COMMON_ACT_ERR_RESET_FULL,
            NULL,
            "Fatal error while reading file. Possible memory shortage.");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, fp);
    }
    
    /* Read the file into the buffer. */
    if (fread(*buffer, 1, *filelength, fp) < *filelength)
    {
        free(*buffer);
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_RESET_FULL,
            NULL,
            "Fatal error while reading file. The file may be corrupt, or there may be a problem with memory.");
        ADI_ERROR_CLOSE_RETURN(device->common.error.newAction, fp);
    }
    
    fclose(fp);
    
    ADI_API_RETURN(device);
}
#endif

int32_t adrv9001_RadioCtrlInit(adi_adrv9001_Device_t *device, adi_adrv9001_RadioCtrlInit_t *radioCtrlInit, uint8_t channelMask)
{
    static const uint8_t TX_CHANNEL_MASK_OFFSET = 4;
    uint8_t i = 0;
    uint8_t rxChannelMask = (channelMask & 0x03);
    uint8_t txChannelMask = ((channelMask >> TX_CHANNEL_MASK_OFFSET) & 0x03);

    static const adi_common_ChannelNumber_e rxChannelArr[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };

    static const adi_common_ChannelNumber_e txChannelArr[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };
    
    static adi_adrv9001_PllLoopFilterCfg_t defaultLoopFilterConfig = {
        .effectiveLoopBandwidth_kHz = 0,
        .loopBandwidth_kHz = 300,
        .phaseMargin_degrees = 60,
        .powerScale = 5
    };

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, radioCtrlInit);

    /* Set Loop filter configuration for PLL LO1 */
    ADI_EXPECT(adi_adrv9001_Radio_PllLoopFilter_Set, device, ADI_ADRV9001_LO1_PLL, &defaultLoopFilterConfig);

    /* Set Loop filter configuration for PLL LO2 */
    ADI_EXPECT(adi_adrv9001_Radio_PllLoopFilter_Set, device, ADI_ADRV9001_LO2_PLL, &defaultLoopFilterConfig); 

    /* Set Loop filter configuration for AUX PLL */
    ADI_EXPECT(adi_adrv9001_Radio_PllLoopFilter_Set, device, ADI_ADRV9001_AUX_PLL, &defaultLoopFilterConfig); 

    for (i = 0; i < ADI_ARRAY_LEN(rxChannelArr) ; i++)
    {
        if ((rxChannelMask >> i) & 0x01)
        {
            ADI_EXPECT(adi_adrv9001_Radio_CarrierFrequency_Set,
                           device, 
                           ADI_RX,
                           rxChannelArr[i],
                           ADI_ADRV9001_PLL_CAL_MODE_NORM,
                           radioCtrlInit->mbSetState,
                           radioCtrlInit->rxCarrierFreq_Hz[i]);
        }
    }

    for (i = 0; i < ADI_ARRAY_LEN(txChannelArr); i++)
    {
        if ((txChannelMask >> i) & 0x01)
        {
            ADI_EXPECT(adi_adrv9001_Radio_CarrierFrequency_Set,
                           device, 
                           ADI_TX,
                           txChannelArr[i],
                           ADI_ADRV9001_PLL_CAL_MODE_NORM,
                           radioCtrlInit->mbSetState,
                           radioCtrlInit->txCarrierFreq_Hz[i]);
        
#if ADI_ADRV9001_SLEWRATE_CONFIG
            ADI_EXPECT(adi_adrv9001_Tx_SlewRateLimiter_Configure, device, txChannelArr[i], &radioCtrlInit->slewRateLimiterCfg);  
#endif
        }
    }

    ADI_API_RETURN(device);
}
