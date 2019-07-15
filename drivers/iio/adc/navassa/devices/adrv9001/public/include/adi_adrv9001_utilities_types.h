/*!
* \file
* \brief Contains ADRV9001 API utility initialization type definitions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_UTILITIES_TYPES_H_
#define _ADI_ADRV9001_UTILITIES_TYPES_H_

#include <stdint.h>
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_rx_types.h"
//#include "adi_adrv9001_data_interface_types.h"
#include "adi_common_error_types.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_cals_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Data structure to hold Radio Ctrl Utility Init structures
*/
typedef struct adi_adrv9001_RadioCtrlInit
{
    adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockDivisor; /*!< ADRV9001 device clock output divisor */
    uint32_t armGpioSigMapSize;
    adi_adrv9001_ArmGpioSignalMap_t    armGpioSigMap[8];   /*!< Configuration to hold GPIO mappings for pin inputs to ADRV9001 */
    adi_adrv9001_SlewRateLimiterCfg_t  slewRateLimiterCfg; /*!< Configuration to hold Slew Rate Limiter on init */
    adi_adrv9001_MBSetInPrimedState_e mbSetState;          /*!< Configuration to hold Mail Box command SET state for setting carrier frequency */

    uint64_t rxCarrierFreq_Hz[ADI_ADRV9001_MAX_RX_ONLY];    /*!< Configuration to hold Rx1/Rx2 channel carrier frequency on init */
    uint64_t txCarrierFreq_Hz[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Configuration to hold Tx1/Tx2 channel carrier frequency on init */

    uint8_t  gainIndex[ADI_ADRV9001_MAX_RX_ONLY];          /*!< Gain Index for the channels selected in rxChannelMask */
    
    adi_adrv9001_TxAttenuationConfig_t txAttenConfig;       /*!< "Initial" Tx attenuation settings; JS objects to this existing */
} adi_adrv9001_RadioCtrlInit_t;

/**
* \brief Data structure to hold Utility Init structures
*/
typedef struct adi_adrv9001_RadioConfig
{
    adi_adrv9001_RadioCtrlInit_t radioCtrlInit;            /*!< RadioCtrl Initialization configuration */
    adi_adrv9001_InitCals_t initCals;                      /*!< Cals Initialization configuration */
} adi_adrv9001_RadioConfig_t;

/**
* \brief Data structure to hold platform file addresses on the SD card
*/
typedef struct adi_adrv9001_PlatformFiles
{
    uint8_t armImageFile[128];
    uint8_t streamImageFile[128];
    uint8_t rxGainTableFile[128];
    uint8_t txAttenTableFile[128];
} adi_adrv9001_PlatformFiles_t;

/**
* \brief Data Structure to hold stream settings
*/
typedef struct adi_adrv9001_StreamSettings
{
    uint8_t rxM; /*!< If link sharing, M value for Rx channels */
    uint8_t rxS; /*!< If link sharing, S value for Rx channels */
    uint8_t orxM; /*!< If link sharing, M value for ORx channels */
    uint8_t orxS; /*!< If link sharing, S value for ORx channels */
//  adi_adrv9001_AdcSampleXbarCfg_t rxSampleXBar;
//  adi_adrv9001_AdcSampleXbarCfg_t orxSampleXBar;
    uint8_t linkSharing; /*!<1 if link sharing. 0 otherwise */
} adi_adrv9001_StreamSettings_t;

    
/**
* \brief Data Structure to hold Resource Config settings
*/
typedef struct adi_adrv9001_ResourceCfg
{
    adi_adrv9001_Init_t *adrv9001Init;
    adi_adrv9001_RadioConfig_t *adrv9001RadioConfig;
    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles;
} adi_adrv9001_ResourceCfg_t;

/**
* \brief Data Structure to hold SSI configuration settings
*/
typedef struct adi_adrv9001_SsiConfigSettings
{
    adi_adrv9001_SsiMode_e ssiMode;
    adi_adrv9001_SsiCalibrationCfg_t ssiCalibrationCfg;
} adi_adrv9001_SsiConfigSettings_t;
        
#ifdef __cplusplus
}
#endif

#endif