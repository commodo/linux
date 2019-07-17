/**
 * \file
 * \brief Contains ADRV9001 calibration related function prototypes for
 *        adi_adrv9001_cals.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_CALS_H_
#define _ADI_ADRV9001_CALS_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/**
 * \brief Runs the ADRV9001 initialization calibrations
 *
 * \pre This function is called after the device has been initialized, and the RF PLL has been
 * verified to be locked
 *
 *  calMask Bit | Calibration
 *  ------------|----------------------
 *       0      | TX QEC Init Cal 
 *       1      | TX LOL Init Cal 
 *       2      | TX Loopback path delay Init Cal 
 *       3      | TX LO Delay Init Cal 
 *       4      | TX BBAF Init Cal 
 *       5      | TX BBAF Gain Delay Init Cal 
 *       6      | TX Attenuation Delay Init Cal 
 *       7      | TX DAC Init Cal 
 *       8      | TX Path Delay Init Cal 
 *       9      | RX HP ADC Init Cal 
 *       10     | RX HP ADC Flash Init Cal 
 *       11     | RX HP ADC DAC Init Cal 
 *       12     | RX HP ADC Stability Init Cal 
 *       13     | RX LP ADC Init Cal 
 *       14     | RX TIA Cutoff Init Cal 
 *       15     | RX TIA Fine Init Cal 
 *       16     | RX T - Cal Init Cal 
 *       17     | RX FIIC Init Cal 
 *       18     | RX Internal Loopback LO Delay Init Cal 
 *       19     | RX RFDC Init Cal 
 *       20     | RX Gain Path Delay Init Cal 
 *       21     | PLL Init Cal 
 *       22     | AUX PLL Init Cal
 *      [23]    | Reserved

 * \param device            Pointer to the device settings structure
 * \param Mcs_Requested     Callback function to execute while waiting for init cals to run
 * \param initCals          Pointer to the InitCals structure which calibrations to run
 * \param timeout_ms        A timeout value in milliseconds to wait for the calibrations to complete
 * \param errorFlag         A 3-bit error flag that helps identify any errors during Initial calibrations.
 *                          '0' indicates that there was no error.
 *
 * \retval ADI_ADI_COMMON_ACT_WARN_RESET_LOG      Recovery action for log reset
 * \retval ADI_ADI_COMMON_ACT_ERR_CHECK_PARAM     Recovery action for bad parameter check
 * \retval ADI_ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required
 * \retval ADI_ADI_COMMON_ACT_ERR_CHECK_TIMER     Recovery action for timer time-out check required
 * \retval ADI_ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
 */
int32_t adi_adrv9001_InitCalsRun(adi_adrv9001_Device_t *device, 
                                 int32_t(*Mcs_Requested)(void),
                                 adi_adrv9001_InitCals_t *initCals,
                                 uint32_t timeout_ms,
                                 uint8_t *errorFlag);
    
/**
 * \brief Builds a default instance of adi_adrv9001_InitCals_t with all Init Cals turned ON
 *
 * \param initCals Pointer to an instance of adi_adrv9001_InitCals_t which will be populated with default values
 *
 * \retval ADI_COMMON_ACT_NO_ACTION           Function completed successfully, no action required
 */
int32_t adi_adrv9001_InitCalsBuildDefault(adi_adrv9001_InitCals_t *initCals);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_CALS_H_ */

