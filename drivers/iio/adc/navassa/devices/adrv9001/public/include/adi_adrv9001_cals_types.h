/**
 * \file
 * \brief Contains ADRV9001 API Calibration data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_CALS_TYPES_H_
#define _ADI_ADRV9001_CALS_TYPES_H_

#define ARMINITCAL_ERRCODE(armCalId, armCalErrCode) ((armCalId << 8) | armCalErrCode)

#ifdef __cplusplus
extern "C" {
#endif
    
/**
 *  \brief Enum to select desired InitCals select bits in the initCalMask.
 */	
typedef enum adi_adrv9001_InitCalibrations
{
    ADI_ADRV9001_TX_QEC              = 0x00000001, /*!< TX QEC Init Cal */
    ADI_ADRV9001_TX_LOL              = 0x00000002, /*!< TX LOL Init Cal */
    ADI_ADRV9001_TX_LBPD             = 0x00000004, /*!< TX Loopback path delay Init Cal */
    ADI_ADRV9001_TX_LOD              = 0x00000008, /*!< TX LO Delay Init Cal */
    ADI_ADRV9001_TX_BBAF             = 0x00000010, /*!< TX BBAF Init Cal */
    ADI_ADRV9001_TX_BBAF_GD          = 0x00000020, /*!< TX BBAF Gain Delay Init Cal */
    ADI_ADRV9001_TX_ATTD             = 0x00000040, /*!< TX Attenuation Delay Init Cal */
    ADI_ADRV9001_TX_DAC              = 0x00000080, /*!< TX DAC Init Cal */
    ADI_ADRV9001_TX_PD               = 0x00000100, /*!< TX Path Delay Init Cal */
    ADI_ADRV9001_RX_HPADC_RC         = 0x00000200, /*!< RX HP ADC Init Cal */
    ADI_ADRV9001_RX_HPADC_FLASH      = 0x00000400, /*!< RX HP ADC Flash Init Cal */
    ADI_ADRV9001_RX_HPADC_DAC        = 0x00000800, /*!< RX HP ADC DAC Init Cal */
    ADI_ADRV9001_RX_HPADC_STABILITY  = 0x00001000, /*!< RX HP ADC Stability Init Cal */
    ADI_ADRV9001_RX_LPADC            = 0x00002000, /*!< RX LP ADC Init Cal */
    ADI_ADRV9001_RX_TIA_CUTOFF       = 0x00004000, /*!< RX TIA Cutoff Init Cal */
    ADI_ADRV9001_RX_TIA_FINE         = 0x00008000, /*!< RX TIA Fine Init Cal */
    ADI_ADRV9001_RX_TCAL             = 0x00010000, /*!< RX T-Cal Init Cal */
    ADI_ADRV9001_RX_FIIC             = 0x00020000, /*!< RX FIIC Init Cal */
    ADI_ADRV9001_RX_ILB_LOD          = 0x00040000, /*!< RX Internal Loopback LO Delay Init Cal */
    ADI_ADRV9001_RX_RFDC_OFFSET      = 0x00080000, /*!< RX RFDC Init Cal */
    ADI_ADRV9001_RX_GPD              = 0x00100000, /*!< RX Gain Path Delay Init Cal */
    ADI_ADRV9001_PLL                 = 0x00200000, /*!< PLL Init Cal */
    ADI_ADRV9001_AUXPLL              = 0x00400000, /*!< AUX PLL Init Cal */
    
    ADI_ADRV9001_TX_ALL              = 0x000001FF, /*!< TX all Init Cals */
    ADI_ADRV9001_RX_ALL              = 0x001FFE00, /*!< RX all Init Cals */
    ADI_ADRV9001_RX_TX_ALL           = 0x001FFFFF, /*!< RX / TX all Init Cals*/
    ADI_ADRV9001_SYSTEM_ALL          = 0x00600000, /*!< All system Init Cals */
}adi_adrv9001_InitCalibrations_e;

/**
 *  \brief Enum to run desired InitCals algorithms.
 */	
typedef enum adi_adrv9001_InitCalMode
{
    ADI_ADRV9001_INITCAL_RUN_ALL_ALGO_MODE = 0, /*!< Run all selected algorithms on all profiles */
    ADI_ADRV9001_INITCAL_RUN_RX_ALGO_MODE  = 1, /*!< Run selected RX algorithms on RX profiles */
    ADI_ADRV9001_INITCAL_RUN_LB_ALGO_MODE  = 2  /*!< Run all selected algorithms on Loopback profiles */
}adi_adrv9001_InitCalMode_e;

/**
* \brief Data structure to hold Cals Init structures
*/
typedef struct adi_adrv9001_InitCals
{
    uint32_t sysInitCalMask;                              /*!< Calibration bit mask for non-channel related init cals */
    uint32_t chanInitCalMask[ADI_ADRV9001_MAX_RX_ONLY];   /*!< Array containing calibration bit mask for channel related init cals.
                                                               It contains two masks:
                                                               1. chanInitCalMask[0]: CH_1 for marks on RX1/TX1 channels, 
                                                               2. chanInitCalMask[1]: CH_2 for masks on RX2/TX2 channels */
    adi_adrv9001_InitCalMode_e  calMode;                  /*!< Enum specifies the mode to run desired InitCals algorithms */
} adi_adrv9001_InitCals_t;
    
    
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_CALS_TYPES_H_ */