/*!
* \file
* \brief Contains ADRV9001 API configuration and run-time type definitions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TYPES_H_
#define _ADI_ADRV9001_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* ADI specific header files */
#include "adi_common.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adrv9001_shared_resource_manager_types.h"

/* Header files related to libraries */


/* System header files */


/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

#define ADI_ADRV9001_TX_PROFILE_VALID		  0x01
#define ADI_ADRV9001_RX_PROFILE_VALID		  0x02
#define ADI_ADRV9001_ORX_PROFILE_VALID		  0x04
#define ADI_ADRV9001_LB_PROFILE_VALID		  0x08

#define ADI_ADRV9001_NUM_CHANNELS 2
#define ADI_ADRV9001_NUM_PORTS 2

/* TODO: Scrub defines below */
/* Tx 1/2 and Rx 1/2*/
#define ADI_ADRV9001_MAX_NUM_CHANNELS         4

#define ADI_ADRV9001_MAX_TXCHANNELS           2
#define ADI_ADRV9001_MAX_RXCHANNELS           8
#define ADI_ADRV9001_MAX_RX_ONLY              2
#define ADI_ADRV9001_MAX_ORX_ONLY             2
#define ADI_ADRV9001_MAX_ILB_ONLY             2
#define ADI_ADRV9001_MAX_ELB_ONLY             2

#define ADI_ADRV9001_TX_INITIALIZED_CH_OFFSET 8

#define ADI_ADRV9001_MAX_TX_CHANNEL_START     0
#define ADI_ADRV9001_MAX_TX_CHANNEL_END       1
#define ADI_ADRV9001_MAX_RX_CHANNEL_START     0
#define ADI_ADRV9001_MAX_RX_CHANNEL_END       1
#define ADI_ADRV9001_MAX_ORX_CHANNEL_START    2
#define ADI_ADRV9001_MAX_ORX_CHANNEL_END      3
#define ADI_ADRV9001_MAX_LB_CHANNEL_START     4
#define ADI_ADRV9001_MAX_LB_CHANNEL_END       7

#define ADI_ADRV9001_MAX_RX_ORX_CHANNELS	  ( ADI_ADRV9001_MAX_RX_ONLY + ADI_ADRV9001_MAX_ORX_ONLY )

#define ADI_ADRV9001_MAX_AUXDACS              4U

#define ADI_ADRV9001_MAX_NUM_PLL              5
#define ADI_ADRV9001_NUM_RF_PLL               2

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/**
 *  \brief Enum of possible ADRV9001 HS divider settings
 */
typedef enum adi_adrv9001_HsDiv
{
    ADI_ADRV9001_HSDIV_4   = 0x0, /*!< High speed clock divide by 4 setting */
    ADI_ADRV9001_HSDIV_6   = 0x1, /*!< High speed clock divide by 6 setting */
    ADI_ADRV9001_HSDIV_8   = 0x2  /*!< High speed clock divide by 8 setting */
} adi_adrv9001_HsDiv_e;

/**
* \brief Possible device clock divisor values
*/
typedef enum adi_adrv9001_DeviceClockDivisor
{
    ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS = 0,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_2      = 1,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_4      = 2,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_8      = 3,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_16     = 4,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_32     = 5,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_64     = 6
} adi_adrv9001_DeviceClockDivisor_e;

/**
 *  \brief Enum of possible ADRV9001 internal Clock divisor settings
 */
typedef enum adi_adrv9001_InternalClock_Divisor
{
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_1 = 0x1, /*!< Clock divide by 1 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_2 = 0x2, /*!< Clock divide by 2 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_3 = 0x3, /*!< Clock divide by 3 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_4 = 0x4, /*!< Clock divide by 4 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_5 = 0x5, /*!< Clock divide by 5 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_6 = 0x6, /*!< Clock divide by 6 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_7 = 0x7, /*!< Clock divide by 7 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_8 = 0x8, /*!< Clock divide by 8 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_9 = 0x9, /*!< Clock divide by 9 setting */
} adi_adrv9001_InternalClock_Divisor_e;

/**
 *  \brief Available modes for the CLK_PLL that drives the High Speed Digital Clock
 */
typedef enum adi_adrv9001_ClkPllMode
{
    ADI_ADRV9001_CLK_PLL_HP_MODE = 0x0,	/*!< Clock PLL HP Mode */
    ADI_ADRV9001_CLK_PLL_LP_MODE = 0x1,	/*!< Clock PLL LP Mode */
} adi_adrv9001_ClkPllMode_e;

/**
 *  \brief Data structure to hold ADRV9001 API State
 */
typedef enum adi_adrv9001_ApiStates
{
    ADI_ADRV9001_STATE_POWERON_RESET    = 0x00,
    ADI_ADRV9001_STATE_ANA_INITIALIZED  = 0x01,
    ADI_ADRV9001_STATE_DIG_INITIALIZED  = 0x02,
    ADI_ADRV9001_STATE_STREAM_LOADED    = 0x04,
    ADI_ADRV9001_STATE_ARM_DEBUG_LOADED = 0x08,
    ADI_ADRV9001_STATE_ARM_LOADED       = 0x10,
    ADI_ADRV9001_STATE_INITCALS_RUN     = 0x20,
    ADI_ADRV9001_STATE_PRIMED           = 0x40,
    ADI_ADRV9001_STATE_IDLE             = 0x80,
    ADI_ADRV9001_STATE_STANDBY          = 0x100,
} adi_adrv9001_ApiStates_e;

/**
 * \brief Enumerated list of CMOS pads drive strength options for SPI_DO signal
 */
typedef enum adi_adrv9001_CmosPadDrvStr
{
    ADI_ADRV9001_CMOSPAD_DRV_WEAK   = 0,	/*!<    5pF load @ 75MHz */
    ADI_ADRV9001_CMOSPAD_DRV_STRONG = 1		/*!<  100pF load @ 20MHz */
} adi_adrv9001_CmosPadDrvStr_e;

/**
 * \brief Enumerated list of RFPLL phase synchronization modes
 *
 * RFPLL Phase sync requires extra time to sync each time the RFPLL frequency
 * is changed. If RFPLL phase sync is not required, it may be desired to
 * disable the feature to allow the RFPLL to lock faster.
 *
 * Depending on the desired accuracy of the RFPLL phase sync, several options
 * are provided.
 */
typedef enum adi_adrv9001_RfPllMcs
{
    ADI_ADRV9001_RFPLLMCS_NOSYNC = 0,                /*!< Disable RFPLL phase synchronization */
    ADI_ADRV9001_RFPLLMCS_INIT_AND_SYNC = 1,         /*!< Enable RFPLL phase sync init only */
    ADI_ADRV9001_RFPLLMCS_INIT_AND_CONTTRACK = 2     /*!< Enable RFPLL phase sync init and track continuously */
} adi_adrv9001_RfPllMcs_e;

/**
 * \brief Enumerated list of options to use to set the LO source for the Rx/Tx mixers.
 */
typedef enum adi_adrv9001_LoSel
{
    ADI_ADRV9001_LOSEL_LO1 = 1,
    ADI_ADRV9001_LOSEL_LO2 = 2,
    ADI_ADRV9001_LOSEL_AUXLO = 3
} adi_adrv9001_LoSel_e;

/**
 * \brief Enumerated list of options to use to set the LO source for the Tx, Rx, LB mixers.
 */
typedef enum adi_adrv9001_TxRxLoSel
{
    ADI_ADRV9001_TXRXLOSEL_RX_LO1 = 1,
    ADI_ADRV9001_TXRXLOSEL_RX_LO2 = 1,
    ADI_ADRV9001_TXRXLOSEL_TX_LO1 = 1,
    ADI_ADRV9001_TXRXLOSEL_TX_LO2 = 1,
    ADI_ADRV9001_TXRXLOSEL_AUXLO  = 2
} adi_adrv9001_TxRxLoSel_e;

/**
 * \brief Enumerated list of options to use to set the LO Gen Power.
 */
typedef enum adi_adrv9001_LoGenPower
{
    ADI_ADRV9001_LOGENPOWER_RFPLL_LDO = 1,
    ADI_ADRV9001_LOGENPOWER_OFFCHIP   = 2
} adi_adrv9001_LoGenPower_e;

/**
 * \brief Enumerated list of options to use to set the ADRV9001 PLL Output and Input modes.
 */
typedef enum adi_adrv9001_PllLoMode
{
    ADI_ADRV9001_INT_LO1       = 0,	/*!< Internal RFPLL LO1 frequency will output from ADRV9001 */
    ADI_ADRV9001_INT_LO2       = 0,	/*!< Internal RFPLL LO2 frequency will output from ADRV9001 */
    ADI_ADRV9001_EXT_LO_OUTPUT = 1,	/*!< External RFPLL LO in ADRV9001 to generate the LO frequency */
    ADI_ADRV9001_EXT_LO_INPUT1 = 2,	/*!< Provided external LO1 in ADRV9001 to generate the LO frequency */
    ADI_ADRV9001_EXT_LO_INPUT2 = 3	/*!< Provided external LO2 in ADRV9001 to generate the LO frequency */
} adi_adrv9001_PllLoMode_e;

/**
 * \brief Enumerated list of options to use to set Differential/Single Ended for Ext Lo
 */
typedef enum adi_adrv9001_ExtLoType
{
    ADI_ADRV9001_EXT_LO_DIFFERENTIAL = 0,
    ADI_ADRV9001_EXT_LO_SINGLE_ENDED = 1

} adi_adrv9001_ExtLoType_e;

/**
 * \brief Enumerated list of options for Rx pin selection
 */
typedef enum adi_adrv9001_RxPinSel
{
    ADI_ADRV9001_RX_A = 0,
    ADI_ADRV9001_RX_B = 1
    
} adi_adrv9001_RxRfInputSel_e;

/**
 *  \brief Enum to select the proper gain for the FIR
 */
typedef enum adi_adrv9001_FirGain
{
    ADRV9001_FIR_GAIN_NEG_12_DB   = -12, /*!< FIR gain -12 */
    ADRV9001_FIR_GAIN_NEG_6_DB    = -6,  /*!< FIR gain -6 */
    ADRV9001_FIR_GAIN_ZERO_DB     = 0,   /*!< FIR gain 0 */
    ADRV9001_FIR_GAIN_POS_6_DB    = 6,   /*!< FIR gain 6 */
    ADRV9001_FIR_GAIN_POS_9P54_DB = 9,   /*!< FIR gain 9.54 */
    ADRV9001_FIR_GAIN_POS_12_DB   = 12,  /*!< FIR gain 12 */
    ADRV9001_FIR_GAIN_POS_14_DB   = 14,  /*!< FIR gain 14 */
    ADRV9001_FIR_GAIN_POS_20_DB   = 20,  /*!< FIR gain 20 */
    ADRV9001_FIR_GAIN_POS_24_DB   = 24,  /*!< FIR gain 24 */
    ADRV9001_FIR_GAIN_POS_26_DB   = 26   /*!< FIR gain 26 */
} adi_adrv9001_FirGain_e;

/**
 * \brief Enumeration of SSI type
 */
typedef enum adi_adrv9001_SsiType
{
    ADI_ADRV9001_SSI_TYPE_DISABLE = 0,  /*!< Disable SSI Type */
    ADI_ADRV9001_SSI_TYPE_CMOS = 1,     /*!< CMOS SSI Type */
    ADI_ADRV9001_SSI_TYPE_LVDS = 2      /*!< LVDS SSI Type */
} adi_adrv9001_SsiType_e;

/**
 * \brief Enumeration of the SSI data format
 */
typedef enum adi_adrv9001_SsiDataFormat
{
    ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA = 0,  /*!< 2 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA = 1,  /*!< 8 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA = 2, /*!< 16 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA = 3,    /*!< 12 bit I/Q data (LVDS) */
    ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA = 4     /*!< 16 bit I/Q data (CMOS/LVDS) */
} adi_adrv9001_SsiDataFormat_e;

/**
 * \brief Enumeration of the SSI number of lane
 */
typedef enum adi_adrv9001_SsiNumLane
{
    ADI_ADRV9001_SSI_1_LANE = 0, /*!< 1 lane (CMOS/LVDS) */
    ADI_ADRV9001_SSI_2_LANE = 1, /*!< 2 lane (LVDS) */
    ADI_ADRV9001_SSI_4_LANE = 2  /*!< 4 lane (CMOS) */
} adi_adrv9001_SsiNumLane_e;

/**
 * \brief Enumeration of the SSI Strobe
 */
typedef enum adi_adrv9001_SsiStrobeType
{
    ADI_ADRV9001_SSI_SHORT_STROBE = 0, /*!< Short SSI Strobe */
    ADI_ADRV9001_SSI_LONG_STROBE  = 1  /*!< Long SSI Strobe */
} adi_adrv9001_SsiStrobeType_e;

typedef enum adi_adrv9001_AdcType
{
    ADI_ADRV9001_ADC_LP = 0, /*!< Low Power ADC */
    ADI_ADRV9001_ADC_HP = 1  /*!< High Power ADC */
}adi_adrv9001_AdcType_e;

typedef enum adi_adrv9001_DuplexMode
{
    ADI_ADRV9001_TDD_MODE = 0,
    ADI_ADRV9001_FDD_MODE = 1
} adi_adrv9001_DuplexMode_e;

/**
 * \brief SSI modes
 */
typedef enum adi_adrv9001_SsiMode
{
    ADI_ADRV9001_CMOS_1LANE_16I16Q = 0,
    ADI_ADRV9001_CMOS_1LANE_16I,
    ADI_ADRV9001_CMOS_1LANE_8I,
    ADI_ADRV9001_CMOS_1LANE_2I,
    ADI_ADRV9001_CMOS_4LANE_16I16Q,
    ADI_ADRV9001_LVDS_2LANE_16I16Q
} adi_adrv9001_SsiMode_e;

/**
 * \brief LO divider mode
 */
typedef enum adi_adrv9001_LoDividerMode
{
    ADI_ADRV9001_LO_DIV_MODE_HIGH_PERFORMANCE = 0,
    ADI_ADRV9001_LO_DIV_MODE_LOW_POWER
} adi_adrv9001_LoDividerMode_e;
    
/**
 * \brief Available types of PLLs
 */
typedef enum adi_adrv9001_PllType
{
    ADI_ADRV9001_PLL_TYPE_CLK,
    ADI_ADRV9001_PLL_TYPE_RF1,
    ADI_ADRV9001_PLL_TYPE_RF2,
    ADI_ADRV9001_PLL_TYPE_AUX
} adi_adrv9001_PllType_e;

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/
    
/**
* \brief Data structure to hold SPI settings for all system device types
 */
typedef struct adi_adrv9001_SpiSettings
{
    uint8_t msbFirst;                           		/*!< 1 = MSB First, 0 = LSB First Bit order for SPI transaction */
    uint8_t enSpiStreaming;                     		/*!< Not Recommended - most registers in ADRV9001 API are not consecutive */
    uint8_t autoIncAddrUp;                      		/*!< For SPI Streaming, set address increment direction. 1= next addr = addr+1, 0:addr = addr-1 */
    uint8_t fourWireMode;                       		/*!< 1: Use 4-wire SPI, 0: 3-wire SPI (SDIO pin is bidirectional). NOTE: ADI's FPGA platform always uses 4-wire mode */
    adi_adrv9001_CmosPadDrvStr_e cmosPadDrvStrength;   	/*!< Drive strength of CMOS pads when used as outputs (SDIO, SDO, GP_INTERRUPT, GPIO 1, GPIO 0) */
} adi_adrv9001_SpiSettings_t;

/**
 *  \brief Data structure to hold digital clock settings
 */
typedef struct adi_adrv9001_ClockSettings
{
    uint32_t deviceClock_kHz;                           /*!< Device clock frequency in kHz */
    uint32_t clkPllVcoFreq_kHz;                         /*!< CLKPLL VCO frequency in kHz */
    adi_adrv9001_HsDiv_e  clkPllHsDiv;                  /*!< CLKPLL high speed clock divider */
    adi_adrv9001_ClkPllMode_e  clkPllMode;              /*!< CLKPLL Mode */
    adi_adrv9001_InternalClock_Divisor_e  clk1105Div;   /*!< CLK1105 clock divider */
    adi_adrv9001_InternalClock_Divisor_e  armClkDiv;    /*!< ARM Clock divider */
    uint8_t padRefClkDrv;                               /*!< Output Clock Buffer Drive (valid 0-3) */
    uint32_t extLo1OutFreq_kHz;                         /*!< EXT LO1 output frequency in kHz */
    uint32_t extLo2OutFreq_kHz;                         /*!< EXT LO2 output frequency in kHz */
    adi_adrv9001_PllLoMode_e rfPll1LoMode;              /*!< internal LO generation for RF LO1, internal LO can be output, or external LO can be input. */
    adi_adrv9001_PllLoMode_e rfPll2LoMode;              /*!< internal LO generation for RF LO2, internal LO can be output, or external LO can be input. */
    adi_adrv9001_ExtLoType_e ext1LoType;                /*!< 0 = Differential, 1= Single Ended */
    adi_adrv9001_ExtLoType_e ext2LoType;                /*!< 0 = Differential, 1= Single Ended */
    adi_adrv9001_RxRfInputSel_e  rx1RfInputSel;         /*!< 0 = Rx1A, 1 = Rx1B */
    adi_adrv9001_RxRfInputSel_e  rx2RfInputSel;         /*!< 0 = Rx2A, 1 = Rx2B */
    uint16_t extLo1Divider;					            /*!< External LO1 Output divider (valid 2 to 1022) */
    uint16_t extLo2Divider;					            /*!< External LO2 Output divider (valid 2 to 1022) */
    adi_adrv9001_RfPllMcs_e rfPllPhaseSyncMode;         /*!< Set RF PLL phase synchronization mode. Adds extra time to lock RF PLL when PLL frequency changed. See enum for options */
    adi_adrv9001_LoSel_e rx1LoSelect;		            /*!< Rx1 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e rx2LoSelect;		            /*!< Rx2 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e tx1LoSelect;		            /*!< Tx1 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e tx2LoSelect;		            /*!< Tx2 LO select: 1 = LO1, 2 = LO2  */

    adi_adrv9001_LoDividerMode_e rx1LoDivMode;        /*!< RX1 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e rx2LoDivMode;        /*!< RX2 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e tx1LoDivMode;        /*!< TX1 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e tx2LoDivMode;        /*!< TX2 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */

    adi_adrv9001_LoGenPower_e loGen1Select;		      /*!< D0 - LoGen1 Sel: 0 = RFPLL1_LDO, 1 = OffChip */
    adi_adrv9001_LoGenPower_e loGen2Select;		      /*!< D1 - LoGen2 Sel: 0 = RFPLL2_LDO, 1 = OffChip */
} adi_adrv9001_ClockSettings_t;

/**
 *  \brief Data structure to hold ADRV9001 Rx FIR filter settings
 */
typedef struct adi_adrv9001_RxFir
{
    int8_t gain_dB;											/*!< Filter gain in dB (-12dB, -6dB, 0dB, +6dB) */
    uint8_t numFirCoefs;									/*!< Number of coefficients in the FIR filter */
    int32_t coefs[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE];	/*!< An array of filter coefficients */
} adi_adrv9001_RxFir_t;

/**
 *  \brief Data structure to hold ADRV9001 Tx FIR filter settings
 */
typedef struct adi_adrv9001_TxFir
{
    int8_t gain_dB;											/*!< Filter gain in dB (-12dB, -6dB, 0dB, +6dB) */
    uint8_t numFirCoefs;									/*!< Number of coefficients in the FIR filter */
    int32_t coefs[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE];	/*!< An array of filter coefficients */
} adi_adrv9001_TxFir_t;

/**
 * \brief Data structure to hold ADRV9001 SSI configuration.
 */
typedef struct adi_adrv9001_SsiConfig
{
    adi_adrv9001_SsiType_e       ssiType;					/*!< SSI type */
    adi_adrv9001_SsiDataFormat_e ssiDataFormatSel;			/*!< SSI data format */
    adi_adrv9001_SsiNumLane_e    numLaneSel;				/*!< SSI number of lanes */
    adi_adrv9001_SsiStrobeType_e strobeType;				/*!< SSI strobe type */
    uint8_t						 lsbFirst;					/*!< SSI LSB first */
    uint8_t						 qFirst;					/*!< SSI Q data first */
    bool						 refClockGpioEn;			/*!< Reference clock GPIO enable */
    uint8_t						 lvdsBitInversion;			/*!< LVDS SSI bit inversion */
    uint8_t						 lvdsUseLsbIn12bitMode;		/*!< LVDS use LSB in 12 bit mode */
    bool						 lvdsTxFullRefClkEn;        /*!< LVDS Tx full refclk enable */
    bool						 lvdsRxClkInversionEn;      /*!< LVDS Rx clock inversion enable */
    uint32_t					 rfLvdsDiv;					/*!< RF LVDS clock = BBPLL / rfLvdsDiv */
    bool                         cmosTxDdrNegStrobeEn;      /*!< CMOS Tx DDR negative strobe enable */
    bool                         cmosDdrPosClkEn;           /*!< CMOS DDR positive clock enable */
    bool                         cmosDdrClkInversionEn;     /*!< CMOS DDR clock inversion enable */
    bool                         cmosDdrEn;                 /*!< CMOS DDR enable */
} adi_adrv9001_SsiConfig_t;

/**
 * \brief Data structure to hold ADRV9001 CMOS delay calibration configuration for each channel.
 */
typedef struct adi_adrv9001_CmosSsiCalibrationCfg
{
    uint8_t cmosRxClkInvert[ADI_ADRV9001_MAX_RX_ONLY];      /*!< CMOS CLK adjustment for Rx1/Rx2 channels */
    uint8_t cmosTxClkInvert[ADI_ADRV9001_MAX_TXCHANNELS];   /*!< CMOS CLK adjustment for Tx1/Tx2 channels */
} adi_adrv9001_CmosSsiCalibrationCfg_t;

/**
 * \brief Data structure to hold ADRV9001 LVDS delay calibration configuration for each channel.
 */
typedef struct adi_adrv9001_LvdsSsiCalibrationCfg
{
    uint8_t lvdsRxClkDelay[ADI_ADRV9001_MAX_RX_ONLY];       /*!< LVDS CLK delay for Rx1/Rx2 channels */
    uint8_t lvdsRxStrobeDelay[ADI_ADRV9001_MAX_RX_ONLY];    /*!< LVDS Strobe delay for Rx1/Rx2 channels */
    uint8_t lvdsRxIDataDelay[ADI_ADRV9001_MAX_RX_ONLY];     /*!< LVDS I data delay for Rx1/Rx2 channels */
    uint8_t lvdsRxQDataDelay[ADI_ADRV9001_MAX_RX_ONLY];     /*!< LVDS Q data delay for Rx1/Rx2 channels */

    uint8_t lvdsTxClkDelay[ADI_ADRV9001_MAX_TXCHANNELS];    /*!< LVDS CLK delay for Tx1/Tx2 channels */
    uint8_t lvdsTxStrobeDelay[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< LVDS Strobe delay for Tx1/Tx2 channels */
    uint8_t lvdsTxIDataDelay[ADI_ADRV9001_MAX_TXCHANNELS];  /*!< LVDS I data delay for Tx1/Tx2 channels */
    uint8_t lvdsTxQDataDelay[ADI_ADRV9001_MAX_TXCHANNELS];  /*!< LVDS Q data delay for Tx1/Tx2 channels */
} adi_adrv9001_LvdsSsiCalibrationCfg_t;

/**
 * \brief Data structure to hold ADRV9001 SSI delay calibration configuration for each channel.
 */
typedef struct adi_adrv9001_SsiCalibrationCfg
{
    adi_adrv9001_CmosSsiCalibrationCfg_t cmosSsiCalibrationCfg;  /*!< Delay configuration for CMOS */
    adi_adrv9001_LvdsSsiCalibrationCfg_t lvdsSsiCalibrationCfg;  /*!< Delay configuration for LVDS */
} adi_adrv9001_SsiCalibrationCfg_t;

/**
 *  \brief Data structure to hold settings for the current Rx specific use case profile
 */
typedef struct adi_adrv9001_RxProfile
{
    uint32_t		primarySigBandwidth_Hz;			/*!< Rx primary signal BW in Hz */
    uint32_t		rxOutputRate_Hz;				/*!< Rx output data rate in Hz */
    uint32_t		rxInterfaceSampleRate_Hz;		/*!< Rx sample rate at serial interface */
    int32_t			rxOffsetLo_kHz;                 /*!< Offset in kHz. 0: On LO */
    uint8_t			rxSignalOnLo;                   /*!< !0: Signal on LO  0: no signal on LO */
    adi_adrv9001_RxSignalType_e outputSignaling;    /*!< Output to BBIC signal type */
    uint8_t			filterOrder;					/*!< 1st or 2nd order ABBF filter */
    uint8_t			filterOrderLp;					/*!< 1st or 2nd order ABBF filter Low Power ADC*/
    uint32_t		hpAdcCorner;					/*!< High Power ADC corner freq*/
    uint32_t		adcClk_kHz;						/*!< ADC clock 2.2G/1.47G/1.1G */
    uint32_t		rxCorner3dB_kHz;				/*!< TIA bandwidth in kHz */
    uint32_t		rxCorner3dBLp_kHz;				/*!< TIA bandwidth for Low Power ADC */
    uint8_t			tiaPower;                       /*!< RX TIA Power setting */
    uint8_t			tiaPowerLp;                     /*!< RX TIA Power setting Low Power ADC*/
    adi_adrv9001_RxChannels_e	channelType;		/*!< Channel type described by this profile (Rx/ORx/Loopback) */
    adi_adrv9001_AdcType_e		adcType;			/*!< ADC type: low/high power ADC */
    adi_adrv9001_RxDpProfile_t	rxDpProfile;		/*!< RX digital data path config */
    adi_adrv9001_SsiConfig_t	rxSsiConfig;		/*!< RX Serial data interface config */
} adi_adrv9001_RxProfile_t;

/**
 *  \brief Data structure to hold Rx data path settings
 */
typedef struct adi_adrv9001_RxChannelCfg
{
    adi_adrv9001_RxProfile_t profile; /*!< Rx datapath profile, 3dB corner frequencies, and digital filter enables */
} adi_adrv9001_RxChannelCfg_t;

/**
 *  \brief Data structure to hold ADRV9001 Rx Channel configuration settings
 */
typedef struct adi_adrv9001_RxSettings
{
    uint32_t rxInitChannelMask;                                            /*!< Rx channel mask of which channels to initialize */
    adi_adrv9001_RxChannelCfg_t rxChannelCfg[ADI_ADRV9001_MAX_RXCHANNELS]; /*!< Rx settings per Rx channel */
} adi_adrv9001_RxSettings_t;

/**
* \brief Data Structure to hold ADRV9001 device Rx Max and Min gain indices
*/
typedef struct
{
    /* FIXME: Vivek - Need to check if gain index is needed for ILB and ELB */
    uint8_t rx1MinGainIndex;	/*!< Current device minimum Rx1 gain index */
    uint8_t rx1MaxGainIndex;	/*!< Current device maximum Rx1 gain index */
    uint8_t rx2MinGainIndex;	/*!< Current device minimum Rx2 gain index */
    uint8_t rx2MaxGainIndex;	/*!< Current device maximum Rx2 gain index */
} adi_adrv9001_GainIndex_t;

/**
 *  \brief Data structure to hold ADRV9001 Tx Profile Configuration settings
 */
typedef struct adi_adrv9001_TxProfile
{
    uint32_t primarySigBandwidth_Hz;              /*!< Tx primary signal BW in Hz*/
    uint32_t txInputRate_Hz;                      /*!< Tx input data rate in Hz */
    uint32_t txInterfaceSampleRate_Hz;			  /*!< TX sample rate at serial interface */
    uint32_t validDataDelay;					  /*!< Valid data delay relative to TX Enable by 184MHz clock counter */
    uint32_t txBbf3dBCorner_kHz;                  /*!< Tx BBF 3dB corner in kHz - butterFilterBw */
    adi_adrv9001_TxSignalType_e  outputSignaling; /*!< Output to Analog signal type */
    uint8_t  txPdBiasCurrent;					  /*!< pre-distorter programmable bias current*/
    uint8_t  txPdGainEnable;					  /*!< TX Pre-distortion gain enable */

    uint32_t txPrePdRealPole_kHz;				  /*!< TX Pre-distortion pole */

    uint32_t txPostPdRealPole_kHz;				  /*!< Post-distorter (i.e. interstage) filter Fc  */
    uint8_t  txBbfPowerMode;                      /*!< TXBBF  filter power mode */
    uint8_t  txExtLoopBackType;					  /*!< 0: No external loopback connect,
                                                   *   1: loopback before PA,
                                                   *   2: loopback after PA. */
    uint8_t  txExtLoopBackForInitCal;			  /*!< 0: ext loop back should not be used for init cal */
    uint32_t frequencyDeviation_Hz;               /*!< frequency deviation value in Hz for both FM_IQ and FM_DM.*/
    adi_adrv9001_TxDpProfile_t txDpProfile;       /*!< TX digital data path config */
    adi_adrv9001_SsiConfig_t   txSsiConfig;       /*!< TX Serial data interface config */
} adi_adrv9001_TxProfile_t;

/**
 *  \brief Data structure to hold ADRV9001 Tx Channel configuration settings
 */
typedef struct adi_adrv9001_TxSettings
{
    uint32_t txInitChannelMask;                                            /*!< Tx channel mask of which channels to initialize */
    adi_adrv9001_TxProfile_t txProfile[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Tx settings per Tx channel */
} adi_adrv9001_TxSettings_t;

/**
* \brief Data structure to hold WB/NB compensation PFIR structure
*/
typedef struct adi_adrv9001_pfirWbNbBuffer
{
    uint8_t  		numCoeff;                  /*!< number of coefficients */
    adi_adrv9001_PfirSymmetric_e symmetricSel; /*!< symmetric selection */
    adi_adrv9001_PfirNumTaps_e   tapsSel;      /*!< taps selection */
    adi_adrv9001_PfirGain_e      gainSel;      /*!< gain selection */
    int32_t 		coefficients[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE]; /*!< coefficients */
} adi_adrv9001_PfirWbNbBuffer_t;

/**
* \brief Data structure to hold RX NB pulse shaping RFIR Buffer structure
*/
typedef struct adi_adrv9001_PfirPulseBuffer_t
{
    uint8_t  		numCoeff;			       /*!< number of coefficients */
    adi_adrv9001_PfirSymmetric_e symmetricSel; /*!< symmetric selection */
    uint8_t         taps;           	       /*!< taps in number */
    adi_adrv9001_PfirGain_e      gainSel;      /*!< gain selection */
    int32_t 		coefficients[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE]; /*!< coefficients   */
} adi_adrv9001_PfirPulseBuffer_t;

/**
* \brief Data structure to hold RX Low/High TIA Bandwidth HP/LP ADC PFIR Buffer structure
*/
typedef struct adi_adrv9001_PfirMag21Buffer
{
    uint8_t	  numCoeff;
    int32_t	  coefficients[ADI_ADRV9001_MAG_COMP_PFIR_COEFS_MAX_SIZE];
} adi_adrv9001_PfirMag21Buffer_t;

/**
* \brief Data structure to hold TX/RX Magnitude Compensation PFIR for NB
*/
typedef struct {
    uint8_t	  numCoeff;
    int32_t   coefficients[ADI_ADRV9001_MAG_COMP_NB_PFIR_COEFS_MAX_SIZE];
} adi_adrv9001_PfirMag13Buffer_t;

/*! PFIR coefficent buffer */
typedef struct adi_adrv9001_PfirBuffer
{
    /*!< During dynamic profile switching, the first two types of PFIRs, RXWbNb and TXWbNbPulShp, would be reloaded for each
     *   profile before the switch */
    /*!< RX WB/NB Compensation PFIR (Channel Filter) coefficient Bank A/B/C/D  in rxnb_dem, block30 */
    adi_adrv9001_PfirWbNbBuffer_t     pfirRxWbNbChFilterCoeff_A;
    adi_adrv9001_PfirWbNbBuffer_t     pfirRxWbNbChFilterCoeff_B;
    adi_adrv9001_PfirWbNbBuffer_t     pfirRxWbNbChFilterCoeff_C;
    adi_adrv9001_PfirWbNbBuffer_t     pfirRxWbNbChFilterCoeff_D;

    /*!< TX WB/NB Preprocessing pulse shaping PFIR coefficient Bank A/B/C/D */
    adi_adrv9001_PfirWbNbBuffer_t     pfirTxWbNbPulShpCoeff_A;
    adi_adrv9001_PfirWbNbBuffer_t     pfirTxWbNbPulShpCoeff_B;
    adi_adrv9001_PfirWbNbBuffer_t     pfirTxWbNbPulShpCoeff_C;
    adi_adrv9001_PfirWbNbBuffer_t     pfirTxWbNbPulShpCoeff_D;

    /*!< RX NB Pulse Shaping pFIR  128 taps CH1/2 in rxnb_dem */
    adi_adrv9001_PfirPulseBuffer_t     pfirRxNbPulShp[ADI_ADRV9001_MAX_RX_ONLY];

    /*!< Channel 1/2 Low/High TIA Bandwidth HP/LP ADC */
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagLowTiaLowSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagLowTiaHighSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagHighTiaHighSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagLowTiaLowSRLp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagLowTiaHighSRLp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t     pfirRxMagHighTiaHighSRLp[ADI_ADRV9001_MAX_RX_ONLY];

    /*!< TX Magnitude Compensation PFIR 21 taps */
    adi_adrv9001_PfirMag21Buffer_t     pfirTxMagComp1;
    adi_adrv9001_PfirMag21Buffer_t     pfirTxMagComp2;

    /*!< TX/RX Magnitude Compensation PFIR for NB */
    adi_adrv9001_PfirMag13Buffer_t     pfirTxMagCompNb[ADI_ADRV9001_MAX_TXCHANNELS];
    adi_adrv9001_PfirMag13Buffer_t     pfirRxMagCompNb[ADI_ADRV9001_MAX_RX_ONLY];
} adi_adrv9001_PfirBuffer_t;

/**
* \brief Data structure to hold PLL modulus settings
*/
typedef struct adi_adrv9001_pllModulus
{
    uint32_t        modulus[ADI_ADRV9001_MAX_NUM_PLL];  /*!< PLL modulus */
    uint32_t        dmModulus[ADI_ADRV9001_NUM_RF_PLL]; /*!< RF PLL modulus in DM mode */
} adi_adrv9001_pllModulus_t;

/**
* \brief Data structure to hold Device system configuration
*/
typedef struct adi_adrv9001_DeviceSysConfig
{
    adi_adrv9001_DuplexMode_e   duplexMode;
    uint8_t						fhModeOn;
    uint8_t						monitorModeOn;
    uint8_t						numDynamicProfile;  /*!< Number of dynamic Profile */
    uint8_t                     extMcsOn;           /*!< External MCS On flag. 0 means off */
    adi_adrv9001_AdcType_e      adcTypeMonitor;     /*!< ADC type used in Monitor Mode */
    adi_adrv9001_pllModulus_t   pllModulus;         /*!< PLL modulus */
} adi_adrv9001_DeviceSysConfig_t;

/**
* \brief Data structure to hold ADRV9001 device instance initialization settings
*/
typedef struct adi_adrv9001_Init
{
    adi_adrv9001_ClockSettings_t    clocks;			/*!< Holds settings for CLKPLL and reference clock */
    adi_adrv9001_RxSettings_t       rx;				/*!< Rx settings data structure */
    adi_adrv9001_TxSettings_t       tx;				/*!< Tx settings data structure */
    adi_adrv9001_DeviceSysConfig_t  sysConfig;		/*!< Device system config struct */
    adi_adrv9001_PfirBuffer_t       pfirBuffer;		/*!< Holds the Data Interface CSSI/LSSI data link settings */
} adi_adrv9001_Init_t;

/**
* \brief Data structure to hold clock divide ratios
*/
typedef struct adi_adrv9001_ClkDivideRatios
{
    uint8_t devClkDivideRatio; /*!< Dev clock divide ratio */
    uint8_t refClkDivideRatio; /*!< Ref clock divide ratio */
    uint8_t armClkDivideRatio; /*!< ARM clock divide ratio w.r.t hsDigClk*/
    uint8_t agcClkDivideRatio; /*!< AGC module clock divide ratio w.r.t hsDigClk*/
    uint8_t regClkDivideRatio; /*!< Register bus clock divide ratio w.r.t hsDigClk*/
    uint8_t txAttenDeviceClockDivideRatio; /*!< Tx Atten module clock divide ratio w.r.t hsDigClk*/
    uint8_t anaRefClockRatio;  /*!< Analog clock divide ratio */
} adi_adrv9001_ClkDivideRatios_t;

/**
* \brief Data structure to hold a ADRV9001 device instance status information
*
* Application layer allocates this memory, but only API writes to it to keep up with the API run time state.
*/
/* FIXME: Vivek - Commented structures that are not needed for initial porting. TBD in future when more features are added */
typedef struct adi_adrv9001_Info
{
    uint8_t deviceSiRev;												/*!< ADRV9001 silicon rev read during ADRV9001_initialize */
    uint8_t deviceProductId;                                            /*!< ADRV9001 Product ID read during HWOpen */
    uint16_t devState;									                /*!< Current device state of the part, i.e., radio on, radio off, arm loaded,
                                                                             etc., defined by deviceState enum */
    uint32_t initializedChannels;										/*!< Holds Rx/ORx/Tx channels that were initialized and calibrated for the
                                                                             current device */
    uint32_t trackingCalEnableMask;										/*!< Keeps track of tracking calibration mask of Rx and ORx as set by the
                                                                             customer through the api */
    uint32_t profilesValid;												/*!< Current device profilesValid bit field for use notification, i.e.,
                                                                             Tx = 0x01, Rx = 0x02, Orx = 0x04 */
    adi_adrv9001_TxAttenStepSize_e txAttenStepSize;                     /*!< Current tx Atten step size */
    adi_adrv9001_TxSignalType_e outputSignaling[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Output to Analog signal type */
    uint32_t swTest;													/*!< Software testmode signal */
    uint32_t hsDigClk_kHz;												/*!< Calculated in initialize() digital clock used throughout API functions */
    uint32_t deviceClock_kHz;                                           /*!< Device clock frequency in kHz (copy from adi_adrv9001_ClockSettings_t struct) */
    adi_adrv9001_ClkPllMode_e  clkPllMode;                              /*!< CLKPLL Mode */
    adi_adrv9001_ClkDivideRatios_t clkDivideRatios;						/*!< Clock divide ratios w.r.t hsDigClk for various modules in the device */
    uint32_t  profileAddr;												/*!< Address to load Profile */
    uint32_t  adcProfileAddr;											/*!< Address to load ADC Profile */
    uint32_t  pfirProfileAddr;                                          /*!< Address to load PFIR coefficients */
    uint32_t txInputRate_kHz[ADI_ADRV9001_MAX_TXCHANNELS];				/*!< Tx Input sample rate from currently loaded profile */
    uint32_t rxOutputRate_kHz[ADI_ADRV9001_MAX_RXCHANNELS];				/*!< Rx Output sample rate from currently loaded profile */
    adi_adrv9001_GainIndex_t gainIndexes;								/*!< Current device Rx min max gain index values */
    uint16_t chunkStreamImageSize[12];									/*!< Stream Image Size */
    uint16_t chunkStreamImageOffset[12];								/*!< Stream Image Offset */
    uint32_t currentStreamBinBaseAddr;									/*!< Address to load current stream */
    uint32_t currentStreamBaseAddr;										/*!< Address to load current stream base */
    uint8_t currentStreamNumberStreams;									/*!< Number of Streams for current stream  */
    uint8_t currentStreamImageIndex;									/*!< Index of current stream  */
    uint32_t currentStreamImageSize;									/*!< Image size of current stream */
    adrv9001_SharedResourcePool_t sharedResourcePool[ADRV9001_NUM_SHARED_RESOURCES]; /*!< Shared resource pool for the given device instance*/
} adi_adrv9001_Info_t;

/**
* \brief Data structure to hold ADRV9001 device instance settings
*/
typedef struct adi_adrv9001_Device
{
    adi_common_Device_t			common;        /*!< Common layer structure */
    adi_adrv9001_Info_t			devStateInfo;  /*!< ADRV9001 run time state information container */
    adi_adrv9001_SpiSettings_t	spiSettings;   /*!< Pointer to ADRV9001 SPI Settings */
    /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
    /* TODO: Add device descriptor structure     */
    /* TODO: Add capabilities structure  - what is licensed    */
} adi_adrv9001_Device_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_TYPES_H_ */
