/**
* \file
* \brief Contains ADRV9001 API Tx datapath data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the FPGA9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TX_TYPES_H_
#define _ADI_ADRV9001_TX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* System header files */
#include <stdbool.h>

#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_gpio_types.h"

/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

#define ADRV9001_TX_ATTEN_TABLE_MAX 840

/* FIXME: Vivek - Need to confirm all these values with ARM firmware team */
#define ADRV9001_ADDR_TX1_ATTEN_TABLE 0x45300000
#define ADRV9001_ADDR_TX2_ATTEN_TABLE 0x45400000

#define ADRV9001_TX_PA_RAMP_LUT_SIZE        256

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/**
*  \brief Enum to set the Tx Attenuation step size
*/
typedef enum adi_adrv9001_TxAttenStepSize
{
    ADI_ADRV9001_TXATTEN_0P05_DB = 0,    /*!< Tx attenuation 0.05dB step size */
    ADI_ADRV9001_TXATTEN_0P1_DB  = 1,    /*!< Tx attenuation 0.1dB step size */
    ADI_ADRV9001_TXATTEN_0P2_DB  = 2,    /*!< Tx attenuation 0.2dB step size */
    ADI_ADRV9001_TXATTEN_0P4_DB  = 3     /*!< Tx attenuation 0.4dB step size */
} adi_adrv9001_TxAttenStepSize_e;

/**
*  \brief Enum for the different attenuation mode
*/
typedef enum adi_adrv9001_TxAttenMode
{
    ADI_ADRV9001_TXATTEN_BYPASS_MODE    = 0,     /*!< Tx attenuation mode Bypass: zero total attenuation */
    ADI_ADRV9001_TXATTEN_SPI_ATTEN_MODE = 1,     /*!< Tx attenuation mode set by 10-bit attenuation index used to determine total attenuation */

    /* FIXME: Vivek - Not clear why ADI_ADRV9001_TXATTEN_GPIO_MODE = 3, why not '2'; Need to confirm */
    /* JS: ADI_ADRV9001_TXATTEN_SPI_DIRECT_MODE = 2,    // Unclear how this works and if supported */
    ADI_ADRV9001_TXATTEN_GPIO_MODE      = 3,     /*!< Tx attenuation is control with GPIO Incr/Decr: total attenuation is altered incrementally using pin control */
} adi_adrv9001_TxAttenMode_e;

/**
 * \brief Enumerations of the Tx Data Path pre-processor Mode
 */
typedef enum adi_adrv9001_TxDpPreProcMode
{
    ADI_ADRV9001_TX_DP_PREPROC_MODE0 = 0u, /*!< Selects mode0 for the Tx PreProc, bypass mode                  */
    ADI_ADRV9001_TX_DP_PREPROC_MODE1 = 1u, /*!< Selects mode1 for the Tx PreProc, I and Q PFIR mode            */
    ADI_ADRV9001_TX_DP_PREPROC_MODE2 = 2u, /*!< Selects mode2 for the Tx PreProc, I PFIR only mode             */
    ADI_ADRV9001_TX_DP_PREPROC_MODE3 = 3u  /*!< Selects mode3 for the Tx PreProc, cascaded I and Q PFIR mode   */
} adi_adrv9001_TxDpPreProcMode_e;

/**
 * \brief Enumerations of the Tx Data Path IQDMDUC Mode
 */
typedef enum adi_adrv9001_TxDpIqDmDucMode
{
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE0 = 0u, /*!< Selects mode0 for the Tx IqDmDuc, bypass IQ in IQ out  */
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE1 = 1u, /*!< Selects mode1 for the Tx IqDmDuc, IQ DM                */
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE2 = 2u  /*!< Selects mode2 for the Tx IqDmDuc, DUC                  */
} adi_adrv9001_TxDpIqDmDucMode_e;

/**
 *  \brief Enum select for slew rate limiter statistics mode
 */
typedef enum adi_adrv9001_SrlStatisticsMode
{
    ADI_ADRV9001_SRL_STATISTICS_MIN_SLEW_FACTOR_OBSERVED = 0u,
    ADI_ADRV9001_SRL_STATISTICS_NUM_OF_SAMPLES_SLEW_RATE_LIMITED,
    ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_10,     /* clear statistics to zero, same as b11 below */
    ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_11      /* clear statistics to zero, same as b10 above */
} adi_adrv9001_SrlStatisticsMode_e;

/**
 *  \brief Enum select for slew rate table
 */
typedef enum adi_adrv9001_SrlTableSel
{
    ADI_ADRV9001_SRL_TABLE0 = 0x0000,        /*!< Slew limit is 10% of full scale */
    ADI_ADRV9001_SRL_TABLE1 = 0x0001,        /*!< Slew limit is 20% of full scale */
    ADI_ADRV9001_SRL_TABLE2 = 0x0002,        /*!< Slew limit is 30% of full scale */
    ADI_ADRV9001_SRL_TABLE3 = 0x0003         /*!< Slew limit is 50% of full scale */
} adi_adrv9001_SrlTableSel_e;

/**
*  \brief Enum select for PA protection input
*/
typedef enum adi_adrv9001_PaProtectionInputSel
{
    ADI_ADRV9001_COMPLEX_MULT_OUTPUT   = 0x0000, /*!< Input data to PA protection block is probed from complex mult output */
    ADI_ADRV9001_TXQEC_ACTUATOR_OUTPUT = 0x0001  /*!< Input data to PA protection block is probed from tx qec actuator output */
} adi_adrv9001_PaProtectionInputSel_e;

/**
 * \brief Enumeration of the Tx signal type
 */
typedef enum adi_adrv9001_TxSignalType
{
    ADI_ADRV9001_TX_IQ            = 0,
    ADI_ADRV9001_TX_IQ_FM_FSK     = 1,
    ADI_ADRV9001_TX_DIRECT_FM_FSK = 2
} adi_adrv9001_TxSignalType_e;

/**
 * \brief Configuration structure for Tx power control
 */
typedef struct adi_adrv9001_TxAttenuationConfig_t
{
    bool disableTxOnPllUnlock;                          /*!< If true, the PA will ramp down to the max attenuation if
                                                         *   an RF1 or RF2 PLL unlock occurs
                                                         *   NOTE: Currently read-only */
    bool dacFullScaleBoostEnable;                       /*!< Enables DAC full scale boost, increasing DAC output power by 3 dB */
    adi_adrv9001_TxAttenStepSize_e txAttenStepSize;	    /*!< Tx Attenuation step size */
    adi_adrv9001_TxAttenMode_e attenMode;				/*!< The mode to control Tx attenuation */
} adi_adrv9001_TxAttenuationConfig_t;

/**
* \brief Data structure to hold an adrv9001 device Tx attenuation Table Row
*        (Single row of a complete table)
*/
typedef struct adi_adrv9001_TxAttenTableRow
{
    uint16_t txAttenMult;       /*!< Digital attenuation multiplier; Valid range: 0-4095 */
    uint8_t  txAttenHp;         /*!< Analog coarse attenuation gain; Valid range: 0-63 */
    uint8_t  Reserve;
} adi_adrv9001_TxAttenTableRow_t;

/**
*  \brief Data structure to hold ADRV9001 Tx PA Protection configuration settings
*/
typedef struct adi_adrv9001_TxPaProtectCfg
{
    uint8_t avgDuration;                 /*!< PA Protection Average Power Measurement Duration. */
    uint8_t peakDuration;                /*!< PA Protection Peak Power Measurement Duration */
    uint8_t txAttenStep;                 /*!< PA Protection Attenuation gain step. Gain step down is not allowed for Tokelau device. This field is not being used actively. */
    uint8_t gainStepDownEn;              /*!< PA Protection Gain Step Down Enable. Gain step down is not allowed for Tokelau device. This field is not being used actively.*/
    uint16_t powerThreshold;             /*!< PA Protection Average Power Threshold. */
    uint16_t peakThreshold; /*!< PA Protection Peak Power Threshold. Max value for Silicon A: 255  Max Value for Silicon B: 8191 */
    uint8_t peakCount;                   /*!< Peak Count Causing PA Error. */
    uint8_t rampStepDuration;            /*!< PA Protection Ramp Step duration. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    uint8_t rampStepSize;                /*!< PA Protection Ramp Step Size. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    uint8_t rampMaxAtten;                /*!< PA Protection Ramp Max Attenuation. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    uint8_t avgPowerEnable;              /*!< This enables average power measurement block. If enabled, PA error is flagged when average power measurement is above average power threshold */
    uint8_t peakPowerEnable;             /*!< This enables peak power measurement block. If enabled, PA error is flagged when peak power count is above peak count threshold */
    uint8_t avgPeakRatioEnable; /*!< This enables average to peak power ratio calculation block, both avgPower and peakPower calculations must be enabled before enabling ratio calculation */
    adi_adrv9001_PaProtectionInputSel_e inputSel; /*!< This selects the source of input signal for Pa protection block */
} adi_adrv9001_TxPaProtectCfg_t;

/**
 * \brief Data structure to hold ADRV9001 TX pre-processor block
 */
typedef struct adi_adrv9001_TxPreProc
{
    /* tx pre-processor symbol map */
    uint32_t    txPreProcSymbol0;         /*!< TX Preprocessor symbol mapping symbol 0, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol1;         /*!< TX Preprocessor symbol mapping symbol 1, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol2;         /*!< TX Preprocessor symbol mapping symbol 2, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol3;         /*!< TX Preprocessor symbol mapping symbol 3, in S3.14 18-bit format */
    uint8_t     txPreProcSymMapDivFactor; /*!< Division factor that controls the symbol mapping. 5 bit (supports 1, 2, 3, 4, 5, 19, 16, 20)*/

    /* Tx pre-processor config parameters that support four modes of operation */
    adi_adrv9001_TxDpPreProcMode_e txPreProcMode; /*!< TX_DP_PREPROC_MODE0 - bypass mode,
                                                       TX_DP_PREPROC_MODE1 - I and Q PFIR mode,
                                                       TX_DP_PREPROC_MODE2 - I PFIR only mode,
                                                       TX_DP_PREPROC_MODE3 - Cascaded I and Q PFIR mode
                                                       STATIC, not configurable on the fly, BBIC to determine based on usecase */

    adi_adrv9001_PfirBank_e txPreProcWbNbPfirIBankSel; /*!< Config of Preproc PFIR I: Block #3, TPFIR_I, Dynamic, configurable during dynamic profile switching */
    adi_adrv9001_PfirBank_e txPreProcWbNbPfirQBankSel; /*!< Config of Preproc PFIR Q: Block #5, TPFIR_Q, Dynamic, configurable during dynamic profile switching */
} adi_adrv9001_TxPreProc_t;

/**
 * \brief Data structure to hold ADRV9001 Tx Narrowband interpolation top block
 */
typedef struct adi_adrv9001_TxNbIntTop
{
    uint8_t txInterpBy2Blk20En;   /*!< Tx Interpolator2 Narrowband Top block 20. Bit field tx_dp_int2_20_en */
    uint8_t txInterpBy2Blk18En;   /*!< Tx Interpolator3 Narrowband Top block 18. Bit field tx_dp_int2_18_en */
    uint8_t txInterpBy2Blk16En;   /*!< Tx Interpolator2 Narrowband Top block 16. Bit field tx_dp_int2_16_en */
    uint8_t txInterpBy2Blk14En;   /*!< Tx Interpolator2 Narrowband Top block 14. Bit field tx_dp_int2_14_en */
    uint8_t txInterpBy2Blk12En;   /*!< Tx Interpolator2 Narrowband Top block 12. Bit field tx_dp_int2_12_en */
    uint8_t txInterpBy3Blk10En;   /*!< Tx Interpolator3 Narrowband Top block 10. Bit field tx_dp_int3_10_en */
    uint8_t txInterpBy2Blk8En;    /*!< Tx Interpolator2 Narrowband Top block 8. Bit field tx_dp_int2_8_en */

    uint8_t txScicBlk32En;        /*!< Tx TSCIC Narrowband Top block 32. Bit field tx_dp_tscic_32_en */
    uint8_t txScicBlk32DivFactor; /*!< Tx TSCIC Narrowband Top block 32. Bit field tx_dp_tscic_32_div_factor */
} adi_adrv9001_TxNbIntTop_t;

/**
 * \brief Data structure to hold ADRV9001 Tx Wideband interpolation top block
 */
typedef struct adi_adrv9001_TxWbIntTop
{
    uint8_t txInterpBy2Blk30En; /*!< Tx Interpolator2 Wideband Top block 30. Bit field tx_dp_int2_30_en */
    uint8_t txInterpBy2Blk28En; /*!< Tx Interpolator2 Wideband Top block 28. Bit field tx_dp_int2_28_en */
    uint8_t txInterpBy2Blk26En; /*!< Tx Interpolator2 Wideband Top block 26. Bit field tx_dp_int2_26_en */
    uint8_t txInterpBy2Blk24En; /*!< Tx Interpolator2 Wideband Top block 24. Bit field tx_dp_int2_24_en*/
    uint8_t txInterpBy2Blk22En; /*!< Tx Interpolator2 Wideband Top block 22. Bit field tx_dp_int2_22_en*/
    uint8_t txWbLpfBlk22p1En;   /*!< Tx Wideband LPF Wideband Top block 22.1. Bit field tx_dp_int2_22_1_en */
} adi_adrv9001_TxWbIntTop_t;

/**
 * \brief Data structure to hold ADRV9001 Tx interpolation top block
 */
typedef struct adi_adrv9001_TxIntTop
{
    uint8_t interpBy3Blk44p1En; /*!< Tx Interpolator3 Top block 44.1. Bit field tx_dp_int3_44_1_en */
    uint8_t sinc3Blk44En;       /*!< Tx Sinc3 Top block 44. Bit field tx_dp_sinc3_44_en*/
    uint8_t sinc2Blk42En;       /*!< Tx Sinc2 Top block 42. Bit field tx_dp_int2_42_en*/
    uint8_t interpBy3Blk40En;   /*!< Tx Interpolator3 Top block 40. Bit field tx_dp_int3_40_en*/
    uint8_t interpBy2Blk38En;   /*!< Tx Interpolator2 Top block 38. Bit field tx_dp_int2_38_en */
    uint8_t interpBy2Blk36En;   /*!< Tx Interpolator2 Top block 36. Bit field tx_dp_int2_36_en*/
} adi_adrv9001_TxIntTop_t;

/**
 * \brief Data structure to hold ADRV9001 TX Interpolation Top frequency device mapper block
 */
typedef struct adi_adrv9001_TxIntTopFreqDevMap
{
    /* Frequency dev mapper */
    uint32_t rrc2Frac;		/*!< Frequency Deviation */
    uint32_t mpll;			/*!< Frequency fraction multiplier */
    uint32_t nchLsw;		/*!< Frequency offset. Contains least significant word, i.e Bit[0:31] */
    uint8_t  nchMsb;		/*!< Frequency offset. Contains most significant 3 bits, i.e Bit[32:34] */
    uint8_t  freqDevMapEn;	/*!< Enable/disable block #47 Freq_Dev Mapper. */
    uint8_t  txRoundEn;		/*!< Enable/disable tx round block #46. */
} adi_adrv9001_TxIntTopFreqDevMap_t;

/**
 * \brief Data structure to hold ADRV9001 TX IQDM block
 */
typedef struct adi_adrv9001_TxIqdmDuc
{
    /* Tx IQ DM DUC config parameters that support three modes of operation */
    adi_adrv9001_TxDpIqDmDucMode_e iqdmDucMode; /*!< TX_DP_IQDMDUC_MODE0 - bypass IQ in IQ out, TX_DP_IQDMDUC_MODE1 - IQ DM, TX_DP_IQDMDUC_MODE2 - DUC */

    /* Parameters for TX_DP_IQDMDUC_MODE1 IqDm mode */
    uint32_t iqdmDev;       /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmDevOffset; /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmScalar;    /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmThreshold; /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */

    adi_adrv9001_NcoDpConfig_t iqdmNco; /*!< Parameters for IQDM NCO */
} adi_adrv9001_TxIqdmDuc_t;

/**
 * \brief Data structure to hold ADRV9001 TX Channel profile data
 */
typedef struct adi_adrv9001_TxDpProfile
{
    adi_adrv9001_TxPreProc_t           txPreProc;          /*!< Tx pre processor block (Static Settings) */
    adi_adrv9001_TxWbIntTop_t          txWbIntTop;         /*!< txwb_int_top (Wideband Top) block interpolater configs (Dynamic Settings)*/
    adi_adrv9001_TxNbIntTop_t          txNbIntTop;         /*!< txnb_int_top (Narrowband Top) block interpolater configs (Dynamic Settings) */
    adi_adrv9001_TxIntTop_t            txIntTop;           /*!< tx_int_top (TOP) block interpolater configs (Dynamic Settings) */
    adi_adrv9001_TxIntTopFreqDevMap_t  txIntTopFreqDevMap; /*!< tx_int_top (TOP) block frequency dev map configs (Static Settings) */
    adi_adrv9001_TxIqdmDuc_t           txIqdmDuc;          /*!< tx_iqdm_duc block configs (Static Settings) */
} adi_adrv9001_TxDpProfile_t;

/**
 *  \brief Data structure to hold ADRV9001 Tx Slew Rate Limiter Configuration
 */
typedef struct adi_adrv9001_SlewRateLimiterCfg
{
    bool srlEnable;                                         /*!< default = true */
    bool srlStatisticsEnable;                               /*!< default = true */
    adi_adrv9001_SrlTableSel_e srlTableSelect;              /*!< default = adi_adrv9001_srl_table3 */
    uint8_t srlSlewOffset;                                  /*!< 0 (default), 1, . . ., 15 */
    adi_adrv9001_SrlStatisticsMode_e srlStatisticsMode;     /*!< default = adi_adrv9001_srl_statistics_clear_to_zero */
} adi_adrv9001_SlewRateLimiterCfg_t;

/**
 *  \brief Data structure to hold ADRV9001 Tx PA ramp configuration
 */
typedef struct adi_adrv9001_PaRampCfg
{
    bool     enable;                                   /*!< PA Ramp enable; True = Enable PA ramp of Tx channel; False = Disable */
    bool     gpioTriggerSelect;                        /*!< Source of the enable signal. True: GPIO triggered; False: SPI triggered */
    uint16_t rampClock_kHz;                            /*!< Clock rate of Tx PA Ramp block */
    uint32_t triggerDelayRise;                         /*!< Programmable Delay Enable on the rising edge */
    uint32_t triggerDelayFall;                         /*!< Programmable Delay Enable on the falling edge */
    adi_adrv9001_GpioPinSel_e     gpioSource;          /*!< Desired GPIO pin to be used as source of trigger if gpioTriggered = True */
    uint8_t  upEndIndex;                               /*!< 8-bit look-up-table index. This index indicates the end of the ramp up waveform. */
    bool     asymmetricRamp;                           /*!< False = symmetric, True = Ramp-down waveform is asymmetric to the Ramp-up waveform */
    uint8_t  downEndIndex;                             /*!< 8-bit look-up-table index. This index indicates the start of the ramp down waveform. 
                                                          Valid only when asymmetricRamp=1 */
    adi_adrv9001_AuxDacs_e  auxDacChannelSelect;       /*!< Choose the AuxDacChannel [0, 1, 2, 3] to ouptut the Ramp or SPI signal */
    uint16_t paRampLUT[ADRV9001_TX_PA_RAMP_LUT_SIZE];  /*!< PA Ramp look-up-table. 256 depth Array of LUT elements */
} adi_adrv9001_PaRampCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_TX_TYPES_H_ */
