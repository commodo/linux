/**
* \file
* \brief Contains ADRV9001 API AGC data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_AGC_TYPES_H_
#define _ADI_ADRV9001_AGC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
*  \brief Data structure to hold AGC peak settings
*  The evaluation software GUI for the product can be used to generate a structure with suggested settings.
*/
typedef struct adi_adrv9001_AgcPeak
{
    uint16_t    agcUnderRangeLowInterval;           /*!< Update interval for AGC loop mode in AGC clock cycles. Valid range is 0 to 65535 */
    uint8_t     agcUnderRangeMidInterval;           /*!< 2nd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeMidInterval+1)*agcUnderRangeLowInterval_ns. Valid range is 0 to 63 */
    uint8_t     agcUnderRangeHighInterval;          /*!< 3rd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeHighInterval+1)*2nd update interval. Valid range is 0 to 63 */
    uint8_t     apdHighThresh;                      /*!< AGC APD high threshold. Valid range is 0 to 63 */
    uint8_t     apdLowThresh;                       /*!< AGC APD low threshold. Valid range is 0 to 63. Recommended to be 3dB below apdHighThresh */
    uint8_t     apdUpperThreshPeakExceededCnt;      /*!< AGC APD peak detect upper threshold count. Valid range is 0 to 255 */
    uint8_t     apdLowerThreshPeakExceededCnt;      /*!< AGC APD peak detect lower threshold count. Valid range is 0 to 255 */
    uint8_t     apdGainStepAttack;                  /*!< AGC APD peak detect attack gain step. Valid range is 0 to 31 */
    uint8_t     apdGainStepRecovery;                /*!< AGC APD gain index step size for recovery. Valid range is 0 to 31 */
    bool        enableHbOverload;                   /*!< Enable or disables the HB overload detector. */
    uint8_t     hbOverloadDurationCnt;              /*!< Sets the window of clock cycles (at the HB output rate) to meet the overload count. (0 = 2 cycles, 1 = 4 cycles, 2 = 8 cycles, 3 = 12 cycles, 4 = 16 cycles, 5 = 24 cycles, 6 = 32 cycles) */
    uint8_t     hbOverloadThreshCnt;                /*!< Sets the number of actual overloads required to trigger the overload signal. Valid range from 1 to 15 */
    uint8_t     hbHighThresh;                       /*!< AGC HB output high threshold. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeLowThresh;              /*!< AGC HB output low threshold. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeMidThresh;              /*!< AGC HB output low threshold for 2nd interval for multiple time constant AGC mode. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeHighThresh;             /*!< AGC HB output low threshold for 3rd interval for multiple time constant AGC mode. Valid range from  0 to 255 */
    uint8_t     hbUpperThreshPeakExceededCnt;       /*!< AGC HB output upper threshold count. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeHighThreshExceededCnt;  /*!< AGC HB output lower threshold count. Valid range from  0 to 255 */
    uint8_t     hbGainStepHighRecovery;             /*!< AGC HB gain index step size. Valid range from  0 to 31 */
    uint8_t     hbGainStepLowRecovery;              /*!< AGC HB gain index step size, when the HB Low Overrange interval 2 triggers. Valid range from  0 to 31 */
    uint8_t     hbGainStepMidRecovery;              /*!< AGC HB gain index step size, when the HB Low Overrange interval 3 triggers. Valid range from  0 to 31 */
    uint8_t     hbGainStepAttack;                   /*!< AGC HB output attack gain step. Valid range from  0 to 31 */
    uint8_t     hbOverloadPowerMode;                /*!< Select magnitude measurements or power mearurements. 0 = enable peak mode 1 = enable I2 + Q2 power mode */
    uint8_t     hbUnderRangeMidThreshExceededCnt;   /*!< AGC HB output upper threshold count. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeLowThreshExceededCnt;   /*!< AGC HB output lower threshold count. Valid range from  0 to 255 */
} adi_adrv9001_AgcPeak_t;

/**
* \brief Data structure to hold AGC power settings
*  The evaluation software GUI for the product can be used to generate a structure with suggested settings.
*/
typedef struct adi_adrv9001_AgcPower
{
    bool        powerEnableMeasurement;                 /*!< Enable the Rx power measurement block */
    uint8_t     underRangeHighPowerThresh;              /*!< AGC power measurement detect lower 0 threshold. Valid Range from 0 to 127. */
    uint8_t     underRangeLowPowerThresh;               /*!< AGC power measurement detect lower 1 threshold. Valid offset from 0 to 31 */
    uint8_t     underRangeHighPowerGainStepRecovery;    /*!< AGC power measurement detect lower 0 recovery gain step. Valid range from  0 to 31 */
    uint8_t     underRangeLowPowerGainStepRecovery;     /*!< AGC power measurement detect lower 1 recovery gain step. Valid range from  0 to 31 */
    uint8_t     powerMeasurementDuration;               /*!< Average power measurement duration = 8*2^powerMeasurementDuration. Valid range from 0 to 31 */
    uint8_t     powerMeasurementDelay;                  /*!< Average power measurement delay between sucessive measurement. Valid range from 0 to 255 */
    uint16_t    rxTddPowerMeasDuration;                 /*!< Measurement duration to detect power for specific slice of the gain update counter. */
    uint16_t    rxTddPowerMeasDelay;                    /*!< Measurement delay to detect power for specific slice of the gain update counter. */
    uint8_t     overRangeHighPowerThresh;               /*!< AGC upper 0 (overRangeHighPowerThreshold) threshold for power measurement. Valid Range from 0 to 127.*/
    uint8_t     overRangeLowPowerThresh;                /*!< AGC upper 1 (overRangeLowPowerThreshold)  threshold for power measurement. Valid offset from 0 to 15 */
    uint8_t     overRangeHighPowerGainStepAttack;       /*!< AGC power measurement detect lower 0 attack gain step. Valid range from  0 to 31 */
    uint8_t     overRangeLowPowerGainStepAttack;        /*!< AGC power measurement detect lower 1 attack gain step. Valid range from  0 to 31 */
} adi_adrv9001_AgcPower_t;

/**
* \brief Data structure to hold all Ext LNA settings
*  The evaluation software GUI for the product can be used to generate a structure with suggested settings.
*/
typedef struct adi_adrv9001_ExtLna
{
    uint8_t     gpio;                               /*!< TBD */
    uint8_t     powerDown;                          /*!< TBD */
    uint8_t     settlingDelay;                      /*!< External LNA Settling Delay. Valid range is from 0 to 255 */
} adi_adrv9001_ExtLna_t;
    
/**
* \brief Data structure to hold all AGC configuration settings for initialization
*  The evaluation software GUI for the product can be used to generate a structure with suggested settings.
*/
typedef struct adi_adrv9001_AgcCfg
{
    uint8_t     peakWaitTime;                       /*!< AGC peak wait time. Valid range is from 0 to 31 */
    uint8_t     maxGainIndex;                       /*!< AGC Rx max gain index. Valid range is from minGainIndex to 255 */
    uint8_t     minGainIndex;                       /*!< AGC Rx min gain index. Valid range is from 0 to maxGainIndex */
    uint32_t    gainUpdateCounter;                  /*!< AGC gain update time denominated in AGC clock cycles. Valid range is from 0 to 4194303*/
    uint8_t     attackDelay_us;                     /*!< Delay time, denominated in microseconds, before starting AGC, after starting Rx */
    uint8_t     slowLoopSettlingDelay;              /*!< On any gain change, the AGC waits for the time (range 0 to 127) specified in AGC clock cycles to allow gain transients to flow through the Rx path before starting any measurements. */
    bool        lowThreshPreventGainInc;            /*!< Prevent gain index from incrementing while peak thresholds are exceeded */
    uint8_t     changeGainIfThreshHigh;             /*!< Enable immediate gain change if high threshold counter is exceeded. Bit 0 enables ULB high threshold, Bit 1 enables HB high threshold */
    bool        peakThreshGainControlMode;          /*!< Enable gain change based only on the signal peak threshold over-ranges. Power-based AGC changes are disabled in this mode. */
    bool        resetOnRxon;                        /*!< Reset the AGC slow loop state machine to max gain when the Rx Enable is taken low */
    uint8_t     resetOnRxonGainIndex;               /*!< AGC Reset On gain index. Valid range is from 0 to 255 */
    bool        enableSyncPulseForGainCounter;      /*!< Enable the AGC gain update counter to be sync'ed to a time-slot boundary. */
    bool        enableFastRecoveryLoop;             /*!< Enable multiple time constants in AGC loop for fast attack and fast recovery. */
 
    adi_adrv9001_AgcPower_t power;
    adi_adrv9001_AgcPeak_t  peak;
    adi_adrv9001_ExtLna_t   extLna;
} adi_adrv9001_AgcCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_AGC_TYPES_H_ */
