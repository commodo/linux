/**
* \file
* \brief Contains AGC features related function implementation defined in
* adi_adrv9001_agc.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_rx_agc.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"

#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"

#include "adrv9001_bf_analog_rx_mem_map.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_rx.h"
#include "adrv9001_bf_nvs_regmap_rxb.h"
#include "adrv9001_decode_bf_enum.h"

#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_agc.c"
#endif

/* Forward declaration */
static int32_t Agc_MinMaxGainIndex_Set_Validate(adi_adrv9001_Device_t *device,
                                                adi_common_ChannelNumber_e channel,
                                                uint8_t minGainIndex,
                                                uint8_t maxGainIndex);


static int32_t adi_adrv9001_AgcCfgValidate(adi_adrv9001_Device_t *device,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_AgcCfg_t *agcCfg)
{
    static const uint8_t PEAK_WAIT_TIME_MAX = 0x1F;
    static const uint32_t GAIN_UPDATE_COUNTER_MAX = 0x003FFFFF;
    static const uint8_t UNDER_RANGE_HIGH_POWER_THRESH_MAX = 0x7F;
    static const uint8_t UNDER_RANGE_LOW_POWER_THRESH_MAX = 0x1F;
    static const uint8_t UNDER_RANGE_HIGH_POWER_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t UNDER_RANGE_LOW_POWER_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t POWER_MEASUREMENT_DURATION_MAX = 0x1F;
    static const uint8_t SLOW_LOOP_SETTLING_DELAY_MAX = 0x7F;
    static const uint8_t APD_HIGH_THRESH_MAX = 0x3F;
    static const uint8_t APD_LOW_THRESH_MAX = 0x3F;
    static const uint8_t APD_LOW_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t APD_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_OVER_LOAD_DURATION_CNT_MAX = 0x07;
    static const uint8_t HB_OVER_LOAD_THRESH_CNT_MAX = 0x0F;
    static const uint8_t HB_GAIN_STEP_HIGH_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_LOW_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_MID_RECOVERY_MAX = 0x1F;
    static const uint8_t OVER_RANGE_HIGH_POWER_THRESH_MID_RECOVERY_MAX = 0x7F;
    static const uint8_t OVER_RANGE_LOW_POWER_THRESH_MID_RECOVERY_MAX = 0x0F;
    static const uint8_t OVER_RANGE_HIGH_POWER_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t OVER_RANGE_LOW_POWER_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t CHANGE_GAIN_IF_THRESH_HIGH_MAX = 0x03;
    static const uint8_t ATTACK_DELAY_MAX = 0x3F;
    static const uint8_t UNDER_RANGE_MID_INTERVAL_MAX = 0x3F;
    static const uint8_t UNDER_RANGE_HIGH_INTERVAL_MAX = 0x3F;
    static const uint8_t ENABLE_MAX = 0x01;

    ADI_NULL_PTR_RETURN(&device->common, agcCfg);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_EXPECT(Agc_MinMaxGainIndex_Set_Validate, device, channel, agcCfg->minGainIndex, agcCfg->maxGainIndex);
    ADI_RANGE_CHECK(device, agcCfg->peakWaitTime,           0, PEAK_WAIT_TIME_MAX);
    ADI_RANGE_CHECK(device, agcCfg->gainUpdateCounter,      0, GAIN_UPDATE_COUNTER_MAX);
    ADI_RANGE_CHECK(device, agcCfg->attackDelay_us,         0, ATTACK_DELAY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->slowLoopSettlingDelay,  0, SLOW_LOOP_SETTLING_DELAY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->changeGainIfThreshHigh, 0, CHANGE_GAIN_IF_THRESH_HIGH_MAX);

    /* Power Configuration register */
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeHighPowerThresh,           0, UNDER_RANGE_HIGH_POWER_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeLowPowerThresh,            0, UNDER_RANGE_LOW_POWER_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeHighPowerGainStepRecovery, 0, UNDER_RANGE_HIGH_POWER_GAIN_STEP_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeLowPowerGainStepRecovery,  0, UNDER_RANGE_LOW_POWER_GAIN_STEP_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.powerMeasurementDuration,            0, POWER_MEASUREMENT_DURATION_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeHighPowerThresh,            0, OVER_RANGE_HIGH_POWER_THRESH_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeLowPowerThresh,             0, OVER_RANGE_LOW_POWER_THRESH_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeHighPowerGainStepAttack,    0, OVER_RANGE_HIGH_POWER_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeLowPowerGainStepAttack,     0, OVER_RANGE_LOW_POWER_GAIN_STEP_ATTACK_MAX);

    /* Agc Peak */
    ADI_RANGE_CHECK(device, agcCfg->peak.agcUnderRangeMidInterval,  0, UNDER_RANGE_MID_INTERVAL_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.agcUnderRangeHighInterval, 0, UNDER_RANGE_HIGH_INTERVAL_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdHighThresh,             0, APD_HIGH_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdLowThresh,              0, APD_LOW_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdGainStepAttack,         0, APD_LOW_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdGainStepRecovery,       0, APD_GAIN_STEP_RECOVERY_MAX);

    /* HB Configuration */
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadDurationCnt,  0, HB_OVER_LOAD_DURATION_CNT_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadThreshCnt,    0, HB_OVER_LOAD_THRESH_CNT_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepHighRecovery, 0, HB_GAIN_STEP_HIGH_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepLowRecovery,  0, HB_GAIN_STEP_LOW_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepMidRecovery,  0, HB_GAIN_STEP_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepAttack,       0, HB_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadPowerMode,    0, ENABLE_MAX);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Agc_Configure(adi_adrv9001_Device_t *device,
                                      adi_common_ChannelNumber_e channel,
                                      adi_adrv9001_AgcCfg_t *agcCfg)
{
    static const uint8_t HB_THRESH_CONFIG = 0x1;
    static const uint8_t POWER_INPUT_SELECT = 0x1;
    static const uint8_t POWER_LOG_SHIFT = 0x1;

    adrv9001_BfNvsRegmapRxChanAddr_e rxAddr = ADRV9001_BF_RX1_CORE;
    adrv9001_BfNvsRegmapRxbChanAddr_e rxbAddr = ADRV9001_BF_RXB1_CORE;
    adrv9001_BfAnalogRxMemMapChanAddr_e anaAddr = ADRV9001_BF_RX1_ANA;
    uint8_t bfValue = 0;
    uint8_t regId = 0;

    /* apdHighThresh and apdLowThresh have to be written to 7 registers */
    static uint8_t REGISTER_ID = 7;

    ADI_PERFORM_VALIDATION(adi_adrv9001_AgcCfgValidate, device, channel, agcCfg);

    rxAddr = adrv9001_RxChanAddr_Get(channel);
    rxbAddr = adrv9001_RxbChanAddr_Get(channel);
    anaAddr = adrv9001_AnalogRxChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfSet,           device, rxbAddr, agcCfg->peakWaitTime);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfSet,       device, rxbAddr, agcCfg->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfSet,       device, rxbAddr, agcCfg->minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfSet,      device, rxbAddr, agcCfg->gainUpdateCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAttackDelayBfSet,            device, rxbAddr, agcCfg->attackDelay_us);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfSet, device, rxbAddr, agcCfg->enableFastRecoveryLoop);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfSet,   device, rxbAddr, agcCfg->lowThreshPreventGainInc);

    bfValue = agcCfg->changeGainIfThreshHigh & 0x01;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfSet, device, rxbAddr, bfValue);
    bfValue = (agcCfg->changeGainIfThreshHigh >> 1) & 0x01;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfSet, device, rxbAddr, bfValue);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfSet,  device, rxbAddr, agcCfg->peakThreshGainControlMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonBfSet,                   device, rxbAddr, agcCfg->resetOnRxon);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfSet,          device, rxbAddr, agcCfg->resetOnRxonGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfSet, device, rxbAddr, agcCfg->enableSyncPulseForGainCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfSet,         device, rxbAddr, agcCfg->slowLoopSettlingDelay);

    /* Agc Peak */
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval0BfSet,     device, rxbAddr, agcCfg->peak.agcUnderRangeLowInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfSet, device, rxbAddr, agcCfg->peak.agcUnderRangeMidInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfSet, device, rxbAddr, agcCfg->peak.agcUnderRangeHighInterval);

    for (regId = 0; regId < REGISTER_ID; regId++)
    {
        ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfSet, device, anaAddr, regId, agcCfg->peak.apdHighThresh);
        ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfSet, device, anaAddr, regId, agcCfg->peak.apdLowThresh);
    }

    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetDecayBfSet,  device, anaAddr, 0);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfSet, device, anaAddr, 1);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfSet, device, anaAddr, 0);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfSet, device, rxbAddr, agcCfg->peak.apdUpperThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfSet, device, rxbAddr, agcCfg->peak.apdLowerThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbGainStepBfSet,                 device, rxbAddr, agcCfg->peak.apdGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbGainStepBfSet,                 device, rxbAddr, agcCfg->peak.apdGainStepRecovery);

    /*HB Configuration register*/
    ADI_EXPECT(adrv9001_NvsRegmapRxRxEnableDecOverloadBfSet,         device, rxAddr, agcCfg->peak.enableHbOverload);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfSet,  device, rxAddr, agcCfg->peak.hbOverloadDurationCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfSet, device, rxAddr, agcCfg->peak.hbOverloadThreshCnt);

    /* HB */
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfSet,     device, rxAddr, agcCfg->peak.hbHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfSet, device, rxAddr, agcCfg->peak.hbUnderRangeLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfSet, device, rxAddr, agcCfg->peak.hbUnderRangeMidThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfSet,     device, rxAddr, agcCfg->peak.hbUnderRangeHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfSet,            device, rxbAddr, agcCfg->peak.hbUpperThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfSet,             device, rxbAddr, agcCfg->peak.hbUnderRangeHighThreshExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfSet,                       device, rxbAddr, agcCfg->peak.hbGainStepHighRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfSet,                   device, rxbAddr, agcCfg->peak.hbGainStepLowRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfSet,                   device, rxbAddr, agcCfg->peak.hbGainStepMidRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfSet,                      device, rxbAddr, agcCfg->peak.hbGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfSet,                    device, rxAddr, agcCfg->peak.hbOverloadPowerMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecThresholdConfigBfSet,                      device, rxAddr, HB_THRESH_CONFIG);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfSet,                 device, rxbAddr, agcCfg->peak.hbUnderRangeMidThreshExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfSet,                 device, rxbAddr, agcCfg->peak.hbUnderRangeLowThreshExceededCnt);

    /* APD Low Frequency MITIGATION Mode Setup */
    static const uint8_t APD_LOW_FREQ_ADCOVRG_2ND_HIGH_COUNTER = 3;
    static const uint8_t APD_LOW_FREQ_ERROR_MITIGATION_MODE = 1;
    static const uint16_t APD_LOW_FREQ_THRESH_OFFSET = 6;

    bfValue = agcCfg->peak.hbHighThresh - APD_LOW_FREQ_THRESH_OFFSET;
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecimatedDataOverloadSecondaryUpperThresholdBfSet, device, rxbAddr, bfValue);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAdcovrg2ndHighCounterBfSet,            device, rxbAddr, APD_LOW_FREQ_ADCOVRG_2ND_HIGH_COUNTER);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcApdLowFreqErrorMitigationModeBfSet, device, rxbAddr, APD_LOW_FREQ_ERROR_MITIGATION_MODE);

    /* Power Configuration register */
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerEnableMeasBfSet,                 device, rxbAddr, agcCfg->power.powerEnableMeasurement);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerInputSelectBfSet,                device, rxbAddr, POWER_INPUT_SELECT);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdBfSet,                 device, rxbAddr, agcCfg->power.underRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdBfSet,                 device, rxbAddr, agcCfg->power.underRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.underRangeHighPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.underRangeLowPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfSet,        device, rxbAddr, agcCfg->power.powerMeasurementDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerWaitDelayBfSet,                  device, rxbAddr, agcCfg->power.powerMeasurementDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfSet,          device, rxbAddr, agcCfg->power.rxTddPowerMeasDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfSet,             device, rxbAddr, agcCfg->power.rxTddPowerMeasDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLockLevelBfSet,                       device, rxbAddr, agcCfg->power.overRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfSet,                 device, rxbAddr, agcCfg->power.overRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerLogShiftBfSet,                   device, rxbAddr, POWER_LOG_SHIFT);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.overRangeHighPowerGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.overRangeLowPowerGainStepAttack);

    /* External LNA */
    ADI_EXPECT(adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfSet, device, rxbAddr, agcCfg->extLna.settlingDelay);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_Agc_Inspect_Validate(adi_adrv9001_Device_t *device,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_AgcCfg_t *agcCfg)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, agcCfg);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Agc_Inspect(adi_adrv9001_Device_t *device,
                                    adi_common_ChannelNumber_e channel,
                                    adi_adrv9001_AgcCfg_t *agcCfg)
{
    adrv9001_BfNvsRegmapRxChanAddr_e rxAddr = ADRV9001_BF_RX1_CORE;
    adrv9001_BfNvsRegmapRxbChanAddr_e rxbAddr = ADRV9001_BF_RXB1_CORE;
    adrv9001_BfAnalogRxMemMapChanAddr_e anaAddr = ADRV9001_BF_RX1_ANA;
    uint8_t bfValue = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Agc_Inspect_Validate, device, channel, agcCfg);

    rxAddr = adrv9001_RxChanAddr_Get(channel);
    rxbAddr = adrv9001_RxbChanAddr_Get(channel);
    anaAddr = adrv9001_AnalogRxChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfGet,      device, rxbAddr, &agcCfg->peakWaitTime);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfGet,  device, rxbAddr, &agcCfg->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfGet,  device, rxbAddr, &agcCfg->minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfGet, device, rxbAddr, &agcCfg->gainUpdateCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAttackDelayBfGet,       device, rxbAddr, &agcCfg->attackDelay_us);

    /* AGC Control register - Slowloop_config*/
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfGet, device, rxbAddr, &bfValue);
    agcCfg->enableFastRecoveryLoop = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfGet, device, rxbAddr, &bfValue);
    agcCfg->lowThreshPreventGainInc = bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfGet, device, rxbAddr, &bfValue);
    agcCfg->changeGainIfThreshHigh = (bfValue & 0x01);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfGet, device, rxbAddr, &bfValue);
    agcCfg->changeGainIfThreshHigh |= ((bfValue << 1) & 0x02);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfGet, device, rxbAddr, &bfValue);
    agcCfg->peakThreshGainControlMode = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonBfGet, device, rxbAddr, &bfValue);
    agcCfg->resetOnRxon = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfGet, device, rxbAddr, &agcCfg->resetOnRxonGainIndex);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfGet, device, rxbAddr, &bfValue);
    agcCfg->enableSyncPulseForGainCounter = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfGet, device, rxbAddr, &agcCfg->slowLoopSettlingDelay);

    /* Agc Peak */
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval0BfGet,             device, rxbAddr, &agcCfg->peak.agcUnderRangeLowInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfGet,         device, rxbAddr, &agcCfg->peak.agcUnderRangeMidInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfGet,         device, rxbAddr, &agcCfg->peak.agcUnderRangeHighInterval);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfGet,             device, anaAddr, 0, &agcCfg->peak.apdHighThresh);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfGet,             device, anaAddr, 0, &agcCfg->peak.apdLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfGet, device, rxbAddr, &agcCfg->peak.apdUpperThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfGet, device, rxbAddr, &agcCfg->peak.apdLowerThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbGainStepBfGet,                 device, rxbAddr, &agcCfg->peak.apdGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbGainStepBfGet,                 device, rxbAddr, &agcCfg->peak.apdGainStepRecovery);

    /*HB Configuration register*/
    ADI_EXPECT(adrv9001_NvsRegmapRxRxEnableDecOverloadBfGet, device, rxAddr, &bfValue);
    agcCfg->peak.enableHbOverload = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfGet,                device, rxAddr,  &agcCfg->peak.hbOverloadDurationCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfGet,               device, rxAddr,  &agcCfg->peak.hbOverloadThreshCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfGet,     device, rxAddr,  &agcCfg->peak.hbHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfGet, device, rxAddr,  &agcCfg->peak.hbUnderRangeLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfGet, device, rxAddr,  &agcCfg->peak.hbUnderRangeMidThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfGet,     device, rxAddr,  &agcCfg->peak.hbUnderRangeHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfGet,            device, rxbAddr, &agcCfg->peak.hbUpperThreshPeakExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfGet,             device, rxbAddr, &agcCfg->peak.hbUnderRangeHighThreshExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfGet,                       device, rxbAddr, &agcCfg->peak.hbGainStepHighRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfGet,                   device, rxbAddr, &agcCfg->peak.hbGainStepLowRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfGet,                   device, rxbAddr, &agcCfg->peak.hbGainStepMidRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfGet,                      device, rxbAddr, &agcCfg->peak.hbGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfGet,                    device, rxAddr,  &agcCfg->peak.hbOverloadPowerMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfGet,                 device, rxbAddr, &agcCfg->peak.hbUnderRangeMidThreshExceededCnt);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfGet,                 device, rxbAddr, &agcCfg->peak.hbUnderRangeLowThreshExceededCnt);

    /* Power Configuration register */
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerEnableMeasBfGet, device, rxbAddr, &bfValue);
    agcCfg->power.powerEnableMeasurement = (bool)bfValue;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.underRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.underRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.underRangeHighPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.underRangeLowPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfGet,        device, rxbAddr, &agcCfg->power.powerMeasurementDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerWaitDelayBfGet,                  device, rxbAddr, &agcCfg->power.powerMeasurementDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfGet,          device, rxbAddr, &agcCfg->power.rxTddPowerMeasDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfGet,             device, rxbAddr, &agcCfg->power.rxTddPowerMeasDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLockLevelBfGet,                       device, rxbAddr, &agcCfg->power.overRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.overRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.overRangeHighPowerGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.overRangeLowPowerGainStepAttack);

    /* External LNA */
    ADI_EXPECT(adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfGet, device, rxbAddr, &agcCfg->extLna.settlingDelay);

    ADI_API_RETURN(device);
}

static int32_t Agc_MinMaxGainIndex_Set_Validate(adi_adrv9001_Device_t *device,
                                                adi_common_ChannelNumber_e channel,
                                                uint8_t minGainIndex,
                                                uint8_t maxGainIndex)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    /* Ensure that Max Gain Index is always greater than Min Gain Index */
    if (minGainIndex >= maxGainIndex)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         maxGainIndex,
                         "Min Gain Index cannot be greater than Max Gain Index");
        ADI_API_RETURN(device);
    }

    /* Ensure that desired limits are within the programmed gain table */
    /* TODO: Remove the fields from devStateInfo - requires reading and parsing gain table or storing values in firmware */
    if (channel == ADI_CHANNEL_1)
    {
        /* range check the gain against the max and min expected values */
        if ((maxGainIndex > device->devStateInfo.gainIndexes.rx1MaxGainIndex) ||
            (minGainIndex < device->devStateInfo.gainIndexes.rx1MinGainIndex))
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             maxGainIndex,
                             "Gain index less than minimum index or greater than maximum gain index");
            ADI_API_RETURN(device);
        }
    }

    if (channel == ADI_CHANNEL_2)
    {
        /* range check the gain against the max and min expected values */
        if ((maxGainIndex > device->devStateInfo.gainIndexes.rx2MaxGainIndex) ||
            (minGainIndex < device->devStateInfo.gainIndexes.rx2MinGainIndex))
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             maxGainIndex,
                             "Gain index less than minimum index or greater than maximum gain index");
            ADI_API_RETURN(device);
        }
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Agc_MinMaxGainIndex_Set(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               uint8_t minGainIndex,
                                               uint8_t maxGainIndex)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(Agc_MinMaxGainIndex_Set_Validate, device, channel, minGainIndex, maxGainIndex);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfSet, device, baseAddr, minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfSet, device, baseAddr, maxGainIndex);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_Agc_MinMaxGainIndex_Get_Validate(adi_adrv9001_Device_t *device,
                                                               adi_common_ChannelNumber_e channel,
                                                               uint8_t *minGainIndex,
                                                               uint8_t *maxGainIndex)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, minGainIndex);
    ADI_NULL_PTR_RETURN(&device->common, maxGainIndex);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Agc_MinMaxGainIndex_Get(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               uint8_t *minGainIndex,
                                               uint8_t *maxGainIndex)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Agc_MinMaxGainIndex_Get_Validate, device, channel, minGainIndex, maxGainIndex);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfGet, device, baseAddr, maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfGet, device, baseAddr, minGainIndex);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_Agc_Reset_Validate(adi_adrv9001_Device_t *device, adi_common_ChannelNumber_e channel)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Agc_Reset(adi_adrv9001_Device_t *device, adi_common_ChannelNumber_e channel)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Agc_Reset_Validate, device, channel);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSoftResetBfSet, device, baseAddr, true);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSoftResetBfSet, device, baseAddr, false);

    ADI_API_RETURN(device);
}

