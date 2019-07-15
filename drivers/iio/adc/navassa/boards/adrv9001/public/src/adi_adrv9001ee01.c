/**
* \file
* \brief Core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2018 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include <stdlib.h>
#include <string.h>

#include "adi_adrv9001ee01.h"
#include "adi_adrv9001ee01_mcs.h"

#include "adrv9001ee01.h"

#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_utilities.h"

#include "adrv9001_init.h"

int32_t adi_adrv9001Ee01_Create(adi_adrv9001Ee01_Board_t *adrv9001Ee01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    /* Initialize spi setting for the EE01 board type */
    adi_adrv9001_SpiSettings_t spiSettings = {
        .msbFirst = 1,
        .enSpiStreaming = 0,
        .autoIncAddrUp = 1,
        .fourWireMode = 1,
        .cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
    };
    
    if (adrv9001Ee01 == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->adrv9001Device = (adi_adrv9001_Device_t*)calloc(1, sizeof(adi_adrv9001_Device_t));
    if (adrv9001Ee01->adrv9001Device == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->fpga9001Device = (adi_fpga9001_Device_t*)calloc(1, sizeof(adi_fpga9001_Device_t));
    if (adrv9001Ee01->fpga9001Device == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* Set function in the platform layer to be ADI_HW_PLATFORM*/
    adi_hal_PlatformSetup(NULL, ADI_HW_PLATFORM);

    /* Creat devHalInfo for the devices*/
    /* Check if function pointer is populated */
    if (adi_hal_DevHalCfgCreate == NULL)
    {
        /* TODO: add an error structure to the board level*/
        printf("NULL Hal layer function pointer");
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->adrv9001Device->common.devHalInfo = adi_hal_DevHalCfgCreate((ADI_HAL_INTERFACE_SPI | ADI_HAL_INTERFACE_LOG | ADI_HAL_INTERFACE_HWRESET | ADI_HAL_INTERFACE_TIMER), 0, "adrv9001Log.txt");
    if (adrv9001Ee01->adrv9001Device->common.devHalInfo == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->fpga9001Device->common.devHalInfo = adi_hal_DevHalCfgCreate((ADI_HAL_INTERFACE_BBICCTRL | ADI_HAL_INTERFACE_LOG | ADI_HAL_INTERFACE_I2C | ADI_HAL_INTERFACE_TIMER), 0, "fpgaLog.txt");
    if (adrv9001Ee01->fpga9001Device->common.devHalInfo == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* Open hardware for different devices*/
    recoveryAction = adi_hal_HwOpen(adrv9001Ee01->fpga9001Device->common.devHalInfo);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        printf("Board HwOpen failed. %s \n", adrv9001Ee01->fpga9001Device->common.error.errormessage);
        return recoveryAction;
    }

    recoveryAction = adi_adrv9001_HwOpen(adrv9001Ee01->adrv9001Device, &spiSettings);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        printf("Navassa HwOpen failed. %s \n", adrv9001Ee01->adrv9001Device->common.error.errormessage);
        return recoveryAction;
    }

    /* Clear any previous adrv9001 Errors */
    recoveryAction = adi_common_ErrorClear(&adrv9001Ee01->adrv9001Device->common);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        return recoveryAction;
    }

    return recoveryAction;
}

int32_t adi_adrv9001Ee01_Destroy(adi_adrv9001Ee01_Board_t *adrv9001Ee01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    if (adrv9001Ee01 == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    if (adrv9001Ee01->adrv9001Device == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    if (adrv9001Ee01->fpga9001Device == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* Open hardware for the different devices*/
    recoveryAction = adi_hal_HwClose(adrv9001Ee01->fpga9001Device->common.devHalInfo);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        return recoveryAction;
    }

    recoveryAction = adi_adrv9001_HwClose(adrv9001Ee01->adrv9001Device);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        return recoveryAction;
    }

    /* Check if function pointer is populated */
    if (adi_hal_DevHalCfgFree == NULL)
    {
        /* TODO: add an error structure to the board level*/
        printf("NULL Hal layer function pointer");
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* De-allocate Memory for HAL config created by adrv9001Board_Program() function */
    adi_hal_DevHalCfgFree(adrv9001Ee01->adrv9001Device->common.devHalInfo);
    adi_hal_DevHalCfgFree(adrv9001Ee01->fpga9001Device->common.devHalInfo);

    free(adrv9001Ee01->adrv9001Device);
    free(adrv9001Ee01->fpga9001Device);
    free(adrv9001Ee01);

    return recoveryAction;
}

int32_t adi_adrv9001Ee01_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                               const char* profileFilename,
                                               adi_adrv9001_RadioConfig_t *adrv9001RadioConfig,
                                               adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles)
{
    adi_adrv9001_Device_t *adrv9001Device = NULL;

    adi_adrv9001_Init_t adrv9001InitInst = { {0} };
    adi_adrv9001_Init_t *adrv9001Init = &adrv9001InitInst;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;
    adrv9001Device = adrv9001Ee01->adrv9001Device;

    ADI_EXPECT(adi_adrv9001_Utilities_DeviceProfile_Load, adrv9001Device, profileFilename, adrv9001Init);

    adi_adrv9001Ee01_Initialize(adrv9001Ee01, adrv9001Init, adrv9001RadioConfig, adrv9001PlatformFiles);

    ADI_API_RETURN(adrv9001Device);
}

/* These values are hard-coded because they are specific to the FPGA9001/Zc706 platform.
 * The calibration process to determine these values for a different FPGA will be documented in the future. */
static void ssiConfigSettingsInitDefault(adi_adrv9001_SsiConfigSettings_t *ssiConfig)
{
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxStrobeDelay[0] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxStrobeDelay[1] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxIDataDelay[0] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxIDataDelay[1] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxQDataDelay[0] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxQDataDelay[1] = 0x0;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsTxClkDelay[0] = 0x3;
    ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsTxClkDelay[1] = 0x3;
}

int32_t adi_adrv9001Ee01_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                    adi_adrv9001_Init_t *adrv9001Init,
                                    adi_adrv9001_RadioConfig_t *adrv9001RadioConfig,
                                    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles)
{
    int32_t recoveryAction = 0;
    int32_t(*Mcs_Requested_func)() = &adi_adrv9001Ee01_Mcs_Requested;
    adi_adrv9001_Device_t *adrv9001Device = NULL;
    adi_fpga9001_Device_t *fpga9001Device = NULL;

    adi_fpga9001_SsiMode_e ssiMode = ADI_FPGA9001_CMOS_1LANE_16I16Q;
    adi_adrv9001_SsiConfigSettings_t ssiConfig = { 0 };
    ssiConfigSettingsInitDefault(&ssiConfig);

    adi_adrv9001_ResourceCfg_t adrv9001ResourceCfg = { adrv9001Init, adrv9001RadioConfig, adrv9001PlatformFiles };

    adi_fpga9001_MmcmCfg_t desiredMmcmCfg = { 0 };

    adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor = ADI_FPGA9001_MMCM_CLKDIV_BYPASS;

    uint8_t initCalsError = 0;
    uint8_t channelMask = 0;

    static const uint8_t ALL_RX_CHANNEL_MASK = 0xF;
    /* TX_CHANNEL_MASK_OFFSET is '4' as channel mask info for RX1/2 and ORX1/2 occupy the least 4 bits */
    static const uint8_t TX_CHANNEL_MASK_OFFSET = 4;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;
    fpga9001Device = adrv9001Ee01->fpga9001Device;
    boardRef = adrv9001Ee01;
    
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001RadioConfig);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001PlatformFiles);

    adrv9001DeviceClockOutDivisor = (adi_fpga9001_Mmcm_ClockOutput_Divisor_e)(adrv9001RadioConfig->radioCtrlInit.adrv9001DeviceClockDivisor);

    mmcmConfigSettingsInitDefault(&desiredMmcmCfg, adrv9001Init->clocks.deviceClock_kHz, adrv9001DeviceClockOutDivisor);

    channelMask = (adrv9001Init->tx.txInitChannelMask << TX_CHANNEL_MASK_OFFSET) | (adrv9001Init->rx.rxInitChannelMask & ALL_RX_CHANNEL_MASK);

    if (fpga9001Device == NULL)
    {
        ADI_ERROR_REPORT(&adrv9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         fpga9001Device,
                         "Error: FPGA device pointer is null.");
        ADI_ERROR_RETURN(adrv9001Device->common.error.newAction);
    }

    adi_common_ErrorClear(&adrv9001Device->common);

    recoveryAction = adi_adrv9001Ee01_SelectFpgaBin(adrv9001Ee01, adrv9001Init);
    ADI_ERROR_RETURN(recoveryAction);

    ADI_EXPECT(adi_adrv9001_HwReset, adrv9001Device);

    ADI_EXPECT(adi_adrv9001_InitAnalog,
               adrv9001Device,
               adrv9001Init,
               adrv9001RadioConfig->radioCtrlInit.adrv9001DeviceClockDivisor);

    ADI_EXPECT(adi_adrv9001_InitDigitalLoad, adrv9001Device, &adrv9001ResourceCfg);

    ADI_EXPECT(adi_fpga9001_Initialize, fpga9001Device, adrv9001Init, &ssiMode, &desiredMmcmCfg, adrv9001DeviceClockOutDivisor);

    ssiConfig.ssiMode = adrv9001_SsiMode_Get(ssiMode);
    if (ssiMode == ADI_COMMON_ACT_ERR_CHECK_PARAM)
    {
        ADI_ERROR_REPORT(&fpga9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         mode,
                         "No equivalent ADRV9001 SSI mode found for the given FPGA SSI mode");
    }

    ADI_EXPECT(adi_adrv9001_InitRadioLoad, adrv9001Device, Mcs_Requested_func, &adrv9001ResourceCfg, &ssiConfig, channelMask);

    /* Reset FPGA SSI channel reset before Init Cal to sync FPGA with the LVDS/CMOS delay configurations set in Navassa device */
    ADI_EXPECT(adi_fpga9001_SsiChannelReset, fpga9001Device);

    ADI_EXPECT(adi_adrv9001_InitCalsRun, adrv9001Device, Mcs_Requested_func, &adrv9001RadioConfig->initCals, 60000, &initCalsError);

    ADI_EXPECT(adi_adrv9001_InitRxGainParameters, adrv9001Device, adrv9001Init, &adrv9001ResourceCfg);

    ADI_API_RETURN(adrv9001Device);
}