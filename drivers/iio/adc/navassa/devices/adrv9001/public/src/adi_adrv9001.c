/**
* \file
* \brief Contains Adrv9001 features related function implementation defined in
* adi_adrv9001.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* Header file corresponding to the C file */
#include "adi_adrv9001.h"

/* ADI specific header files */
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_types.h"
//#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_version.h"
#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"


/* Header files related to libraries */


/* System header files */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#else
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#endif


#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001.c"
#endif

/*********************************************************************************************************/
int32_t adi_adrv9001_HwOpen(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spiSettings)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_EXPECT(adi_adrv9001_HwOpenNoReset, device, spiSettings);

    /* Toggle RESETB pin, Configure and Verify SPI */
    ADI_MSG_EXPECT("Failed to reset device and set SPI Config", adi_adrv9001_HwReset, device);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_HwOpenNoReset(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spiSettings)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
    /* Check for Hal layer functions */
    /* TODO: move section to common layer (add new function) */
    if ((adi_hal_HwOpen == NULL) ||
        (adi_hal_HwClose == NULL) ||
        (adi_hal_HwReset == NULL) ||
        (adi_hal_SpiWrite == NULL) ||
        (adi_hal_SpiRead == NULL) ||
        (adi_hal_LogFileOpen == NULL) ||
        (adi_hal_LogLevelSet == NULL) ||
        (adi_hal_LogLevelGet == NULL) ||
        (adi_hal_LogWrite == NULL) ||
        (adi_hal_Wait_us == NULL) ||
        (adi_hal_Wait_ms == NULL))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_ADI_HAL,
                         ADI_HAL_LIBRARY_NOT_AVAILABLE,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         NULL,
                         "Hal library Function pointers not set properly");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    adi_common_LogLevelSet(&device->common, ADI_ADRV9001_LOGGING);

    recoveryAction = adi_common_hal_HwOpen(&device->common);

    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        switch (recoveryAction)
        {
        case ADI_COMMON_HAL_SPI_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_RESET_INTERFACE,
                             NULL,
                             "SPI error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_GPIO_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_RESET_FEATURE,
                             NULL,
                             "GPIO error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_TIMER_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             NULL,
                             "Timer error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_GEN_SW: /* fall through */
        default:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             NULL,
                             "Param error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    device->spiSettings.autoIncAddrUp		= spiSettings->autoIncAddrUp;
    device->spiSettings.cmosPadDrvStrength	= spiSettings->cmosPadDrvStrength;
    device->spiSettings.enSpiStreaming		= spiSettings->enSpiStreaming;
    device->spiSettings.fourWireMode		= spiSettings->fourWireMode;
    device->spiSettings.msbFirst			= spiSettings->msbFirst;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_HwClose(adi_adrv9001_Device_t *device)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_EXPECT(device);

    recoveryAction = adi_common_hal_HwClose(&device->common);

    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        switch (recoveryAction)
        {
        case ADI_COMMON_HAL_SPI_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_RESET_INTERFACE,
                             NULL,
                             "SPI error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_GPIO_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_RESET_FEATURE,
                             NULL,
                             "GPIO error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_TIMER_FAIL:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             NULL,
                             "Timer error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        case ADI_COMMON_HAL_GEN_SW: /* fall through */
        default:
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             device->common.error.errCode,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             NULL,
                             "Param error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_HwReset(adi_adrv9001_Device_t *device)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    static const uint8_t RESETB_LEVEL_LOW = 0;
    static const uint8_t RESETB_LEVEL_HIGH = 1;
    static const uint8_t RESETB_WAIT_MS = 1;

    ADI_API_ENTRY_EXPECT(device);

    /* toggle RESETB on device with matching spi chip select index */
    recoveryAction = adi_common_hal_HwReset(&device->common, RESETB_LEVEL_LOW);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_ADI_HAL,
                         device->common.error.errCode,
                         ADI_COMMON_ACT_ERR_RESET_FEATURE,
                         NULL,
                         "GPIO Reset error");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adi_common_hal_Wait_ms(&device->common, RESETB_WAIT_MS);

    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_ADI_HAL,
                         device->common.error.errCode,
                         ADI_COMMON_ACT_ERR_CHECK_TIMER,
                         NULL,
                         "Timer error");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adi_common_hal_HwReset(&device->common, RESETB_LEVEL_HIGH);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_ADI_HAL,
                         device->common.error.errCode,
                         ADI_COMMON_ACT_ERR_RESET_FEATURE,
                         NULL,
                         "GPIO Reset error");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Configure and Verify Spi */
    ADI_MSG_EXPECT("Failed to set SPI Config", adi_adrv9001_spi_Configure, device, &device->spiSettings);

    device->devStateInfo.devState = ADI_ADRV9001_STATE_POWERON_RESET;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_InitAnalog(adi_adrv9001_Device_t *device,
                                adi_adrv9001_Init_t *init,
                                adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor)
{
    adi_adrv9001_Info_t devStateInfoClear = { 0 };
    static const uint8_t MAX_GAIN_INDEX = 0xFF;

    ADI_API_ENTRY_PTR_EXPECT(device, init);

    /* Device clock divisor value should be between 0 and 6 */
    ADI_RANGE_CHECK(device, adrv9001DeviceClockOutDivisor, ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS, ADI_ADRV9001_DEVICECLOCKDIVISOR_64);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest >  0)
    {
        devStateInfoClear.swTest = device->devStateInfo.swTest;
    }
#endif

    device->devStateInfo = devStateInfoClear;

    device->devStateInfo.gainIndexes.rx1MaxGainIndex = MAX_GAIN_INDEX;
    device->devStateInfo.gainIndexes.rx2MaxGainIndex = MAX_GAIN_INDEX;

    ADI_EXPECT(adrv9001_InitAnalog, device, init, adrv9001DeviceClockOutDivisor);

    /* Disable stream pin mode until after streams are loaded */
    /* Disable Tx pin mode for all Tx and Rx channels, ORx was defaulted with pin mode disabled */
    ADRV9001_SPIWRITEBYTE(device, "BBIC_ENABLES", ADRV9001_ADDR_BBIC_ENABLES, 0x00);

    device->devStateInfo.devState = (adi_adrv9001_ApiStates_e)(device->devStateInfo.devState | ADI_ADRV9001_STATE_ANA_INITIALIZED);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_InitDigital(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    uint8_t clkSel = 0;

    static const uint8_t HS_DIG_CLK_ENABLE       = 0x40;

    ADI_API_ENTRY_PTR_EXPECT(device, init);

    if (device->devStateInfo.devState != ADI_ADRV9001_STATE_ANA_INITIALIZED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device->devStateInfo.devState,
                         "Invalid State.  Expecting Analog Initialized state\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_EXPECT(adi_adrv9001_arm_Disable, device);
    ADI_EXPECT(adi_adrv9001_arm_AhbSpiBridge_Disable, device);

    /* Clk1105 High Speed clock selection */
    /* 0: High Speed digital clock Divided by 2 */
    /* 1: High Speed digital clock */
    if (init->clocks.clk1105Div ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_2)
    {
        clkSel = 0;
    }
    else if (init->clocks.clk1105Div ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_1)
    {
        clkSel = 1;
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         init->clocks.clk1105Div,
                         "Invalid init->clocks.clk1105Div. Valid range (/1 or /2)\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADRV9001_SPIWRITEBYTE(device, "HS_CLK_DIV", ADRV9001_ADDR_HS_CLK_DIV, clkSel);

    /* ARM Clock Divider Selection: */
    /*         core_bf.arm_clk_sel = 1      */
    /* Else    core_bf.arm_clk_sel = 2      */
    if (init->clocks.armClkDiv ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_4)
    {
        clkSel = 1;
    }
    else if (init->clocks.armClkDiv ==  ADI_ADRV9001_INTERNAL_CLOCK_DIV_6)
    {
        clkSel = 2;
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         init->clocks.armClkDiv,
                         "Invalid init->clocks.armClkDiv. Valid range (/4 or /6)\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    ADRV9001_SPIWRITEBYTE(device, "ARM_CLOCK_CTRL", ADRV9001_ADDR_ARM_CLOCK_CTRL, clkSel);

    /* High speed clock enable */
    clkSel = HS_DIG_CLK_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "TOP_CLOCK_CTRL", ADRV9001_ADDR_TOP_CLOCK_CTRL, clkSel);

    ADI_EXPECT(adi_adrv9001_arm_AhbSpiBridge_Enable, device);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Shutdown(adi_adrv9001_Device_t *device)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_MSG_EXPECT("Failed to reset device", adi_adrv9001_HwReset, device);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_spi_Configure(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spi)
{
    uint8_t spiConfigA = 0;
    uint8_t spiConfigB = 0;
    //uint8_t ioControl = 0;
    static const uint8_t SPICFG_MSBFIRST_OFF = 0;
    static const uint8_t SPICFG_AUTOINC_DOWN = 0;
    static const uint8_t SPICFG_FOURWIREMODE_OFF = 0;
    static const uint8_t SPICFG_ENSPISTREAMING_OFF = 0;

    ADI_API_ENTRY_PTR_EXPECT(device, spi);

    if ((spi->cmosPadDrvStrength != ADI_ADRV9001_CMOSPAD_DRV_WEAK) &&
        (spi->cmosPadDrvStrength != ADI_ADRV9001_CMOSPAD_DRV_STRONG))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            spi->cmosPadDrvStrength,
            "Invalid CMOS Pad Drive Strength\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* core.spi_interface_config_B */
    if (spi->enSpiStreaming == SPICFG_ENSPISTREAMING_OFF)
    {
        spiConfigB |= ADRV9001_CONFIG_B_SINGLE_INSTRUCTION;
    }

    /* Force single instruction mode */
    ADRV9001_SPIWRITEBYTE(device, "SPI_INTERFACE_CONFIG_B", ADRV9001_ADDR_SPI_INTERFACE_CONFIG_B, spiConfigB);

    /* core.spi_interface_config_A */
    /* SPI bit is 1 = LSB first */
    if (spi->msbFirst == SPICFG_MSBFIRST_OFF)
    {
        spiConfigA |= ADRV9001_CONFIG_A_SPI_LSB_FIRST;
    }

    if (spi->autoIncAddrUp != SPICFG_AUTOINC_DOWN)
    {
        spiConfigA |= ADRV9001_CONFIG_A_SPI_ADDR_ASCENSION;
    }

    if (spi->fourWireMode != SPICFG_FOURWIREMODE_OFF)
    {
        spiConfigA |= ADRV9001_CONFIG_A_SPI_SDO_ACTIVE;
    }

    ADRV9001_SPIWRITEBYTE(device, "SPI_INTERFACE_CONFIG_A", ADRV9001_ADDR_SPI_INTERFACE_CONFIG_A, spiConfigA);

/* FIXME - Start: Vivek - In Tokelau, ADRV9001_ADDR_DIGITAL_IO_CONTROL is find in YODA file;
 * In Navassa, this is missing. Need to check whether it is needed.
 * Commenting this code snippet and related variables/constants used in this function for now
*/

    //if (spi->cmosPadDrvStrength == ADI_ADRV9001_CMOSPAD_DRV_STRONG)
    //{
        //ioControl |= ADRV9001_IO_CONTROL_SPI_OUTS_DRV_SEL;
    //}

    /* Force PAD mode */
    //ADRV9001_SPIWRITEBYTE(device, "DIGITAL_IO_CONTROL", ADRV9001_ADDR_DIGITAL_IO_CONTROL, ioControl);

/* FIXME - End: Vivek - In Tokelau, ADRV9001_ADDR_DIGITAL_IO_CONTROL is find in YODA file;
 * In Navassa, this is missing. Need to check whether it is needed.
 * Commenting this code snippet and related variables/constants used in this function for now
*/

#if ADI_ADRV9001_DEVICE_NOT_CONNECTED == 0
#if ADI_ADRV9001_PRE_MCS_BROADCAST_DISABLE > 0
    ADI_MSG_EXPECT("SPI Verify failed", adi_adrv9001_spi_Verify, device) ;
#endif
#endif

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_spi_Inspect(adi_adrv9001_Device_t *device, adi_adrv9001_SpiSettings_t *spi)
{
    uint8_t spiConfigA = 0;
    uint8_t spiConfigB = 0;
    //uint8_t ioControl = 0;
    static const uint8_t SPICFG_MSBFIRST_OFF = 0;
    static const uint8_t SPICFG_AUTOINC_DOWN = 0;
    static const uint8_t SPICFG_FOURWIREMODE_OFF = 0;
    static const uint8_t SPICFG_ENSPISTREAMING_OFF = 0;
    static const uint8_t SPICFG_MSBFIRST_ON = 1;
    static const uint8_t SPICFG_AUTOINC_UP = 1;
    static const uint8_t SPICFG_FOURWIREMODE_ON = 1;
    static const uint8_t SPICFG_ENSPISTREAMING_ON = 1;

    ADI_API_ENTRY_PTR_EXPECT(device, spi);

    ADRV9001_SPIREADBYTE(device, "SPI_INTERFACE_CONFIG_A", ADRV9001_ADDR_SPI_INTERFACE_CONFIG_A, &spiConfigA);

    /* core.spi_interface_config_A */
    /* SPI bit is 1 = LSB first */
    if (ADRV9001_BF_EQUAL(spiConfigA, ADRV9001_CONFIG_A_SPI_LSB_FIRST))
    {
        spi->msbFirst = SPICFG_MSBFIRST_OFF;
    }
    else
    {
        spi->msbFirst = SPICFG_MSBFIRST_ON;
    }

    if (ADRV9001_BF_EQUAL(spiConfigA, ADRV9001_CONFIG_A_SPI_ADDR_ASCENSION))
    {
        spi->autoIncAddrUp = SPICFG_AUTOINC_UP;
    }
    else
    {
        spi->autoIncAddrUp = SPICFG_AUTOINC_DOWN;
    }

    if (ADRV9001_BF_EQUAL(spiConfigA, ADRV9001_CONFIG_A_SPI_SDO_ACTIVE))
    {
        spi->fourWireMode = SPICFG_FOURWIREMODE_ON;
    }
    else
    {
        spi->fourWireMode = SPICFG_FOURWIREMODE_OFF;
    }

/* FIXME - Start: Vivek - In Tokelau, ADRV9001_ADDR_DIGITAL_IO_CONTROL is find in YODA file;
 * In Navassa, this is missing. Need to check whether it is needed.
 * Commenting this code snippet and related variables/constants used in this function for now
*/

    /* Read PAD mode */
    //ADRV9001_SPIREADBYTE(device, "DIGITAL_IO_CONTROL", ADRV9001_ADDR_DIGITAL_IO_CONTROL, &ioControl);

    //if (ADRV9001_BF_EQUAL(ioControl, ADRV9001_IO_CONTROL_SPI_OUTS_DRV_SEL))
    //{
        //spi->cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG;
    //}
    //else
    //{
        //spi->cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_WEAK;
    //}

/* FIXME - End: Vivek - In Tokelau, ADRV9001_ADDR_DIGITAL_IO_CONTROL is find in YODA file;
 * In Navassa, this is missing. Need to check whether it is needed.
 * Commenting this code snippet and related variables/constants used in this function for now
*/

    /* Read single instruction mode */
    ADRV9001_SPIREADBYTE(device, "SPI_INTERFACE_CONFIG_B", ADRV9001_ADDR_SPI_INTERFACE_CONFIG_B, &spiConfigB);

    /* core.spi_interface_config_B */
    if (ADRV9001_BF_EQUAL(spiConfigB, ADRV9001_CONFIG_B_SINGLE_INSTRUCTION))
    {
        spi->enSpiStreaming = SPICFG_ENSPISTREAMING_OFF;
    }
    else
    {
        spi->enSpiStreaming = SPICFG_ENSPISTREAMING_ON;
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_spi_Verify(adi_adrv9001_Device_t *device)
{
    uint8_t spiReg = 0;

    ADI_API_ENTRY_EXPECT(device);

    static const uint8_t SCRATCH_PAD_1 = 0xB6; /* DATA 10110110 */
    static const uint8_t SCRATCH_PAD_2 = 0x49; /* DATA 01001001 */
    static const uint8_t SCRATCH_PAD_3 = 0xA5; /* DATA 10100101 */
    static const uint8_t VENDOR_ID_0   = 0x56;
    static const uint8_t VENDOR_ID_1   = 0x04;

    /* Check SPI read - VENDOR_ID_0 */
    ADRV9001_SPIREADBYTE(device, "VENDOR_ID_0", ADRV9001_ADDR_VENDOR_ID_0, &spiReg);
#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 1)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != VENDOR_ID_0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot read from a low SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI read - VENDOR_ID_1 */
    spiReg = 0;
    ADRV9001_SPIREADBYTE(device, "VENDOR_ID_1", ADRV9001_ADDR_VENDOR_ID_1, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 2)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != VENDOR_ID_1)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot read from a low SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI write - SCRATCHPAD : Data = 10110110 */
    spiReg = 0;
    ADRV9001_SPIWRITEBYTE(device, "SCRATCH_PAD", ADRV9001_ADDR_SCRATCH_PAD, SCRATCH_PAD_1);

    ADRV9001_SPIREADBYTE(device, "SCRATCH_PAD", ADRV9001_ADDR_SCRATCH_PAD, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 3)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != SCRATCH_PAD_1)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot write to a low SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI write - SCRATCHPAD : Data = 01001001 */
    spiReg = 0;
    ADRV9001_SPIWRITEBYTE(device, "SCRATCH_PAD", ADRV9001_ADDR_SCRATCH_PAD, SCRATCH_PAD_2);

    ADRV9001_SPIREADBYTE(device, "SCRATCH_PAD", ADRV9001_ADDR_SCRATCH_PAD, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 4)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != SCRATCH_PAD_2)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot write to a low SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI read - ADRV9001_ADDR_SCRATCH_PAD_READ_ONLY_UPPER_ADDRESS_SPACE : Data = 10100101 */
    spiReg = 0;
    ADRV9001_SPIREADBYTE(device, "SCRATCH_PAD_READ_ONLY_UPPER_ADDR", ADRV9001_ADDR_SCRATCH_PAD_READ_ONLY_UPPER_ADDRESS_SPACE, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 5)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != SCRATCH_PAD_3)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                        ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot read from a high SPI address\n");
                        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI write - SCRATCH_PAD_UPPER_ADDRESS_SPACE : Data = 10110110 */
    spiReg = 0;
    ADRV9001_SPIWRITEBYTE(device, "SCRATCH_PAD_UPPER_ADDR", ADRV9001_ADDR_SCRATCH_PAD_UPPER_ADDRESS_SPACE, SCRATCH_PAD_1);

    ADRV9001_SPIREADBYTE(device, "SCRATCH_PAD_UPPER_ADDR", ADRV9001_ADDR_SCRATCH_PAD_UPPER_ADDRESS_SPACE, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 6)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != SCRATCH_PAD_1)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot write to a high SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check SPI write - SCRATCH_PAD_UPPER_ADDRESS_SPACE : Data = 01001001 */
    spiReg = 0;
    ADRV9001_SPIWRITEBYTE(device, "SCRATCH_PAD_UPPER_ADDR", ADRV9001_ADDR_SCRATCH_PAD_UPPER_ADDRESS_SPACE, SCRATCH_PAD_2);

    ADRV9001_SPIREADBYTE(device, "SCRATCH_PAD_UPPER_ADDR", ADRV9001_ADDR_SCRATCH_PAD_UPPER_ADDRESS_SPACE, &spiReg);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 7)
    {
        spiReg = 0;
    }
#endif

    if (spiReg != SCRATCH_PAD_2)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FULL, spiReg, "Cannot write to a high SPI address\n");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* FIXME: Vivek -  Need to keep an eye on all 'TODO' to make sure Tokelau changes are brought in */
    /* TODO: Add code to check Spi Streaming when HAL support is available. */

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_ApiVersionGet(adi_adrv9001_Device_t *device, adi_common_ApiVersion_t *apiVersion)
{
    const char *version = ADI_ADRV9001_CURRENT_VERSION;

    ADI_API_ENTRY_PTR_EXPECT(device, apiVersion);

    apiVersion->major = 0;
    apiVersion->minor = 0;
    apiVersion->patch = 0;
    apiVersion->build = 0;

    sscanf(version, "%u.%u.%u.%u", &apiVersion->major, &apiVersion->minor,
	  &apiVersion->patch, &apiVersion->build);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_DeviceRevGet(adi_adrv9001_Device_t *device, uint8_t *siRevision)
{
    ADI_API_ENTRY_PTR_EXPECT(device, siRevision);

    ADRV9001_SPIREADBYTE(device, "PRODUCT_ID_1", ADRV9001_ADDR_PRODUCT_ID_1, siRevision);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_ProductIdGet(adi_adrv9001_Device_t *device, uint8_t *productId)
{
    /* FIXME: Vivek - PRODUCT_ID_0 is not used in Navassa. This API can be removed in navassa */
    return ADI_COMMON_ACT_NO_ACTION;
}

int32_t adi_adrv9001_ProfilesVerify(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    ADI_API_ENTRY_PTR_EXPECT(device, init);

    ADI_EXPECT(adrv9001_ProfilesVerify, device, init);

    ADI_API_RETURN(device);
}
