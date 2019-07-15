/**
* \file
* \brief Contains board level Multi-Chip Synchronization (MCS) functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001ee01_mcs.h"

#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_mcs.h"

#include "adrv9001_bf_nvs_pll_mem_map.h"
#include "adrv9001_decode_bf_enum.h"

#include "adi_fpga9001_mcs.h"

static int32_t Mcs_Execute_Validate(adi_adrv9001_Device_t *device, adi_adrv9001_McsAction_e mcsAction)
{
    ADI_RANGE_CHECK(device, mcsAction, ADI_ADRV9001_MCS_INT_DIGITAL_ALL, ADI_ADRV9001_MCS_EXT_ANALOG_3);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001Ee01_Mcs_Execute(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                     adi_adrv9001_McsAction_e mcsAction,
                                     bool bbicInitiated)
{
    uint8_t regVal = 0;
    uint8_t i = 0;
    int32_t halError = ADI_COMMON_ACT_NO_ACTION;
    adi_adrv9001_Device_t *adrv9001Device = NULL;
    adi_fpga9001_Device_t *fpga9001Device = NULL;
    adi_adrv9001_McsAction_e mcsActionRequested = ADI_ADRV9001_MCS_READY;
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;
    bool armReadyForMcs = false;

    static const uint8_t MCS_PERIOD = 10; /* TODO: Value TBD */
    static const uint8_t ANALOG_DEVCLK_REFCLK_DIV_SYNCED = 0x10;
    static const uint8_t ANALOG_SDM_SYNCED = 0x08;
    static const uint8_t ANALOG_CLKGEN_SYNCED = 0x04;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;
    fpga9001Device = adrv9001Ee01->fpga9001Device;

    ADI_PERFORM_VALIDATION(Mcs_Execute_Validate, adrv9001Device, mcsAction);

    baseAddr = adrv9001_PllMemMapAddr_Get(adrv9001Device->devStateInfo.clkPllMode, ADI_ADRV9001_PLL_TYPE_CLK);

    if (true == bbicInitiated)
    {
        ADI_EXPECT(adi_adrv9001_Mcs_PrepareArm, adrv9001Device);
        /* Confirm ARM is sleeping - poll GPINT pin; can't do via interrupt due to lost information */
        for (i = 0; i < 10; i++)
        {
            ADI_EXPECT(adi_fpga9001_Mcs_Requested, fpga9001Device, &armReadyForMcs);
            if (true == armReadyForMcs)
            {
                break;
            }

            halError = adi_common_hal_Wait_us(&adrv9001Device->common, 100000); /* Wait 100ms */

            ADI_ERROR_REPORT(&adrv9001Device->common,
                             ADI_ADRV9001_SRC_ARMCMD,
                             halError,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             NULL,
                             "Timer not working");
            ADI_ERROR_RETURN(adrv9001Device->common.error.newAction);
        }

        if (false == armReadyForMcs)
        {
            ADI_ERROR_REPORT(&adrv9001Device->common,
                             ADI_ADRV9001_SRC_ARMCMD,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "Timed out waiting for ARM to signal ready for MCS");
            ADI_ERROR_RETURN(adrv9001Device->common.error.newAction);
        }
        else
        {
            ADI_EXPECT(adi_adrv9001_Mcs_ActionRequested_Get, adrv9001Device, &mcsActionRequested);
            if (ADI_ADRV9001_MCS_READY != mcsActionRequested)
            {
                ADI_ERROR_REPORT(&adrv9001Device->common,
                                 ADI_ADRV9001_SRC_ARMCMD,
                                 ADI_COMMON_ERR_API_FAIL,
                                 ADI_COMMON_ACT_ERR_RESET_FULL,
                                 NULL,
                                 "ARM acknowledged BBIC-initiated MCS incorrectly");
                ADI_ERROR_RETURN(adrv9001Device->common.error.newAction);
            }
        }
    }

    ADI_EXPECT(adi_adrv9001_arm_Disable, adrv9001Device);

    switch (mcsAction)
    {
    case ADI_ADRV9001_MCS_INT_DIGITAL_ALL:
        ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, adrv9001Device);
        ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device, 2);
        break;
    case ADI_ADRV9001_MCS_INT_DIGITAL_CLK:
        ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, adrv9001Device);
        ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device, 1);
        break;
    case ADI_ADRV9001_MCS_INT_DIGITAL_SSI:
        ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device, 1);
        break;
    case ADI_ADRV9001_MCS_EXT_DIGITAL_ALL:
        ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, adrv9001Device);
        ADI_EXPECT(adi_adrv9001_Mcs_Internal_Select, adrv9001Device, false);
        ADI_EXPECT(adi_adrv9001_Mcs_Capture_Enable, adrv9001Device);
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 2, MCS_PERIOD);
        break;
    case ADI_ADRV9001_MCS_EXT_DIGITAL_CLK:
        ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, adrv9001Device);
        ADI_EXPECT(adi_adrv9001_Mcs_Internal_Select, adrv9001Device, false);
        ADI_EXPECT(adi_adrv9001_Mcs_Capture_Enable, adrv9001Device);
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 1, MCS_PERIOD);
        break;
    case ADI_ADRV9001_MCS_EXT_DIGITAL_SSI:
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 1, MCS_PERIOD);
        break;
    case ADI_ADRV9001_MCS_EXT_ANALOG_ALL:
        ADI_EXPECT(adi_adrv9001_Mcs_Prepare, adrv9001Device, ADI_ADRV9001_PLL_TYPE_CLK);
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 4, MCS_PERIOD);
        ADI_EXPECT(adrv9001_NvsPllMemMapMcsSpiStatusBfGet, adrv9001Device, baseAddr, &regVal);
        ADI_EXPECT(adrv9001_NvsPllMemMapClkgenUseSerdesMcsBfSet, adrv9001Device, baseAddr, 1);
        break;
    case ADI_ADRV9001_MCS_EXT_ANALOG_1:
        ADI_EXPECT(adi_adrv9001_Mcs_Prepare, adrv9001Device, ADI_ADRV9001_PLL_TYPE_CLK);
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 2, MCS_PERIOD);
        ADI_EXPECT(adrv9001_NvsPllMemMapMcsSpiStatusBfGet, adrv9001Device, baseAddr, &regVal);
        if ((regVal & ANALOG_DEVCLK_REFCLK_DIV_SYNCED) != ANALOG_DEVCLK_REFCLK_DIV_SYNCED)
        {
            ADI_ERROR_REPORT(&adrv9001Device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "Analog MCS 1 failed");
        }
        break;
    case ADI_ADRV9001_MCS_EXT_ANALOG_2:
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 1, MCS_PERIOD);
        ADI_EXPECT(adrv9001_NvsPllMemMapMcsSpiStatusBfGet, adrv9001Device, baseAddr, &regVal);
        if ((regVal & ANALOG_SDM_SYNCED) != ANALOG_SDM_SYNCED)
        {
            ADI_ERROR_REPORT(&adrv9001Device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "Analog MCS 2 failed");
        }
        break;
    case ADI_ADRV9001_MCS_EXT_ANALOG_3:
        ADI_EXPECT(adi_fpga9001_Mcs_Start, fpga9001Device, 1, MCS_PERIOD);
        ADI_EXPECT(adrv9001_NvsPllMemMapMcsSpiStatusBfGet, adrv9001Device, baseAddr, &regVal);
        if ((regVal & ANALOG_CLKGEN_SYNCED) != ANALOG_CLKGEN_SYNCED)
        {
            ADI_ERROR_REPORT(&adrv9001Device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_FULL,
                             NULL,
                             "Analog MCS 3 failed");
        }
        ADI_EXPECT(adrv9001_NvsPllMemMapClkgenUseSerdesMcsBfSet, adrv9001Device, baseAddr, 1);
        break;
    default:
        ADI_ERROR_REPORT(&adrv9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FEATURE,
                         NULL,
                         "ADRV9001 did not request a known MCS action");
    }

    ADI_EXPECT(adi_adrv9001_arm_Enable, adrv9001Device);

    ADI_API_RETURN(adrv9001Device);
}

int32_t adi_adrv9001Ee01_Mcs_IrqHandler(adi_adrv9001Ee01_Board_t *adrv9001Ee01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    adi_adrv9001_Device_t *adrv9001Device = NULL;
    adi_adrv9001_McsAction_e mcsActionRequested = ADI_ADRV9001_MCS_READY;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;

    ADI_EXPECT(adi_adrv9001_Mcs_ActionRequested_Get, adrv9001Device, &mcsActionRequested);

    /* FIXME: Remove when FW actually supports this feature */
    mcsActionRequested = ADI_ADRV9001_MCS_INT_DIGITAL_ALL;

    recoveryAction = adi_adrv9001Ee01_Mcs_Execute(adrv9001Ee01, mcsActionRequested, false);
    if (ADI_COMMON_ACT_NO_ACTION != recoveryAction)
    {
        ADI_ERROR_REPORT(&adrv9001Ee01->adrv9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         adrv9001Ee01->adrv9001Device->common.error.errCode,
                         recoveryAction,
                         NULL,
                         adrv9001Ee01->adrv9001Device->common.error.errormessage);
        ADI_ERROR_RETURN(adrv9001Ee01->adrv9001Device->common.error.newAction);
    }

    ADI_API_RETURN(adrv9001Device);
}

adi_adrv9001Ee01_Board_t *boardRef = NULL;
int32_t adi_adrv9001Ee01_Mcs_Requested()
{
    bool requested = false;

    ADI_EXPECT(adi_fpga9001_Mcs_Requested, boardRef->fpga9001Device, &requested);
    if (requested)
    {
        return adi_adrv9001Ee01_Mcs_IrqHandler(boardRef);
    }

    return ADI_COMMON_ACT_NO_ACTION;
}