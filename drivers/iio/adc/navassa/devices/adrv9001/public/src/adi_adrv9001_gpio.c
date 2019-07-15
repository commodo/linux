/**
* \file
* \brief Contains gpio features related function implementation defined in
* adi_adrv9001_gpio.h
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
#include "adi_adrv9001_gpio.h"

/* ADI specific header files */
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_analog_tx_mem_map.h"
#include "adrv9001_bf_nvs_regmap_tx.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_gpio.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_shared_resource_manager.h"
#include "adrv9001_bf_nvs_regmap_core_2.h"

/* Header files related to libraries */

/* System header files */


#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_gpio.c"
#endif

/*********************************************************************************************************/
static int32_t adrv9001_TemperatureGetValidate(adi_adrv9001_Device_t *device,
                                                     int16_t *temperature_C)
{
    ADI_NULL_PTR_RETURN(&device->common, temperature_C);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Temperature_Get(adi_adrv9001_Device_t *device, int16_t *temperature_C)
{
    uint8_t channelMask = 0;
    uint8_t armExtData[2] = { 0 };
    uint8_t armReadBack[2] = { 0 };

    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;

    ADI_PERFORM_VALIDATION(adrv9001_TemperatureGetValidate, device, temperature_C);

    *temperature_C = 0;

    /* Channel mask is not used */
    armExtData[0] = channelMask;
    armExtData[1] = ADRV9001_ARM_OBJECTID_TEMP_SENSOR;

    /* send ARM GET opcode */
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, armExtData, sizeof(armExtData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        armExtData[1],
        ADI_ADRV9001_READ_TEMP_SENSOR_TIMEOUT_US,
        ADI_ADRV9001_READ_TEMP_SENSOR_INTERVAL_US);

    /* read the ARM memory to get temperature */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read, device, ADRV9001_ADDR_ARM_MAILBOX_GET, armReadBack,
                   sizeof(armReadBack), ARM_MEM_READ_AUTOINCR)

    /* Reconstruct temperature */
    *temperature_C = (int16_t)(((int16_t)armReadBack[0] << 0) |
                               ((int16_t)armReadBack[1] << 8));

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpIntHandler(adi_adrv9001_Device_t *device, adi_adrv9001_gpIntStatus_t *gpIntStatus)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    static const uint32_t GPINT_MASK_ALL_INTERRUPTS = 0xFFFFFFFF;

    ADI_API_ENTRY_PTR_EXPECT(device, gpIntStatus);

    /* retrieve the general purpose interrupt bitfield value */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfGet(device, ADRV9001_BF_CORE_1, &gpIntStatus->gpIntSaveIrqMask);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* mask all general purpose interrupt IRQs */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfSet(device, ADRV9001_BF_CORE_1, GPINT_MASK_ALL_INTERRUPTS);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* retrieve general purpose interrupt status word */
    recoveryAction = adrv9001_GpInterruptsStatusWordBfGet(device, ADRV9001_BF_CORE_1, &gpIntStatus->gpIntStatus);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* retrieve the general purpose interrupt IRQ mask */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfGet(device, ADRV9001_BF_CORE_1, &gpIntStatus->gpIntMask);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Mask for active sources */
    gpIntStatus->gpIntActiveSources = (gpIntStatus->gpIntStatus & ~gpIntStatus->gpIntMask);

    /* call the gp handler */
    recoveryAction = adrv9001_GpIntHandler(device, gpIntStatus);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Reload the stored gpInt IRQ mask to its original value when this API was entered */
    /* mask all general purpose interrupt IRQs */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfSet(device, ADRV9001_BF_CORE_1, gpIntStatus->gpIntSaveIrqMask);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, device->common.error.errCode, recoveryAction, NULL, device->common.error.errormessage);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, recoveryAction, gpIntStatus, "Failure setting gpIntStatus");

    return device->common.error.newAction;
}

int32_t adi_adrv9001_GpIntMaskSet(adi_adrv9001_Device_t *device, adi_adrv9001_gpMaskSelect_e maskSelect, adi_adrv9001_gpMaskArray_t *maskArray)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_EXPECT(device, maskArray);

    /* range check channel enum */
    if (maskSelect != ADI_ADRV9001_GPINT)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         maskSelect,
                         "Channel provided is out of range");

        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* retrieve the general purpose interrupt Pin IRQ mask */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfSet(device, ADRV9001_BF_CORE_1, maskArray->gpIntMask);

    ADI_ERROR_REPORT(&device->common,
                     ADI_COMMON_ERRSRC_API,
                     device->common.error.errCode,
                     recoveryAction,
                     maskArray,
                    "Error while trying to set adrv9001_GpInterruptsMaskPinBfSet");

    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpIntMaskGet(adi_adrv9001_Device_t *device, adi_adrv9001_gpMaskSelect_e maskSelect, adi_adrv9001_gpMaskArray_t *maskArray)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_EXPECT(device, maskArray);

    /* range check channel enum */
    if (maskSelect != ADI_ADRV9001_GPINT)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         maskSelect,
                         "Channel provided is out of range");

        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* retrieve the general purpose interrupt Pin IRQ mask */
    recoveryAction = adrv9001_GpInterruptsMaskPinBfGet(device, ADRV9001_BF_CORE_1, &maskArray->gpIntMask);

    ADI_ERROR_REPORT(&device->common,
                     ADI_COMMON_ERRSRC_API,
                     device->common.error.errCode,
                     recoveryAction,
                     maskArray,
                     "Error while trying to get adrv9001_GpInterruptsMaskPinBfGet");

    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpIntStatusGet(adi_adrv9001_Device_t *device, uint32_t *gpIntStatus)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_EXPECT(device, gpIntStatus);

    /* retrieve the general purpose interrupt Pin IRQ mask */
    recoveryAction = adrv9001_GpInterruptsStatusWordBfGet(device, ADRV9001_BF_CORE_1, gpIntStatus);

    ADI_ERROR_REPORT(&device->common,
                     ADI_COMMON_ERRSRC_API,
                     device->common.error.errCode,
                     recoveryAction,
                     gpIntStatus,
                     "Error while trying to get GP Int Status - Silicon A");

    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutEnGet(adi_adrv9001_Device_t *device, uint16_t *gpioOutEn)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gpioOutEn);

    ADI_MSG_EXPECT("Error attempting to read GPIO Direction Control bitfields",
        adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfGet, device, ADRV9001_BF_CORE, gpioOutEn);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioInputDirSet(adi_adrv9001_Device_t *device, uint16_t gpioInputMask)
{
    uint16_t gpioOutEn = 0;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Read the current status of GPIO direction control of ADRV9001 */
    ADI_MSG_EXPECT("Error reading current ADRV9001 GPIO OE while attempting to set GPIO input mask",
        adi_adrv9001_GpioOutEnGet, device, &gpioOutEn);

    /* To set the GPIOs as input, 0 has to be written to the corresponding GPIO direction control bitfield */
    gpioOutEn &= ~gpioInputMask;
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO OE while attempting to set GPIO input direction control",
        adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfSet,
        device,
        ADRV9001_BF_CORE,
        gpioOutEn);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioAnalogInputDirSet(adi_adrv9001_Device_t* device, uint16_t gpioAnalogInputMask)
{
    static const uint16_t GPIO_OUTPUT_ENABLE_MASK = 0x0FFF;
    uint16_t gpioAnalogOutEn = 0;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Check that GPIO Analog Input Mask is within range */
    ADI_RANGE_CHECK(device, gpioAnalogInputMask, 0, GPIO_OUTPUT_ENABLE_MASK);

    /* Read the current status of GPIO direction control of ADRV9001 */
    ADI_MSG_EXPECT("Error reading current ADRV9001 Analog GPIO OE while attempting to set GPIO_ANA input direction control",
        adrv9001_NvsRegmapCore1NvsGpioAnalogDirectionControlOeBfGet,
        device,
        ADRV9001_BF_CORE_1,
        &gpioAnalogOutEn);

    /* To set the GPIOs as input, 0 has to be written to the corresponding GPIO direction control bitfield */
    gpioAnalogOutEn &= ~gpioAnalogInputMask;
    ADI_MSG_EXPECT("Error writing ADRV9001 Analog GPIO OE while attempting to set GPIO_ANA input direction control",
        adrv9001_NvsRegmapCore1NvsGpioAnalogDirectionControlOeBfSet,
        device,
        ADRV9001_BF_CORE_1,
        gpioAnalogOutEn);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioAnalogOutputDirSet(adi_adrv9001_Device_t* device, uint16_t gpioAnalogOutputMask)
{
    static const uint16_t GPIO_OUTPUT_ENABLE_MASK = 0x0FFF;
    uint16_t gpioAnalogOutEn = 0;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Check that GPIO Analog Output Mask is within range */
    ADI_RANGE_CHECK(device, gpioAnalogOutputMask, 0, GPIO_OUTPUT_ENABLE_MASK);

    /* Read the current status of GPIO direction control of ADRV9001 */
    ADI_MSG_EXPECT("Error reading current ADRV9001 Analog GPIO OE while attempting to set GPIO_ANA output mask",
        adrv9001_NvsRegmapCore1NvsGpioAnalogDirectionControlOeBfGet,
        device,
        ADRV9001_BF_CORE_1,
        &gpioAnalogOutEn);

    /* To set the GPIOs as output, 1 has to be written to the corresponding GPIO direction control bitfield */
    gpioAnalogOutEn |= gpioAnalogOutputMask;
    ADI_MSG_EXPECT("Error writing ADRV9001 Analog GPIO OE while attempting to set GPIO_ANA output direction control",
        adrv9001_NvsRegmapCore1NvsGpioAnalogDirectionControlOeBfSet,
        device,
        ADRV9001_BF_CORE_1,
        gpioAnalogOutEn);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutputDirSet(adi_adrv9001_Device_t *device, uint16_t gpioOutputMask)

{
    uint16_t gpioOutEn = 0;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Read the current status of GPIO direction control of ADRV9001 */
    ADI_MSG_EXPECT("Error reading current ADRV9001 GPIO OE while attempting to set GPIO output mask",
        adi_adrv9001_GpioOutEnGet,
        device,
        &gpioOutEn);

    /* To set the GPIOs as output, 1 has to be written to the corresponding GPIO direction control bitfield */
    gpioOutEn |= gpioOutputMask;

    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO OE while attempting to set GPIO output direction control",
        adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfSet,
        device,
        ADRV9001_BF_CORE,
        gpioOutEn);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutSourceCtrlSet(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOutSourceCtrl_t *gpioSrcCtrl)
{
    uint8_t regData = 0;
    static const uint32_t GPIO_SRC_SEL_MASK = 0x0000003F;

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gpioSrcCtrl);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin1Pin0) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio10SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin3Pin2) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio32SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin5Pin4) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio54SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin7Pin6) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio76SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin9Pin8) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio98SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin11Pin10) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1110SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin13Pin12) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1312SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    regData = (uint8_t)((gpioSrcCtrl->gpioPin15Pin14) & GPIO_SRC_SEL_MASK);
    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1514SourceSelBfSet,
        device,
        ADRV9001_BF_CORE,
        regData);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutSourceCtrlGet(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOutSourceCtrl_t *gpioSrcCtrl)
{
    static const uint8_t GPIO_SRC_SEL_MASK = 0x3F;
    uint8_t regData[8] = { 0 };

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gpioSrcCtrl);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio10SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[0]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio32SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[1]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio54SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[2]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio76SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[3]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio98SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[4]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1110SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[5]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1312SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[6]);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Source Control",
        adrv9001_NvsRegmapCoreGpio1514SourceSelBfGet,
        device,
        ADRV9001_BF_CORE,
        &regData[7]);


    /* reconstructing byte reads into gpioSrcCtrl word */
    gpioSrcCtrl->gpioPin15Pin14 = (regData[7] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin13Pin12 = (regData[6] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin11Pin10 = (regData[5] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin9Pin8 = (regData[4] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin7Pin6 = (regData[3] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin5Pin4 = (regData[2] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin3Pin2 = (regData[1] & GPIO_SRC_SEL_MASK);
    gpioSrcCtrl->gpioPin1Pin0 = (regData[0] & GPIO_SRC_SEL_MASK);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutPinLevelSet(adi_adrv9001_Device_t *device, uint16_t gpioOutPinLevel)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);


    ADI_MSG_EXPECT("Error writing ADRV9001 GPIO Output Pin Level",
        adrv9001_NvsRegmapCoreNvsGpioSpiSourceBfSet,
        device,
        ADRV9001_BF_CORE,
        gpioOutPinLevel);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioOutPinLevelGet(adi_adrv9001_Device_t *device, uint16_t *gpioOutPinLevel)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gpioOutPinLevel);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Output Pin Level",
        adrv9001_NvsRegmapCoreNvsGpioSpiSourceBfGet,
        device,
        ADRV9001_BF_CORE,
        gpioOutPinLevel);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioInputPinLevelGet(adi_adrv9001_Device_t *device, uint16_t *gpioInPinLevel)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gpioInPinLevel);

    ADI_MSG_EXPECT("Error reading ADRV9001 GPIO Output Pin Level",
        adrv9001_NvsRegmapCoreNvsGpioSpiReadBfGet,
        device,
        ADRV9001_BF_CORE,
        gpioInPinLevel);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioMonitorOutSrcSet(adi_adrv9001_Device_t *device, uint8_t monitorIndex)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_MSG_EXPECT("Error writing GPIO Monitor Index",
        adrv9001_NvsRegmapCore1ControlOutMuxSelBfSet,
        device,
        ADRV9001_BF_CORE_1,
        monitorIndex);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_GpioMonitorOutSrcGet(adi_adrv9001_Device_t *device, uint8_t *monitorIndex)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, monitorIndex);

    ADI_MSG_EXPECT("Error reading GPIO Monitor Index",
        adrv9001_NvsRegmapCore1ControlOutMuxSelBfGet,
        device,
        ADRV9001_BF_CORE_1,
        monitorIndex);

    ADI_API_RETURN(device);
}
