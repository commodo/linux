/**
 * \file
 * \brief Contains ADRV9001 radio control related private function implementations
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adrv9001_radioctrl.h"
#include "adrv9001_gpio.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_shared_resource_manager.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_gpio.h"

int32_t adrv9001_PllGpInterruptMaskGet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t *pllGpInterruptMask1)
{
    uint8_t bfValue = 0;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, pllGpInterruptMask1);

    switch (pllNameSel)
    {
        case(ADI_ADRV9001_LO1_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRfSynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        case(ADI_ADRV9001_LO2_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRf2SynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        case(ADI_ADRV9001_AUX_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskAuxSynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        default:
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                pllNameSel,
                "Invalid PLL encountered while attempting to retrieve PLL GP Interrupt mask");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_PllGpInterruptMaskSet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t pllGpInterruptMask1)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_EXPECT(device);

    switch (pllNameSel)
    {
        case(ADI_ADRV9001_LO1_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRfSynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        case(ADI_ADRV9001_LO2_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRf2SynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        case(ADI_ADRV9001_AUX_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskAuxSynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        default:
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                pllNameSel,
                "Invalid PLL encountered while attempting to set PLL GP Interrupt mask");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_ArmGpioPinGetValidate(adi_adrv9001_Device_t *device, 
    adi_adrv9001_ArmGpioSignal_e gpioSignalSel,
    adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMap)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, armGpioSigMap);
    
    /*Check that GPIO signal is valid*/
    ADI_RANGE_CHECK(device,
        gpioSignalSel,
        0x00,
        ADI_ADRV9001_GPIO_NUM_SIGNALS-1);    
    
    ADI_API_RETURN(device);
}

int32_t adrv9001_ArmGpioPinGet(adi_adrv9001_Device_t *device,
    adi_adrv9001_ArmGpioSignal_e gpioSignalSel,
    adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMap)
{
    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;
    static const uint8_t ARM_GPIO_MASK = 0x1F;
    static const uint8_t GPIO_CONTROL_SHIFT = 1;
    static const uint8_t GPIO_CONTROL_MASK  = 0x02;
    static const uint8_t GPIO_POLARITY_SHIFT    = 0;
    static const uint8_t GPIO_POLARITY_MASK = 0x01;
    
    uint8_t extData[3] = { 0 };
    uint8_t armData[2] = { 0 };

    /* Check device pointer is not null */
    ADI_PERFORM_VALIDATION(adrv9001_ArmGpioPinGetValidate, device, gpioSignalSel, armGpioSigMap);

    /* Command ARM to return the currently assigned GPIO for the requested signal ID */
    extData[0] = 0;
    extData[1] = ADRV9001_ARM_OBJECTID_GPIO_CTRL;
    extData[2] = (uint8_t)armGpioSigMap->gpioSignalSel;
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));
 
    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        extData[1],
        ADI_ADRV9001_GETARMGPIO_TIMEOUT_US,
        ADI_ADRV9001_GETARMGPIO_INTERVAL_US);

    /* Read GPIO Pin sel from ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read, device, (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_GET, &armData[0], sizeof(armData), ARM_MEM_READ_AUTOINCR);

    /* Return the GPIO data read back from ARM mailbox */
    armGpioSigMap->gpioPinSel = (adi_adrv9001_ArmGpioPin_e)(armData[0] & ARM_GPIO_MASK);
    armGpioSigMap->control = (adi_adrv9001_ArmGpioControl_e)((armData[1] & GPIO_CONTROL_MASK) >> GPIO_CONTROL_SHIFT);    
    armGpioSigMap->polarity = (adi_adrv9001_ArmGpioPolarity_e)((armData[1] & GPIO_POLARITY_MASK) >> GPIO_POLARITY_SHIFT);

    ADI_API_RETURN(device);
}

int32_t adrv9001_ArmGpioPinSetValidate(adi_adrv9001_Device_t *device, adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMap)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, armGpioSigMap);
    
    /*Check that GPIO signal is valid*/
    ADI_RANGE_CHECK(device, armGpioSigMap->gpioSignalSel, 0x00, ADI_ADRV9001_GPIO_NUM_SIGNALS - 1);
    
    /*Check that GPIO pin is valid*/
    ADI_RANGE_CHECK(device, armGpioSigMap->gpioPinSel, ADI_ADRV9001_DGPIO_01, ADI_ADRV9001_GPIO_UNASSIGNED);
    
    /*Check that GPIO control is valid*/
    ADI_RANGE_CHECK(device, armGpioSigMap->control, ADI_ADRV9001_GPIO_CONTROL_BBIC, ADI_ADRV9001_GPIO_CONTROL_ARM);
    
    /*Check that GPIO polarity is valid*/
    ADI_RANGE_CHECK(device, armGpioSigMap->polarity, ADI_ADRV9001_GPIO_POLARITY_NORMAL, ADI_ADRV9001_GPIO_POLARITY_INVERTED);
    
    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceGet(adi_adrv9001_Device_t* device,
    adi_adrv9001_ArmGpioPin_e gpioPinSel,
    adrv9001_SharedResourceType_e *sharedResourceType,
    int32_t *sharedResource)
{
    static const adrv9001_SharedResourceLut_t sharedResourceLut[] = {
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_00_DO_NOT_ASSIGN, ADRV9001_GPIO_00_DO_NOT_ASSIGN },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_01, ADRV9001_GPIO_01 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_02, ADRV9001_GPIO_02 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_03, ADRV9001_GPIO_03 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_04, ADRV9001_GPIO_04 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_05, ADRV9001_GPIO_05 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_06, ADRV9001_GPIO_06 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_07, ADRV9001_GPIO_07 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_08, ADRV9001_GPIO_08 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_09, ADRV9001_GPIO_09 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_10, ADRV9001_GPIO_10 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_11, ADRV9001_GPIO_11 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_12, ADRV9001_GPIO_12 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_13, ADRV9001_GPIO_13 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_14, ADRV9001_GPIO_14 },
        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_15, ADRV9001_GPIO_15 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_00, ADRV9001_GPIO_ANA_00 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_01, ADRV9001_GPIO_ANA_01 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_02, ADRV9001_GPIO_ANA_02 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_03, ADRV9001_GPIO_ANA_03 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_04, ADRV9001_GPIO_ANA_04 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_05, ADRV9001_GPIO_ANA_05 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_06, ADRV9001_GPIO_ANA_06 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_07, ADRV9001_GPIO_ANA_07 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_08, ADRV9001_GPIO_ANA_08 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_09, ADRV9001_GPIO_ANA_09 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_10, ADRV9001_GPIO_ANA_10 },
        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_11, ADRV9001_GPIO_ANA_11 }
    };
    /*****************Please add lut val for shared resources above this line*******************/

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, sharedResourceType);    
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, sharedResource);    
    
    /*Check that GPIO pin is valid*/
    ADI_RANGE_CHECK(device, gpioPinSel, ADI_ADRV9001_DGPIO_01, ADI_ADRV9001_GPIO_UNASSIGNED); 
    
    ADI_RANGE_CHECK(device, gpioPinSel, 0, sizeof(sharedResourceLut)); 
    
    *sharedResourceType = sharedResourceLut[gpioPinSel].sharedResourceType;
    *sharedResource = sharedResourceLut[gpioPinSel].sharedResource;
    
    ADI_API_RETURN(device);
}

int32_t adrv9001_ArmGpioPinSet(adi_adrv9001_Device_t *device, adi_adrv9001_ArmGpioSignalMap_t *armGpioSigMap)
{
#define NUM_GPIOS_PER_CMD 1U
    static const uint8_t GPIO_ENABLE = 0x04;
    static const uint8_t GPIO_DISABLE = 0x00;
    static const uint8_t GPIO_CONTROL_BBIC = 0x00;
    static const uint8_t GPIO_CONTROL_ARM  = 0x02;
    static const uint8_t GPIO_POLARITY_NORMAL = 0x00;
    static const uint8_t GPIO_POLARITY_INVERTED = 0x01;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t resourceAcqReleaseStatus = false;
    int32_t currentGpioPinSel = 0;
    adrv9001_SharedResourceType_e sharedResourceType = ADRV9001_SHARED_RESOURCE_GPIO;  
    int32_t sharedResourceArr[NUM_GPIOS_PER_CMD] = { 0 };
    uint32_t gpioPinMask = 0;
    uint8_t extData[5] = { 0 };
    uint8_t cmdStatusByte = 0;
    adi_adrv9001_ArmGpioSignalMap_t armGpioSigMapRead = { 0 };
    
    /* Check device pointer is not null */
    ADI_PERFORM_VALIDATION(adrv9001_ArmGpioPinSetValidate, device, armGpioSigMap);    
    
    /* Retrieve the currently assigned GPIO */
    ADI_EXPECT(adrv9001_ArmGpioPinGet, device, armGpioSigMap->gpioSignalSel, &armGpioSigMapRead);
    
    /* Disassociate currently assigned GPIO to the signal and release the shared resource */
    if ((armGpioSigMapRead.gpioPinSel > ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN) &&
        (armGpioSigMapRead.gpioPinSel < ADI_ADRV9001_GPIO_UNASSIGNED))
    {
        /* Command ARM to disassociate the currently assigned GPIO for the requested signal ID */
        extData[0] = 0;
        extData[1] = ADRV9001_ARM_OBJECTID_GPIO_CTRL;
        extData[2] = (uint8_t)armGpioSigMapRead.gpioSignalSel;
        extData[3] = (uint8_t)armGpioSigMapRead.gpioPinSel;
        extData[4] = (GPIO_CONTROL_BBIC | GPIO_POLARITY_NORMAL | GPIO_DISABLE);

        ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData)) ;
        
        /* Wait for command to finish executing */
        ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
            ADRV9001_ARM_SET_OPCODE,
            extData[1],
            ADI_ADRV9001_SETARMGPIO_TIMEOUT_US,
            ADI_ADRV9001_SETARMGPIO_INTERVAL_US);
 
        ADI_EXPECT(adrv9001_SharedResourceGet,
            device,
            armGpioSigMapRead.gpioPinSel,
            &sharedResourceType,
            &currentGpioPinSel);
        
        /* Release the shared GPIO resource */
        sharedResourceArr[0] = (int32_t)currentGpioPinSel;
        recoveryAction = adrv9001_SharedResourcesRelease(device,
            sharedResourceType,
            &sharedResourceArr[0],
            NUM_GPIOS_PER_CMD,
            ADRV9001_FEATURE_ARM_GPIO_PIN,
            &resourceAcqReleaseStatus);

        if (resourceAcqReleaseStatus == false)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                currentGpioPinSel,
                "Unable to release GPIO pin currently assigned to requested signal ID. Please check if it is in use by another feature");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

        ADI_ERROR_RETURN(device->common.error.newAction);

        /* Set the direction of Released GPIO to input */
        gpioPinMask = ((uint32_t)0x00000001 << (uint8_t)currentGpioPinSel);
        
        if (sharedResourceType == ADRV9001_SHARED_RESOURCE_GPIO)
        {
            ADI_EXPECT(adi_adrv9001_GpioInputDirSet, device, gpioPinMask);
        }
        else
        {
            ADI_EXPECT(adi_adrv9001_GpioAnalogInputDirSet, device, gpioPinMask) ;
        }
    }

    /* Associate requested signal ID with the GPIO pin selected and acquire the shared GPIO resource */
    if (armGpioSigMap->gpioPinSel < ADI_ADRV9001_GPIO_UNASSIGNED)
    {
        ADI_EXPECT(adrv9001_SharedResourceGet,
            device,
            armGpioSigMap->gpioPinSel,
            &sharedResourceType,
            &currentGpioPinSel);
        
        /* Acquire shared GPIO resource */
        sharedResourceArr[0] = currentGpioPinSel;
        recoveryAction = adrv9001_SharedResourcesAcquire(device,
            sharedResourceType,
            &sharedResourceArr[0],
            NUM_GPIOS_PER_CMD,
            ADRV9001_FEATURE_ARM_GPIO_PIN,
            &resourceAcqReleaseStatus);

        if (resourceAcqReleaseStatus == false)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                armGpioSigMap->gpioPinSel,
                "Unable to acquire GPIO pin currently assigned to requested signal ID. Please check if it is in use by another feature");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

        ADI_ERROR_RETURN(device->common.error.newAction);

        /* Set the direction of Acquired GPIO to input */
        gpioPinMask = ((uint32_t)0x00000001 << (uint8_t)currentGpioPinSel);
        if (sharedResourceType == ADRV9001_SHARED_RESOURCE_GPIO)
        {
            ADI_EXPECT(adi_adrv9001_GpioInputDirSet, device, gpioPinMask);
        }
        else
        {
            ADI_EXPECT(adi_adrv9001_GpioAnalogInputDirSet, device, gpioPinMask);
        }

        /* Command ARM to associate the currently assigned GPIO for the requested signal ID */
        extData[0] = 0;
        extData[1] = ADRV9001_ARM_OBJECTID_GPIO_CTRL;
        extData[2] = (uint8_t)armGpioSigMap->gpioSignalSel;
        extData[3] = (uint8_t)armGpioSigMap->gpioPinSel;
        extData[4] = GPIO_ENABLE;

        if (armGpioSigMap->control ==  ADI_ADRV9001_GPIO_CONTROL_ARM)
        {
            extData[4] |= GPIO_CONTROL_ARM;
        }
        else
        {
            extData[4] &= ~GPIO_CONTROL_ARM;
        }
        
        if (armGpioSigMap->polarity ==  ADI_ADRV9001_GPIO_POLARITY_INVERTED)
        {
            extData[4] |= GPIO_POLARITY_INVERTED;
        }
        else
        {
            extData[4] &= ~GPIO_POLARITY_INVERTED;
        }

        recoveryAction = adi_adrv9001_arm_Cmd_Write(device, ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* Wait for command to finish executing */
        recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(device, NULL, (uint8_t)ADRV9001_ARM_SET_OPCODE, &cmdStatusByte, (uint32_t)ADI_ADRV9001_SETARMGPIO_TIMEOUT_US, (uint32_t)ADI_ADRV9001_SETARMGPIO_INTERVAL_US);

        if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
        {
            /* If cmdStatusByte is non-zero then flag an ARM error and release the acquired shared resource */
            if ((cmdStatusByte >> 1) > 0)
            {
                recoveryAction = adrv9001_SharedResourcesRelease(device,
                                                                 sharedResourceType,
                                                                 &sharedResourceArr[0],
                                                                 NUM_GPIOS_PER_CMD,
                                                                 ADRV9001_FEATURE_ARM_GPIO_PIN,
                                                                 &resourceAcqReleaseStatus);

                if (resourceAcqReleaseStatus == false)
                {
                    ADI_ERROR_REPORT(&device->common,
                                     ADI_COMMON_ERRSRC_API,
                                     ADI_COMMON_ERR_INV_PARAM,
                                     ADI_ADRV9001_ACT_ERR_RESET_ARM,
                                     armGpioSigMap->gpioPinSel,
                                     "Unable to release currently acquired GPIO shared resource for ARM GPIO functionality due to an ARM error");
                    ADI_ERROR_RETURN(device->common.error.newAction);
                }

                ADI_ERROR_RETURN(device->common.error.newAction);

                /* Set the direction of Released GPIO to input */
                gpioPinMask = ((uint32_t)0x00000001 << (uint8_t)currentGpioPinSel);
                if (sharedResourceType == ADRV9001_SHARED_RESOURCE_GPIO)
                {
                    ADI_EXPECT(adi_adrv9001_GpioInputDirSet, device, gpioPinMask);
                }
                else
                {
                    ADI_EXPECT(adi_adrv9001_GpioAnalogInputDirSet, device, gpioPinMask);
                }
 
                ADI_EXPECT(adrv9001_ArmCmdErrorHandler, 
                           device,
                           ADRV9001_ARMCMD_ERRCODE(ADRV9001_ARM_SET_OPCODE, extData[0], cmdStatusByte));
            }
        }
    }

    ADI_API_RETURN(device);
}

