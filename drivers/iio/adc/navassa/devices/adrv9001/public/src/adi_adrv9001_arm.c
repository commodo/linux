/**
* \file
* \brief Contains ARM features related function implementation defined in
* adi_adrv9001_arm.h
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
#include "adi_adrv9001_arm.h"

/* ADI specific header files */
#include "adi_adrv9001.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_radio.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_arm_error_mapping.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"

/* Header files related to libraries */


/* System header files */


#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_arm.c"
#endif

/*********************************************************************************************************/
int32_t adi_adrv9001_arm_Enable(adi_adrv9001_Device_t *device)
{
    uint8_t armCtl1 = 0;

    ADI_API_ENTRY_EXPECT(device);

    ADI_EXPECT(adi_adrv9001_arm_AhbSpiBridge_Enable, device);

    ADRV9001_SPIREADBYTE(device, "ARM_CTL_1", ADRV9001_ADDR_ARM_CTL_1, &armCtl1);

    armCtl1 = armCtl1 | ADRV9001_AC1_ARM_M3_RUN;
    ADRV9001_SPIWRITEBYTE(device, "ARM_CTL_1", ADRV9001_ADDR_ARM_CTL_1, armCtl1);

    // Issue SW interrupt 4 to wake up ARM
    ADI_EXPECT(adi_adrv9001_arm_WakeupInterrupt_Set, device);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Disable(adi_adrv9001_Device_t *device)
{
    uint8_t armCtl1 = 0;

    ADI_API_ENTRY_EXPECT(device);

    /* TODO: Implement ISR to service ARM READY_FOR_MCS */

    ADRV9001_SPIREADBYTE(device, "ARM_CTL_1", ADRV9001_ADDR_ARM_CTL_1, &armCtl1);

    armCtl1 = (armCtl1 & (~ADRV9001_AC1_ARM_M3_RUN));
    ADRV9001_SPIWRITEBYTE(device, "ARM_CTL_1", ADRV9001_ADDR_ARM_CTL_1, armCtl1);

    ADI_EXPECT(adi_adrv9001_arm_AhbSpiBridge_Disable, device);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_AhbSpiBridge_Enable(adi_adrv9001_Device_t *device)
{
    uint8_t ahbSpiReg = 0;

    ADI_API_ENTRY_EXPECT(device);

    ADRV9001_SPIREADBYTE(device, "AHB_SPI_BRIDGE", ADRV9001_ADDR_AHB_SPI_BRIDGE, &ahbSpiReg);

    ahbSpiReg = ahbSpiReg | ADRV9001_AHB_SPI_BRIDGE_ENABLE;
    ADRV9001_SPIWRITEBYTE(device, "AHB_SPI_BRIDGE", ADRV9001_ADDR_AHB_SPI_BRIDGE, ahbSpiReg);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_AhbSpiBridge_Disable(adi_adrv9001_Device_t *device)
{
    uint8_t ahbSpiReg = 0;

    ADI_API_ENTRY_EXPECT(device);

    ADRV9001_SPIREADBYTE(device, "AHB_SPI_BRIDGE", ADRV9001_ADDR_AHB_SPI_BRIDGE, &ahbSpiReg);

    ahbSpiReg = (ahbSpiReg & (~ADRV9001_AC1_ARM_M3_RUN));
    ADRV9001_SPIWRITEBYTE(device, "AHB_SPI_BRIDGE", ADRV9001_ADDR_AHB_SPI_BRIDGE, ahbSpiReg);

    ADI_API_RETURN(device);
}


int32_t adi_adrv9001_arm_StartStatus_Check(adi_adrv9001_Device_t *device, uint32_t timeout_us)
{
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t waitInterval_us = 0;
    uint32_t numEventChecks = 1;
    uint32_t eventCheck = 0;
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t armDebugLoaded = ((device->devStateInfo.devState & ADI_ADRV9001_STATE_ARM_DEBUG_LOADED) == ADI_ADRV9001_STATE_ARM_DEBUG_LOADED) ? 1 : 0;

    ADI_API_ENTRY_EXPECT(device);

    /* Wait for ARM to exit BOOTUP state */
    waitInterval_us = (ADI_ADRV9001_GETARMBOOTUP_INTERVAL_US > timeout_us) ?
        timeout_us : ADI_ADRV9001_GETARMBOOTUP_INTERVAL_US;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);


    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &state);

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest)
        {
            state.bootState = device->devStateInfo.swTest;
        }
#endif
        if ((state.bootState == ADI_ADRV9001_ARM_BOOT_POWERUP) ||
            ((armDebugLoaded == 1) && (state.bootState == ADI_ADRV9001_ARM_BOOT_JTAG_BUILD_STATUS_READY)))
        {
            /* Wait for ARM to boot */
            halError = adi_common_hal_Wait_us(&device->common, waitInterval_us);
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             halError,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             device,
                             "Timer not working");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else if ((armDebugLoaded == 0) && (state.bootState == ADI_ADRV9001_ARM_BOOT_JTAG_BUILD_STATUS_READY))
        {
            device->devStateInfo.devState |= ADI_ADRV9001_STATE_ARM_DEBUG_LOADED;
            ADI_API_RETURN(device);
        }
        else if (state.bootState == ADI_ADRV9001_ARM_BOOT_READY)
        {
            break; /* Valid case - ARM booted successfully */
        }
        else if (state.bootState <= ADI_ADRV9001_ARM_BOOT_CLKGEN_RCAL_ERR)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_ADRV9001_SRC_ARMFWSTATUS,
                             state.bootState,
                             ADI_ADRV9001_ACT_ERR_RESET_ARM,
                             NULL,
                             adrv9001_error_table_ArmBootStatus[state.bootState]);
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_ADRV9001_SRC_ARMFWSTATUS,
                             state.bootState,
                             ADI_ADRV9001_ACT_ERR_RESET_ARM,
                             NULL,
                             "ARM Unknown error during bootup");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    if (state.bootState != ADI_ADRV9001_ARM_BOOT_READY)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_ADRV9001_SRC_ARMFWSTATUS,
                         state.bootState,
                         ADI_ADRV9001_ACT_ERR_RESET_ARM,
                         device,
                         "Timed out waiting for ARM bootup to happen");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

/* FIXME: Disabled Checksum verification as FW code disabled checksum calculation.
          It may have to enabled in future*/
#if 0
    adi_adrv9001_ChecksumTable_t checksum = { { 0 } };
    uint8_t checksumValid = 0;

    if (armDebugLoaded == 0)
    {
        recoveryAction = adi_adrv9001_arm_ChecksumTable_Get(device, &checksum, &checksumValid);
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    device->devStateInfo.devState = (adi_adrv9001_ApiStates_e)(device->devStateInfo.devState | ADI_ADRV9001_STATE_ARM_LOADED);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Profile_Write(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_EXPECT(device, init);

    recoveryAction = adrv9001_ArmProfileWrite(device, init);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmProfileWrite Failed");
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

int32_t adi_adrv9001_arm_PfirProfiles_Write(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_EXPECT(device, init);

    recoveryAction = adrv9001_PfirProfilesWrite(device, init);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "adrv9001_PfirProfilesWrite Failed");
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

int32_t adi_adrv9001_arm_Image_Write(adi_adrv9001_Device_t *device, uint32_t byteOffset, const uint8_t binary[], uint32_t byteCount)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t stackPtr[4] = { 0 };
    uint8_t bootAddr[4] = { 0 };
    uint32_t i = 0;

    static const uint32_t ADRV9001_ADDR_DEVICE_PROFILE_OFFSET      = 0x00000504;
    static const uint32_t ADRV9001_ADDR_PFIR_PROFILE_BUFFER_OFFSET = 0x00000508;

    ADI_API_ENTRY_PTR_EXPECT(device, binary);

    if ((byteCount % 4) > 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            byteCount,
            "Invalid byteCount. Must be multiple of 4.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if ((byteOffset % 4) > 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            byteCount,
            "Invalid byteOffset. Must be multiple of 4.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (byteOffset == 0)
    {
        if (byteCount < 8)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                byteCount,
                "Invalid byteCount. Must be larger than 8.");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

        /* extraction of stack pointer and boot address from top of array */
        stackPtr[0] = binary[0];
        stackPtr[1] = binary[1];
        stackPtr[2] = binary[2];
        stackPtr[3] = binary[3];

        bootAddr[0] = binary[4];
        bootAddr[1] = binary[5];
        bootAddr[2] = binary[6];
        bootAddr[3] = binary[7];


        /*     core.arm_boot_addr_byte0/1/2/3  */
        ADRV9001_SPIWRITEBYTE(device, "ARM_BOOT_ADDR_BYTE_0", ADRV9001_ADDR_ARM_BOOT_ADDR_BYTE0, bootAddr[0]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_BOOT_ADDR_BYTE_1", ADRV9001_ADDR_ARM_BOOT_ADDR_BYTE1, bootAddr[1]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_BOOT_ADDR_BYTE_2", ADRV9001_ADDR_ARM_BOOT_ADDR_BYTE2, bootAddr[2]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_BOOT_ADDR_BYTE_3", ADRV9001_ADDR_ARM_BOOT_ADDR_BYTE3, bootAddr[3]);

        /* core.arm_stack_ptr_byte_0/1/2/3   -- */
        ADRV9001_SPIWRITEBYTE(device, "ARM_STACK_PTR_BYTE_0", ADRV9001_ADDR_ARM_STACK_PTR_BYTE0, stackPtr[0]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_STACK_PTR_BYTE_1", ADRV9001_ADDR_ARM_STACK_PTR_BYTE1, stackPtr[1]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_STACK_PTR_BYTE_2", ADRV9001_ADDR_ARM_STACK_PTR_BYTE2, stackPtr[2]);
        ADRV9001_SPIWRITEBYTE(device, "ARM_STACK_PTR_BYTE_3", ADRV9001_ADDR_ARM_STACK_PTR_BYTE3, stackPtr[3]);

    }

    if ((ADRV9001_ADDR_DEVICE_PROFILE_OFFSET >= byteOffset) &&
        (ADRV9001_ADDR_DEVICE_PROFILE_OFFSET < (byteOffset + byteCount + 4)))
    {
        /* extraction of profile address */
        i = ADRV9001_ADDR_DEVICE_PROFILE_OFFSET - byteOffset;
        device->devStateInfo.profileAddr = (((uint32_t)(binary[i + 3]) << 24) |
            (((uint32_t)binary[i + 2]) << 16) | (((uint32_t)binary[i + 1]) << 8) | ((uint32_t)binary[i]));
    }

    if ((ADRV9001_ADDR_PFIR_PROFILE_BUFFER_OFFSET >= byteOffset) &&
        (ADRV9001_ADDR_PFIR_PROFILE_BUFFER_OFFSET < (byteOffset + byteCount + 4)))
    {
        /* extraction of PFIR buffer address */
        i = ADRV9001_ADDR_PFIR_PROFILE_BUFFER_OFFSET - byteOffset;
        device->devStateInfo.pfirProfileAddr = ((((uint32_t)binary[i + 3]) << 24) |
            (((uint32_t)binary[i + 2]) << 16) | (((uint32_t)binary[i + 1]) << 8) | ((uint32_t)binary[i]));
    }

    recoveryAction = adi_adrv9001_arm_Memory_Write(device,
        ADRV9001_ADDR_ARM_START_PROG + byteOffset,
        &binary[0],
        byteCount);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmImageWrite Failed");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Memory_Read(adi_adrv9001_Device_t *device, uint32_t address, uint8_t returnData[], uint32_t byteCount, uint8_t autoIncrement)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, returnData, byteCount);

    if (!(address >= ADRV9001_ADDR_ARM_START_PROG && address <= ADRV9001_ADDR_ARM_END_PROG) &&
        !(address >= ADRV9001_ADDR_ARM_START_DATA && address <= ADRV9001_ADDR_ARM_END_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            address,
            "Invalid ARM Memory Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (!((address + byteCount - 1) >= ADRV9001_ADDR_ARM_START_PROG && (address + byteCount - 1) <= ADRV9001_ADDR_ARM_END_PROG) &&
        !((address + byteCount - 1) >= ADRV9001_ADDR_ARM_START_DATA && (address + byteCount - 1) <= ADRV9001_ADDR_ARM_END_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            address,
            "Invalid ARM Memory Address + byteCount");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adrv9001_DmaMemRead(device, address, returnData, byteCount, autoIncrement);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmMemRead failed during DMA transaction");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Memory_Write(adi_adrv9001_Device_t *device, uint32_t address, const uint8_t data[], uint32_t byteCount)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, data, byteCount);

    if (!(address >= ADRV9001_ADDR_ARM_START_PROG && address <= ADRV9001_ADDR_ARM_END_PROG) &&
        !(address >= ADRV9001_ADDR_ARM_START_DATA && address <= ADRV9001_ADDR_ARM_END_DATA))
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM,
            address, "Invalid ARM Memory Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (!((address + byteCount - 1) >= ADRV9001_ADDR_ARM_START_PROG && (address + byteCount - 1) <= ADRV9001_ADDR_ARM_END_PROG) &&
        !((address + byteCount - 1) >= ADRV9001_ADDR_ARM_START_DATA && (address + byteCount - 1) <= ADRV9001_ADDR_ARM_END_DATA))
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM,
            address, "Invalid ARM Memory Address + byteCount");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adrv9001_DmaMemWrite(device, address, data, byteCount);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmMemWrite failed during DMA transaction");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Config_Write(adi_adrv9001_Device_t *device, uint8_t objectId, uint16_t byteOffset, const uint8_t data[], uint32_t byteCount)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    uint8_t extendedData[5] = { 0 };

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, data, byteCount);

    extendedData[0] = byteCount;
    extendedData[1] = 0;
    extendedData[2] = 0;
    extendedData[3] = 0;
    extendedData[4] = 0;
    /* FIXME: Vivek - TBD on the fifth payload byte and the 'byteCount' value in adi_adrv9001_arm_Cmd_Write() function below.
     *        Adding one more byte to support Navassa*/

    recoveryAction = adi_adrv9001_arm_Memory_Write(device, ADRV9001_ADDR_ARM_MAILBOX_SET, &extendedData[0], 1);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmConfigWrite failed writing ARM memory");
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adi_adrv9001_arm_Memory_Write(device, ADRV9001_ADDR_ARM_MAILBOX_SET + 1, &data[0], byteCount);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* ARM Object id, byte offset LSB, offset MSB = 0, copy 2 bytes */
    extendedData[0] = ADRV9001_ARM_OBJECTID_CONFIG;
    extendedData[1] = objectId;
    extendedData[2] = (byteOffset & 0xFF);
    extendedData[3] = ((byteOffset >> 8) & 0xFF);
    /* FIXME: Vivek - TBD on the fifth payload byte and the 'byteCount' value in adi_adrv9001_arm_Cmd_Write() function below*/

    recoveryAction = adi_adrv9001_arm_Cmd_Write(device, ADRV9001_ARM_SET_OPCODE, &extendedData[0], 4);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_SET_OPCODE,
        extendedData[1],
        ADI_ADRV9001_WRITEARMCFG_TIMEOUT_US,
        ADI_ADRV9001_WRITEARMCFG_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Config_Read(adi_adrv9001_Device_t *device, uint8_t objectId, uint16_t byteOffset, uint8_t returnData[], uint32_t byteCount)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    uint8_t extendedData[5] = { 0 };
    static const uint8_t AUTO_INC_BIT = 0;

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, returnData, byteCount);

    extendedData[0] = byteCount;
    extendedData[1] = 0;
    extendedData[2] = 0;
    extendedData[3] = 0;
    extendedData[4] = 0;
    /* FIXME: Vivek - TBD on the fifth payload byte and the 'byteCount' value in adi_adrv9001_arm_Cmd_Write() function below
     *        Adding one more byte to support Navassa*/
    recoveryAction = adi_adrv9001_arm_Memory_Write(device, ADRV9001_ADDR_ARM_MAILBOX_GET, &extendedData[0], 1);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmConfigRead failed to write ARM mem");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* ARM Object id, byte offset LSB, offset MSB = 0, byteCount will read that number of bytes */
    extendedData[0] = ADRV9001_ARM_OBJECTID_CONFIG;
    extendedData[1] = objectId;
    extendedData[2] = (byteOffset & 0xFF);
    extendedData[3] = ((byteOffset >> 8) & 0xFF);
    /* FIXME: Vivek - TBD on the fifth payload byte and the 'byteCount' value in adi_adrv9001_arm_Cmd_Write() function below */

    /* send the read config ARM opcode */
    recoveryAction = adi_adrv9001_arm_Cmd_Write(device, ADRV9001_ARM_GET_OPCODE, &extendedData[0], 4);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* check the command status for timeout */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        extendedData[1],
        ADI_ADRV9001_READARMCFG_TIMEOUT_US,
        ADI_ADRV9001_READARMCFG_INTERVAL_US);

#if ADI_ADRV9001_SW_TEST > 0
    /* Test error */
    if (device->devStateInfo.swTest > 1)
    {
        cmdStatusByte = device->devStateInfo.swTest;
    }
#endif

    recoveryAction = adi_adrv9001_arm_Memory_Read(device,
        ADRV9001_ADDR_ARM_MAILBOX_GET,
        &returnData[0],
        byteCount,
        AUTO_INC_BIT);
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_SystemError_Get(adi_adrv9001_Device_t *device, uint8_t *objectId, uint8_t *errorCode)
{
    ADI_API_ENTRY_PTR_EXPECT(device, errorCode);
    ADI_NULL_PTR_RETURN(&device->common, objectId);

    ADRV9001_SPIREADBYTE(device, "arm_cmd_status_12", ADRV9001_ADDR_ARM_CMD_STATUS_12, errorCode);
    ADRV9001_SPIREADBYTE(device, "arm_cmd_status_13", ADRV9001_ADDR_ARM_CMD_STATUS_13, objectId);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_CmdStatus_Get(adi_adrv9001_Device_t *device, uint16_t *errorWord, uint16_t *statusWord)
{
    uint8_t i = 0;
    uint8_t bytes[8] = { 0 };
    uint8_t status = 0;
    static const uint8_t ARM_ERR_MASK_LO = 0x0E;
    static const uint8_t ARM_ERR_MASK_HI = 0xE0;
    static const uint8_t ARM_PENDING_LO = 0x01;
    static const uint8_t ARM_PENDING_HI = 0x10;
    static const uint8_t ERROR_WORD_LO = 0x0001;
    static const uint8_t ERROR_WORD_HI = 0x0002;
    static const uint8_t STATUS_WORD_SHIFT_HI = 3;
    static const uint8_t CMD_STATUS_MAX = 8;


    ADI_API_ENTRY_PTR_EXPECT(device, errorWord);

    ADI_NULL_PTR_RETURN(&device->common, statusWord);

    /* ensuring the errorWord and statusWord are cleared */
    *errorWord = 0;
    *statusWord = 0;

    /* read in the entire 64-bit status register into a byte array and parse one byte at a time */
    for (i = 0; i < CMD_STATUS_MAX; i++)
    {
        ADRV9001_SPIREADBYTE(device, "", ((uint16_t)ADRV9001_ADDR_ARM_CMD_STATUS_0 + i), &bytes[i]);

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 1)
        {
            bytes[i] = (uint8_t)device->devStateInfo.swTest;
        }
#endif

        /* assigning each pending bit from every opcode to a weighted position in statusWord */
        status = (bytes[i] & ARM_PENDING_HI) >> STATUS_WORD_SHIFT_HI;
        status |= (bytes[i] & ARM_PENDING_LO);
        *statusWord |= (uint16_t)(status << (i * 2));

        /* checking each 3-bit error field per nibble for > 0, then assigning to weighted bit in errorWord */
        if ((bytes[i] & ARM_ERR_MASK_LO) > 0)
        {
            *errorWord |= (ERROR_WORD_LO << (i * 2));
        }

        if ((bytes[i] & ARM_ERR_MASK_HI) > 0)
        {
            *errorWord |= (ERROR_WORD_HI << (i * 2));
        }
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_CmdStatusOpcode_Get(adi_adrv9001_Device_t *device, uint8_t opCode, uint8_t *cmdStatByte)
{
    uint8_t cmdByteIndex = 0;
    uint8_t cmdByte = 0;
    static const uint8_t ARM_STATUS_MASK = 0x0F;
    static const uint8_t ARM_STATUS_SHIFT_HI = 4;

    ADI_API_ENTRY_PTR_EXPECT(device, cmdStatByte);

    /* check for even-numbered opCodes including opcode 0, but must not be greater than 30 */
    if ((opCode != (uint8_t)ADRV9001_ARM_STREAM_TRIGGER_OPCODE) && ADRV9001_OPCODE_VALID(opCode))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "Invalid ARM opcode given to ArmCmdStatusOpcodeGet()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* calculating command status register offset based on opcode status packing */
    cmdByteIndex = opCode >> 2;

    /* reading the command status register for given opcode */
    ADRV9001_SPIREADBYTE(device, "", ((uint16_t)ADRV9001_ADDR_ARM_CMD_STATUS_0 + cmdByteIndex), &cmdByte);

    /* identifying nibble location in command register for given opcode */
    if (((opCode >> 1) & (uint8_t)0x01) == (uint8_t)0x01)
    {
        *cmdStatByte = ((cmdByte >> ARM_STATUS_SHIFT_HI) & ARM_STATUS_MASK);
    }
    else
    {
        *cmdStatByte = (cmdByte & ARM_STATUS_MASK);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_CmdStatus_Wait(adi_adrv9001_Device_t *device,
                                        int32_t (*callback)(void),
                                        uint8_t opCode,
                                        uint8_t *cmdStatusByte,
                                        uint32_t timeout_us,
                                        uint32_t waitInterval_us)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t eventCheck = 0;
    uint32_t numEventChecks = 0;
    static const uint8_t ARM_ERR_MASK = 0x0E;
    static const uint8_t ARM_PENDING = 0x01;

    ADI_API_ENTRY_PTR_EXPECT(device, cmdStatusByte);

    /* check for even-numbered opCodes including opcode 0, but must not be greater than 30 */
    if ((opCode != (uint8_t)ADRV9001_ARM_STREAM_TRIGGER_OPCODE) && ADRV9001_OPCODE_VALID(opCode))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "Invalid ARM opcode given to ArmCmdStatusWait()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    waitInterval_us = (waitInterval_us > timeout_us) ? timeout_us : waitInterval_us;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);

    /* timeout event check loop */
    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        if (NULL != callback)
        {
            recoveryAction = callback();
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             recoveryAction,
                             NULL,
                             "Callback function failed in adi_adrv9001_arm_CmdStatus_Wait");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        /* read status of opcode */
        recoveryAction = adi_adrv9001_arm_CmdStatusOpcode_Get(device, opCode, cmdStatusByte);
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         recoveryAction,
                         NULL,
                         "Failed to get ARM command status");
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* If error code is non zero in [3:1], - return error */
        if ((*cmdStatusByte & ARM_ERR_MASK) > 0)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_ADRV9001_SRC_ARMCMD,
                ADI_COMMON_ERR_API_FAIL,
                ADI_ADRV9001_ACT_ERR_RESET_ARM,
                device,
                "ArmCmdStatusWait() failed due to thrown ARM error. Is device in correct state for calling command?");
            ADI_ERROR_RETURN(device->common.error.newAction);

        }

        /* if pending bit is set for opcode of interest and the number of events have not expired, perform wait */
        if (((*cmdStatusByte & ARM_PENDING) > 0) &&
            (eventCheck < numEventChecks))
        {
            halError = adi_common_hal_Wait_us(&device->common, waitInterval_us);

            ADI_ERROR_REPORT(&device->common,
                ADI_ADRV9001_SRC_ARMCMD,
                halError,
                ADI_COMMON_ACT_ERR_CHECK_TIMER,
                device,
                "Timer not working in ArmCmdStatusWait()");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else
        {
            break;
        }
    }

    /* if ARM Command did not complete within the timeout period */
    if ((*cmdStatusByte & ARM_PENDING) > 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_ARMCMD,
            ADI_COMMON_ERR_API_FAIL,
            ADI_ADRV9001_ACT_ERR_RESET_ARM,
            device,
            "ArmCmdStatusWait() failed due to thrown ARM error. ARM time out ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Cmd_Write(adi_adrv9001_Device_t *device, uint8_t opCode, const uint8_t extendedData[], uint32_t byteCount)
{
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint8_t armCommandBusy = 0;
    uint8_t i = 0;
    uint32_t timeout_us = ADI_ADRV9001_SENDARMCMD_TIMEOUT_US;
    uint32_t waitInterval_us = ADI_ADRV9001_SENDARMCMD_INTERVAL_US;
    uint32_t eventCheck = 0;
    uint32_t numEventChecks = 0;
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

#define EXT_CMD_BYTE_MAX 5
    //    static const char *cmdByte[EXT_CMD_BYTE_MAX] = { "ARM_EXT_CMD_BYTE_1", "ARM_EXT_CMD_BYTE_2", "ARM_EXT_CMD_BYTE_3", "ARM_EXT_CMD_BYTE_4" };

        ADI_API_ENTRY_EXPECT(device);

    if (byteCount > 0)
    {
        ADI_NULL_PTR_RETURN(&device->common, extendedData);
    }

    /* check for even-numbered opCodes only including opcode 0, but must not be greater than opCode 30 */
    if ((opCode != (uint8_t)ADRV9001_ARM_STREAM_TRIGGER_OPCODE) && ADRV9001_OPCODE_VALID(opCode))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "Invalid ARM opcode given to ArmCmdWrite()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* the number of valid extended data bytes is from 0-4 */
    if (byteCount > EXT_CMD_BYTE_MAX)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "Invalid number of extended data in ArmCmdWrite()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* setting a 2 second timeout for mailbox busy bit to be clear (can't send an arm mailbox command until mailbox is ready) */

    waitInterval_us = (waitInterval_us > timeout_us) ? timeout_us : waitInterval_us;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);

    /* timeout event loop to permit non-blocking of thread */
    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        recoveryAction = adi_adrv9001_arm_MailboxBusy_Get(device, &armCommandBusy);
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Invalid Get for armCommandBusy");
        ADI_ERROR_RETURN(device->common.error.newAction);

#if ADI_ADRV9001_SW_TEST > 0
        /* Test error */
        if (device->devStateInfo.swTest > 0)
        {
            armCommandBusy = true;
        }
#endif
        if (armCommandBusy == true)
        {
            halError = adi_common_hal_Wait_us(&device->common, waitInterval_us);
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_ADI_HAL,
                halError,
                ADI_COMMON_ACT_ERR_CHECK_TIMER,
                device,
                "Timer not working in ArmCmdWrite()");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else
        {
            break;
        }
    }

    /* if busy bit remains set after timeout event loop function is exited, otherwise command is sent after extended bytes */
    if (armCommandBusy == true)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_ARMCMD,
            ADI_COMMON_ERR_API_FAIL,
            ADI_ADRV9001_ACT_ERR_RESET_ARM,
            device,
            "ARM Mailbox Busy. Command not executed in ArmCmdWrite()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else
    {
        if (byteCount > 0)
        {
            for (i = 0; i < byteCount; i++)
            {
                ADRV9001_SPIWRITEBYTE(device, "ARM_EXT_CMD_BYTE_1+", ((uint16_t)ADRV9001_ADDR_ARM_EXT_CMD_BYTE_1 + i), extendedData[i]);
            }
        }

        ADRV9001_SPIWRITEBYTE(device, "ARM_COMMAND", ADRV9001_ADDR_ARM_COMMAND, opCode);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Version(adi_adrv9001_Device_t *device, adi_adrv9001_ArmVersion_t *armVersion)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t ver[8] = { 0 };
    uint32_t fullVersion = 0;
    static const uint8_t ARMBUILD_DEBUG = 0x01;
    static const uint8_t ARMBUILD_TESTOBJ = 0x04;
    static const uint8_t ARMBUILD_USES_FOUR_DIGIT_VERSION = 0x01;

    ADI_API_ENTRY_PTR_EXPECT(device, armVersion);

    if ((device->devStateInfo.devState & ADI_ADRV9001_STATE_ARM_LOADED) != ADI_ADRV9001_STATE_ARM_LOADED)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_ARMCMD,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "ARM binary must be loaded before calling ArmVersionGet()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }


    recoveryAction = adi_adrv9001_arm_Memory_Read(device, ADRV9001_ADDR_ARM_VERSION, &ver[0], sizeof(ver), 0);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Failed to read ARM memory");
    ADI_ERROR_RETURN(device->common.error.newAction);
    fullVersion = (((uint32_t)ver[0]) | (((uint32_t)ver[1]) << 8) | (((uint32_t)ver[2]) << 16) | (((uint32_t)ver[3]) << 24));


#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest > 1)
    {
        fullVersion = device->devStateInfo.swTest;
    }
#endif

    if (ARMBUILD_USES_FOUR_DIGIT_VERSION)
    {
        armVersion->majorVer = (uint8_t)(fullVersion >> 28) & 0x0F;
        armVersion->minorVer = (uint8_t)(fullVersion >> 24) & 0x0F;
        armVersion->maintVer = (uint8_t)(fullVersion >> 16) & 0xFF;
        armVersion->rcVer = (uint16_t)(fullVersion & 0xFFFF);
    }
    else
    {
        armVersion->rcVer = (uint8_t)(fullVersion % 100);
        armVersion->minorVer = (uint8_t)((fullVersion / 100) % 100);
        armVersion->majorVer = (uint8_t)(fullVersion / 10000);
        armVersion->maintVer = 0;
    }

    if (ver[4] & ARMBUILD_DEBUG)
    {
        armVersion->armBuildType = ADI_ADRV9001_ARMBUILD_DEBUG;
    }
    else if (ver[4] & ARMBUILD_TESTOBJ)
    {
        armVersion->armBuildType = ADI_ADRV9001_ARMBUILD_TESTOBJ;
    }
    else
    {
        armVersion->armBuildType = ADI_ADRV9001_ARMBUILD_RELEASE;
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_ChecksumTable_Get(adi_adrv9001_Device_t *device, adi_adrv9001_ChecksumTable_t *checksum, uint8_t *checksumValid)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t buildTimeChecksum = 0x00000000;
    uint32_t calculatedChecksum = 0x00000000;
    uint8_t checkData[sizeof(adi_adrv9001_ChecksumTable_t)] = { 0 };
    uint32_t checkAddr = 0;
    uint32_t timeout_us = ADI_ADRV9001_VERIFY_ARM_CHKSUM_TIMEOUT_US;
    uint32_t waitInterval_us = ADI_ADRV9001_VERIFY_ARM_CHKSUM_INTERVAL_US;
    uint32_t numEventChecks = 1;
    uint32_t eventCheck = 0;
    int j = 0;
    int i = 0;

    static const uint8_t CHECKSUM_BYTES = 0x4;
    static const uint8_t CHECKSUMENTRYSIZE = 8;
    static const uint32_t MIN_TIMEOUT_US = 10000;

    /* Arm stream checksum order: main, rx1/2, tx1/2, orx12 */
    static const uint32_t streamChannel[] = {
        0xFFFFFFFF,
        ADI_ADRV9001_RX1,
        ADI_ADRV9001_RX2,
        ADI_CHANNEL_1 << ADI_ADRV9001_TX_INITIALIZED_CH_OFFSET,
        ADI_CHANNEL_2 << ADI_ADRV9001_TX_INITIALIZED_CH_OFFSET,
        (ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2 |
                             ADI_ADRV9001_ILB1 |
                             ADI_ADRV9001_ILB2 |
                             ADI_ADRV9001_ELB1 |
                             ADI_ADRV9001_ELB2),
    };


    ADI_API_ENTRY_PTR_EXPECT(device, checksum);

    ADI_NULL_PTR_RETURN(&device->common, checksumValid);

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest == 1)
    {
        checksum->fwCheckSums.buildChecksum = 12345678;
        checksum->fwCheckSums.runChecksum = 12345678;
        *checksumValid = 1;
        return (int32_t)recoveryAction;
    }
#endif

    if (timeout_us < MIN_TIMEOUT_US)
    {
        /* Set minimum timeout of 10 ms. Prevent divide by zero error */
        timeout_us = MIN_TIMEOUT_US;
    }

    recoveryAction = adi_adrv9001_arm_Memory_Read(device, ADRV9001_ADDR_ARM_CALC_CHKSUM_PTR, checkData, CHECKSUM_BYTES, 0);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Failed to read ARM memory");
    ADI_ERROR_RETURN(device->common.error.newAction);
    checkAddr = ((((uint32_t)checkData[3]) << 24) | (((uint32_t)checkData[2]) << 16) | (((uint32_t)checkData[1]) << 8) | ((uint32_t)checkData[0]));

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest >= 2)
    {
        checkAddr = ADRV9001_ADDR_ARM_CALC_CHKSUM_PTR;
    }
#endif

    waitInterval_us = (waitInterval_us > timeout_us) ? timeout_us : waitInterval_us;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);

    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        recoveryAction = adi_adrv9001_arm_Memory_Read(device, checkAddr, &checkData[0], sizeof(checkData), 0);
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* ARM firmware checksum */
        buildTimeChecksum = ((((uint32_t)checkData[3]) << 24) | (((uint32_t)checkData[2]) << 16) | (((uint32_t)checkData[1]) << 8) | ((uint32_t)checkData[0]));
        calculatedChecksum = ((((uint32_t)checkData[7]) << 24) | (((uint32_t)checkData[6]) << 16) | (((uint32_t)checkData[5]) << 8) | ((uint32_t)checkData[4]));

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest == 3)
        {
            calculatedChecksum = 0;
            buildTimeChecksum = calculatedChecksum;
        }
#endif

        if ((calculatedChecksum == 0) && (eventCheck < numEventChecks))
        {
            /* wait */
            halError = adi_common_hal_Wait_us(&device->common, timeout_us);
#if ADI_ADRV9001_SW_TEST > 0
            if (device->devStateInfo.swTest == 3)
            {
                halError = 1;
            }
#endif
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_ADI_HAL,
                halError,
                ADI_COMMON_ACT_ERR_CHECK_TIMER,
                eventCheck,
                "Timer not working");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else
        {
            break;
        }
    }

#if ADI_ADRV9001_SW_TEST > 0
    if (device->devStateInfo.swTest >= 5)
    {
        calculatedChecksum = 12345678;
        buildTimeChecksum = calculatedChecksum;
    }
    if (device->devStateInfo.swTest == 2)
    {
        calculatedChecksum = 0;
        buildTimeChecksum = calculatedChecksum;
    }
#endif

    /* ARM completed calculating checksum */
    if ((calculatedChecksum > 0) && (buildTimeChecksum > 0))
    {
        *checksumValid = 1;
        checksum->fwCheckSums.buildChecksum = buildTimeChecksum;
        checksum->fwCheckSums.runChecksum = calculatedChecksum;

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest == 5)
        {
            calculatedChecksum = 12345678;
            buildTimeChecksum = calculatedChecksum + 1;
        }
#endif

        /* performing checksum check, skip if checksum was not calculated (ARM DEBUG_MODE)*/
        if ((calculatedChecksum > 0) && (buildTimeChecksum != calculatedChecksum))
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_ADRV9001_SRC_ARM,
                ADI_COMMON_ERR_API_FAIL,
                ADI_ADRV9001_ACT_ERR_RESET_ARM,
                calculatedChecksum,
                "FW checksum is invalid");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

        for (j = 0; j < ADRV9001_MAX_NUM_STREAM; j++)
        {
            i = CHECKSUMENTRYSIZE * (j + 1);

            /* Streams checksum */
            buildTimeChecksum = ((((uint32_t)checkData[i + 3]) << 24) | (((uint32_t)checkData[i + 2]) << 16) | (((uint32_t)checkData[i + 1]) << 8) | ((uint32_t)checkData[i + 0]));
            calculatedChecksum = ((((uint32_t)checkData[i + 7]) << 24) | (((uint32_t)checkData[i + 6]) << 16) | (((uint32_t)checkData[i + 5]) << 8) | ((uint32_t)checkData[i + 4]));

#if ADI_ADRV9001_SW_TEST > 0
            if (device->devStateInfo.swTest == 6)
            {
                calculatedChecksum = 12345678;
                buildTimeChecksum = calculatedChecksum + 1;
            }
#endif
            checksum->streamsCheckSum[j].buildChecksum = buildTimeChecksum;
            checksum->streamsCheckSum[j].runChecksum = calculatedChecksum;

            /* Skip if channel not initialize */
            if ((device->devStateInfo.initializedChannels & streamChannel[j]) > 0)
            {
                /* performing checksum check */
                if (buildTimeChecksum != calculatedChecksum)
                {
                    ADI_ERROR_REPORT(&device->common,
                        ADI_ADRV9001_SRC_ARMCMD,
                        ADI_COMMON_ERR_API_FAIL,
                        ADI_ADRV9001_ACT_ERR_RESET_ARM,
                        calculatedChecksum,
                        "Streams checksum is invalid");
                    ADI_ERROR_RETURN(device->common.error.newAction);
                }
            }

        }

        //i = offsetof(adi_adrv9001_checksumTable_t, deviceProfileCheckSum);
        i = CHECKSUMENTRYSIZE * (ADRV9001_MAX_NUM_STREAM + 1);

        /* Device profile checksum */
        buildTimeChecksum = ((((uint32_t)checkData[i + 3]) << 24) | (((uint32_t)checkData[i + 2]) << 16) | (((uint32_t)checkData[i + 1]) << 8) | ((uint32_t)checkData[i + 0]));
        calculatedChecksum = ((((uint32_t)checkData[i + 7]) << 24) | (((uint32_t)checkData[i + 6]) << 16) | (((uint32_t)checkData[i + 5]) << 8) | ((uint32_t)checkData[i + 4]));

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest == 7)
        {
            calculatedChecksum = 12345678;
            buildTimeChecksum = calculatedChecksum + 1;
        }
#endif
        checksum->deviceProfileCheckSum.buildChecksum = buildTimeChecksum;
        checksum->deviceProfileCheckSum.runChecksum = calculatedChecksum;

#if ADI_ADRV9001_PROFILE_CHECKSUM_ENABLE > 0
        /* performing checksum check */
        if (buildTimeChecksum != calculatedChecksum)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_ARMCMD,
                ADI_COMMON_ERR_API_FAIL,
                ADI_ADRV9001_ACT_ERR_RESET_ARM,
                calculatedChecksum,
                "Arm Profile checksum is invalid");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
#endif

        i = CHECKSUMENTRYSIZE * (ADRV9001_MAX_NUM_STREAM + 2);

        /* ADC profile checksum */
        buildTimeChecksum = ((((uint32_t)checkData[i + 3]) << 24) | (((uint32_t)checkData[i + 2]) << 16) | (((uint32_t)checkData[i + 1]) << 8) | ((uint32_t)checkData[i + 0]));
        calculatedChecksum = ((((uint32_t)checkData[i + 7]) << 24) | (((uint32_t)checkData[i + 6]) << 16) | (((uint32_t)checkData[i + 5]) << 8) | ((uint32_t)checkData[i + 4]));

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest == 8)
        {
            calculatedChecksum = 12345678;
            buildTimeChecksum = calculatedChecksum + 1;
        }
#endif
        checksum->adcProfilefwCheckSum.buildChecksum = buildTimeChecksum;
        checksum->adcProfilefwCheckSum.runChecksum = calculatedChecksum;

#if ADI_ADRV9001_PROFILE_CHECKSUM_ENABLE > 0
        /* performing checksum check */
        if (buildTimeChecksum != calculatedChecksum)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_ARMCMD,
                ADI_COMMON_ERR_API_FAIL,
                ADI_ADRV9001_ACT_ERR_RESET_ARM,
                calculatedChecksum,
                "Adc Profile checksum is invalid");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
#endif
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_MailboxBusy_Get(adi_adrv9001_Device_t *device, uint8_t *mailboxBusy)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, mailboxBusy);

    /* Read arm_command_busy bit in arm_command(0x00c3) register*/
    recoveryAction = adrv9001_NvsRegmapCoreArmCommandBusyBfGet(device, ADRV9001_BF_CORE, mailboxBusy);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Invalid Get for mailboxBusy");
    ADI_ERROR_RETURN(device->common.error.newAction);

    return device->common.error.newAction;
}

int32_t adi_adrv9001_arm_FwStatus_Check(adi_adrv9001_Device_t *device, uint32_t timeout_us,
    uint32_t fwCheckStatus)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;
    uint32_t fwStatus = 0;
    uint32_t waitInterval_us = 0;
    uint32_t numEventChecks = 1;
    uint32_t eventCheck = 0;
    uint8_t status = 0;

    static const uint32_t ADRV9001_ARM_FWSTATUS_ERROR = ADI_ADRV9001_SRC_ARMFWSTATUS << 16;

    ADI_API_ENTRY_EXPECT(device);

    /* Wait for ARM to exit BOOTUP state */
    waitInterval_us = (ADI_ADRV9001_GETARMBOOTUP_INTERVAL_US > timeout_us) ?
        timeout_us : ADI_ADRV9001_GETARMBOOTUP_INTERVAL_US;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);


    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        ADRV9001_SPIREADBYTE(device, "ARM_CMD_STATUS_8", ADRV9001_ADDR_ARM_CMD_STATUS_8, &status);

        fwStatus = (uint32_t)status;

#if ADI_ADRV9001_SW_TEST > 0
        if (device->devStateInfo.swTest)
        {
            ADRV9001_DEBUG_INFO("SwTest Mode: set fwStatus = device->devStateInfo.swTest");
            fwStatus = device->devStateInfo.swTest;
        }
#endif
        if (fwStatus == fwCheckStatus)
        {
            break;
        }
        else
        {
            /*Wait interval then check again */

            halError = adi_common_hal_Wait_us(&device->common, waitInterval_us);
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_ADI_HAL,
                             halError,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             device,
                             "Timer not working");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    if (fwStatus != fwCheckStatus)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_ADRV9001_SRC_ARMFWSTATUS,
                         (ADRV9001_ARM_FWSTATUS_ERROR + fwStatus),
                         ADI_ADRV9001_ACT_ERR_RESET_ARM,
                         device,
                         "ArmBootStatus_e: Timed out waiting for ARM fwCheckStatus to happen");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "ArmFwStatusCheck Failed");
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Profile_Program(adi_adrv9001_Device_t *device)
{
    uint8_t extData[2] = { 0 };

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Executing the SET Device Profile command */
    extData[0] = 0;
    extData[1] = ADRV9001_ARM_OBJECTID_DEVICE_PROFILE;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
                                        (uint8_t)ADRV9001_ARM_SET_OPCODE,
                                        extData[1],
                                        (uint32_t)ADI_ADRV9001_SETARMPROFILE_TIMEOUT_US,
                                        (uint32_t)ADI_ADRV9001_SETARMPROFILE_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_System_Program(adi_adrv9001_Device_t *device,
                                        int32_t(*Mcs_Requested)(void),
                                        uint8_t channelMask)
{
    uint8_t extData[2] = { 0 };
    uint8_t cmdStatusByte = 0;
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    adi_adrv9001_MailboxChannelMask(channelMask, &extData[0]);

    /* Executing the SET System Config command */
    extData[1] = ADRV9001_ARM_OBJECTID_SYSTEM_CONFIG;

    recoveryAction = adi_common_hal_Wait_us(&device->common, 1000000);

    ADI_ERROR_REPORT(&device->common,
                     ADI_ADRV9001_SRC_ARMCMD,
                     recoveryAction,
                     ADI_COMMON_ACT_ERR_CHECK_TIMER,
                     device,
                     "Timer not working in ArmCmdStatusWait()");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    recoveryAction = adi_common_hal_Wait_us(&device->common, 1000000);

    ADI_ERROR_REPORT(&device->common,
                     ADI_ADRV9001_SRC_ARMCMD,
                     recoveryAction,
                     ADI_COMMON_ACT_ERR_CHECK_TIMER,
                     device,
                     "Timer not working in ArmCmdStatusWait()");
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Wait for command to finish executing */
    recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(device,
                                                     Mcs_Requested,
                                                     ADRV9001_ARM_SET_OPCODE,
                                                     &cmdStatusByte,
                                                     ADI_ADRV9001_SETARMSYSTEM_TIMEOUT_US,
                                                     ADI_ADRV9001_SETARMSYSTEM_INTERVAL_US);

    /* ARM error handler to provide valid recovery action based on ARM error code */
    if ((cmdStatusByte >> 1) > 0)
    {
        ADI_EXPECT(adrv9001_ArmCmdErrorHandler,
                   device,
                   ADRV9001_ARMCMD_ERRCODE(ADRV9001_ARM_SET_OPCODE, 0, cmdStatusByte));
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_arm_Clocks_ProgramValidate(adi_adrv9001_Device_t *device, adi_adrv9001_ArmClock_t *armClock)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, armClock);

    /*Check that channel mask is valid*/
    ADI_RANGE_CHECK(device, armClock->channelMask, 0x01, 0x0F);

    /*Check that clock type is valid*/
    ADI_RANGE_CHECK(device, armClock->clockType, 0x0, (ADI_ADRV9001_ARMCLOCK_MAX - 1));

    /*Check that clock enable is valid*/
    ADI_RANGE_CHECK(device, armClock->clockEnable, 0x0, 0x1);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_Clocks_Program(adi_adrv9001_Device_t *device, adi_adrv9001_ArmClock_t *armClock)
{
    uint8_t extData[4] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_arm_Clocks_ProgramValidate, device, armClock);

    /* Executing the SET System Config command */
    extData[0] = armClock->channelMask;
    extData[1] = ADRV9001_ARM_OBJECTID_CLOCK_ENABLE;
    extData[2] = armClock->clockType;
    extData[3] = armClock->clockEnable;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
                                        ADRV9001_ARM_SET_OPCODE,
                                        extData[1],
                                        ADI_ADRV9001_SETARMCLOCK_TIMEOUT_US,
                                        ADI_ADRV9001_SETARMCLOCK_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_arm_WakeupInterrupt_Set(adi_adrv9001_Device_t *device)
{
    uint8_t sw4RegisterRead = 0;
    static const uint8_t SET_SW_INTERRUPT_4 = 0x01;

    ADI_API_ENTRY_EXPECT(device);

    /* Read SW interrupt 4 register */
    ADRV9001_SPIREADBYTE(device, "ADRV9001_ADDR_SW_INTERRUPT_4", ADRV9001_ADDR_SW_INTERRUPT_4, &sw4RegisterRead);

    /* Issue SW interrupt 4 to wake up ARM */
    ADRV9001_SPIWRITEBYTE(device,
                          "ADRV9001_ADDR_SW_INTERRUPT_4",
                          ADRV9001_ADDR_SW_INTERRUPT_4,
                          (sw4RegisterRead | SET_SW_INTERRUPT_4));

    /* Read SW interrupt 4 register */
    ADRV9001_SPIREADBYTE(device, "ADRV9001_ADDR_SW_INTERRUPT_4", ADRV9001_ADDR_SW_INTERRUPT_4, &sw4RegisterRead);

    /* Clear SW interrupt 4 to wake up ARM */
    ADRV9001_SPIWRITEBYTE(device,
                          "ADRV9001_ADDR_SW_INTERRUPT_4",
                          ADRV9001_ADDR_SW_INTERRUPT_4,
                          (sw4RegisterRead & (~SET_SW_INTERRUPT_4)));

    ADI_API_RETURN(device);
}
