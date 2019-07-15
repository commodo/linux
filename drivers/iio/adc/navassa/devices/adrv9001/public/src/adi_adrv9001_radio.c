/**
* \file
* \brief Contains features related function implementation defined in
* adi_adrv9001_radioctrl.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_tx.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_nvs_pll_mem_map.h"
#include "adrv9001_init.h"
#include "adrv9001_radioctrl.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_bf_nvs_regmap_core_2.h"

#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_radioctrl.c"
#endif

/* TODO: Refactor to common validators */
static int32_t adi_adrv9001_Channel_Validate(adi_adrv9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Port_Validate(adi_adrv9001_Device_t *device,
                                          adi_common_Port_e port)
{
    switch (port)
    {
    case ADI_RX:    /* Falls through */
    case ADI_TX:
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         port,
                         "Invalid parameter value. port must be either ADI_RX or ADI_TX.");
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Radio_CarrierFrequency_Set_Validate(adi_adrv9001_Device_t *device,
                                                          adi_common_Port_e port,
                                                          adi_common_ChannelNumber_e channel,
                                                          adrv9001_PllCalMode_e pllCalMode,
                                                          adi_adrv9001_MBSetInPrimedState_e mbSetState,
                                                          uint64_t carrierFrequency_Hz)
{
    static const uint64_t CARRIER_FREQUENCY_MIN_HZ =   30000000;    /* 30 MHz */
    static const uint64_t CARRIER_FREQUENCY_MAX_HZ = 6000000000;    /* 6 GHz */
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);

    ADI_RANGE_CHECK(device, pllCalMode, ADI_ADRV9001_PLL_CAL_MODE_NORM, ADI_ADRV9001_PLL_RESERVED);

    /*Check that Mail Box command SET state selected is valid*/
    if ((mbSetState != ADI_ADRV9001_MB_NOT_ALLOWED) &&
        (mbSetState != ADI_ADRV9001_MB_RESERVED))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            mbSetState,
            "Invalid Mail Box SET command state selected");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    
    /* Unfortunately, can't use ADI_RANGE_CHECK due to format specifiers causing -Wformat error */
    if (carrierFrequency_Hz < CARRIER_FREQUENCY_MIN_HZ || carrierFrequency_Hz > CARRIER_FREQUENCY_MAX_HZ)
    {
        snprintf(device->common.error.errormessage,
                 sizeof(device->common.error.errormessage),
                 "Invalid parameter value. %s was %llu, but must be between %llu and %llu, inclusive.",
                 "carrierFrequency_Hz",
                 carrierFrequency_Hz,
                 CARRIER_FREQUENCY_MIN_HZ,
                 CARRIER_FREQUENCY_MAX_HZ);
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         carrierFrequency_Hz,
                         device->common.error.errormessage);
    }
    
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_CarrierFrequency_Set(adi_adrv9001_Device_t *device,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                adrv9001_PllCalMode_e pllCalMode,
                                                adi_adrv9001_MBSetInPrimedState_e mbSetState,
                                                uint64_t carrierFrequency_Hz)
{
    uint8_t armData[8] = { 0 };
    uint8_t extData[4] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Radio_CarrierFrequency_Set_Validate, device, port, channel, pllCalMode, mbSetState, carrierFrequency_Hz);

    /* Loading byte array with parsed bytes from carrierFrequency_Hz word */
    armData[0] = (uint8_t)((carrierFrequency_Hz >> 0) & 0xFF);
    armData[1] = (uint8_t)((carrierFrequency_Hz >> 8) & 0xFF);
    armData[2] = (uint8_t)((carrierFrequency_Hz >> 16) & 0xFF);
    armData[3] = (uint8_t)((carrierFrequency_Hz >> 24) & 0xFF);
    armData[4] = (uint8_t)((carrierFrequency_Hz >> 32) & 0xFF);
    armData[5] = (uint8_t)((carrierFrequency_Hz >> 40) & 0xFF);
    armData[6] = (uint8_t)((carrierFrequency_Hz >> 48) & 0xFF);
    armData[7] = (uint8_t)((carrierFrequency_Hz >> 56) & 0xFF);

    /* Write carrier Frequency to ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, device, (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);

    /* Executing the SET carrier Freq command */
    extData[1] = ADRV9001_ARM_OBJECTID_CHANNEL_CARRIER_FREQUENCY;
    extData[2] = (uint8_t)pllCalMode;
    extData[3] = (uint8_t)mbSetState;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_SETCARRIER_FREQUENCY_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_SETCARRIER_FREQUENCY_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_CarrierFrequency_Get(adi_adrv9001_Device_t *device,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                uint64_t *carrierFrequency_Hz)
{
    const uint8_t ARM_MEM_READ_AUTOINCR = 0;

    uint8_t armData[8] = { 0 };
    uint8_t extData[4] = { 0 };

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, carrierFrequency_Hz);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);
    
    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);

    /* Executing the GET PLL Freq command */
    extData[1] = ADRV9001_ARM_OBJECTID_CHANNEL_CARRIER_FREQUENCY;
    extData[2] = 0;
    extData[3] = 0;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
                   device,
                   (uint8_t)ADRV9001_ARM_GET_OPCODE,
                   &extData[0],
                   sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_GET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_GETCARRIER_FREQUENCY_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_GETCARRIER_FREQUENCY_INTERVAL_US);

    /* Read PLL Frequency from ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
                   device,
                   (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_GET,
                   &armData[0],
                   sizeof(armData),
                   ARM_MEM_READ_AUTOINCR);

    /*Form pllFrequency word with data read back from ARM mailbox*/
    *carrierFrequency_Hz = ((uint64_t)(armData[0]) << 0) |
                           ((uint64_t)(armData[1]) << 8) |
                           ((uint64_t)(armData[2]) << 16) |
                           ((uint64_t)(armData[3]) << 24) |
                           ((uint64_t)(armData[4]) << 32) |
                           ((uint64_t)(armData[5]) << 40) |
                           ((uint64_t)(armData[6]) << 48) |
                           ((uint64_t)(armData[7]) << 56);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_PllStatus_Get(adi_adrv9001_Device_t *device, uint32_t *pllLockStatus)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t pllLockStatusRead = 0;

    static const uint8_t CLK_PLL_LOCK_STATUS_SHIFT    = 0;
    static const uint8_t CLK_PLL_LP_LOCK_STATUS_SHIFT = 1;
    static const uint8_t LO1_PLL_LOCK_STATUS_SHIFT    = 2;
    static const uint8_t LO2_PLL_LOCK_STATUS_SHIFT    = 3;
    static const uint8_t AUX_PLL_LOCK_STATUS_SHIFT    = 4;
    static const uint32_t PLL_STATUS_MASK = 0x0000001F;

    /* Check device pointer and pllLockStatus are not null */
    ADI_API_ENTRY_PTR_EXPECT(device, pllLockStatus);

    /* Clear status of all PLLs */
    *pllLockStatus &= ~PLL_STATUS_MASK;

    /* Read CLK Pll status */
    ADI_EXPECT(adrv9001_NvsPllMemMapSynLockBfGet, device, ADRV9001_BF_CLK_PLL, &pllLockStatusRead);

    /* Update pllLockStatus bit 0 with Clk Pll Status */
    *pllLockStatus |= ((uint32_t)pllLockStatusRead & 0x00000001) << CLK_PLL_LOCK_STATUS_SHIFT;

    /* Read CLK LP (Low Power) Pll status */
    ADI_EXPECT(adrv9001_NvsPllMemMapSynLockBfGet, device, ADRV9001_BF_CLK_PLL_LP, &pllLockStatusRead);

    /* Update pllLockStatus bit 1 with Clk LP Pll Status */
    *pllLockStatus |= ((uint32_t)pllLockStatusRead & 0x00000001) << CLK_PLL_LP_LOCK_STATUS_SHIFT;

    /* Read LO1 Pll status */
    ADI_EXPECT(adrv9001_NvsPllMemMapSynLockBfGet, device, ADRV9001_BF_RF1_PLL, &pllLockStatusRead);

    /* Update pllLockStatus bit 2 with LO1 Pll Status */
    *pllLockStatus |= ((uint32_t)pllLockStatusRead & 0x00000001) << LO1_PLL_LOCK_STATUS_SHIFT;

    /* Read LO2 Pll status */
    ADI_EXPECT(adrv9001_NvsPllMemMapSynLockBfGet, device, ADRV9001_BF_RF2_PLL, &pllLockStatusRead);

    /* Update pllLockStatus bit 3 with LO2 Pll Status */
    *pllLockStatus |= ((uint32_t)pllLockStatusRead & 0x00000001) << LO2_PLL_LOCK_STATUS_SHIFT;

    /* Read Aux Pll status */
    ADI_EXPECT(adrv9001_NvsPllMemMapSynLockBfGet, device, ADRV9001_BF_AUX_PLL, &pllLockStatusRead);

    /* Update pllLockStatus bit 4 with Aux Pll Status */
    *pllLockStatus |= ((uint32_t)pllLockStatusRead & 0x00000001) << AUX_PLL_LOCK_STATUS_SHIFT;

    return recoveryAction;
}

static int32_t adi_adrv9001_Radio_ChannelEnableMode_Set_Validate(adi_adrv9001_Device_t *device,
                                                           adi_common_ChannelNumber_e channel,
                                                           adi_common_Port_e port,
                                                           adi_adrv9001_ChannelEnableMode_e mode)
{
    ADI_EXPECT(adi_adrv9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_adrv9001_Port_Validate, device, port);
    ADI_RANGE_CHECK(device, mode, ADI_ADRV9001_SPI_MODE, ADI_ADRV9001_PIN_MODE);
    
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_ChannelEnableMode_Set(adi_adrv9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_ChannelEnableMode_e mode)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Radio_ChannelEnableMode_Set_Validate, device, channel, port, mode);

    /* Disable ARM override ctrl and set pin mode bitfield to 0 for SPI mode or 1 for pin mode */
    if (port == ADI_RX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx1ArmOverrideCtrlBfSet, device, ADRV9001_BF_CORE_2, false);
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx1PinModeBfSet, device, ADRV9001_BF_CORE_2, mode);
    }
    else if (port == ADI_RX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx2ArmOverrideCtrlBfSet, device, ADRV9001_BF_CORE_2, false);
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx2PinModeBfSet, device, ADRV9001_BF_CORE_2, mode);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx1ArmOverrideCtrlBfSet, device, ADRV9001_BF_CORE_2, false);
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx1PinModeBfSet, device, ADRV9001_BF_CORE_2, mode);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx2ArmOverrideCtrlBfSet, device, ADRV9001_BF_CORE_2, false);
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx2PinModeBfSet, device, ADRV9001_BF_CORE_2, mode);
    }
    else
    {
        /* Nothing to do. Validation should ensure this is never reached */
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Radio_ChannelEnableMode_Get_Validate(adi_adrv9001_Device_t *device,
                                                           adi_common_Port_e port,
                                                           adi_common_ChannelNumber_e channel,
                                                           adi_adrv9001_ChannelEnableMode_e *mode)
{
    ADI_EXPECT(adi_adrv9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_adrv9001_Port_Validate, device, port);
    ADI_NULL_PTR_RETURN(&device->common, mode);
    
    ADI_API_RETURN(device);
}
int32_t adi_adrv9001_Radio_ChannelEnableMode_Get(adi_adrv9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_ChannelEnableMode_e *mode)
{
    uint8_t regVal = 0;
    ADI_PERFORM_VALIDATION(adi_adrv9001_Radio_ChannelEnableMode_Get_Validate, device, port, channel, mode);

    if (port == ADI_RX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx1PinModeBfGet, device, ADRV9001_BF_CORE_2, &regVal);
    }
    else if (port == ADI_RX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx2PinModeBfGet, device, ADRV9001_BF_CORE_2, &regVal);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx1PinModeBfGet, device, ADRV9001_BF_CORE_2, &regVal);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx2PinModeBfGet, device, ADRV9001_BF_CORE_2, &regVal);
    }
    else
    {
        /* Nothing to do. Validation should ensure this is never reached */
    }
    
    *mode = (adi_adrv9001_ChannelEnableMode_e)regVal;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_State_Get(adi_adrv9001_Device_t *device, adi_adrv9001_RadioState_t *radioState)
{
    uint8_t regValue = 0;

    /* Range checks */
    ADI_API_ENTRY_PTR_EXPECT(device, radioState);

    ADRV9001_SPIREADBYTE(device, "arm_cmd_status_8", ADRV9001_ADDR_ARM_CMD_STATUS_8, &regValue);

    radioState->systemState         = regValue & 0x03;
    radioState->monitorModeState    = (regValue >> 2) & 0x03;
    radioState->bootState           = (regValue >> 4) & 0x0F;

    ADRV9001_SPIREADBYTE(device, "arm_cmd_status_9", ADRV9001_ADDR_ARM_CMD_STATUS_9, &regValue);

    radioState->channelStates[ADI_RX][ADI_CHANNEL_1] = regValue & 0x03;
    radioState->channelStates[ADI_RX][ADI_CHANNEL_2] = (regValue >> 2) & 0x03;
    radioState->channelStates[ADI_TX][ADI_CHANNEL_1] = (regValue >> 4) & 0x03;
    radioState->channelStates[ADI_TX][ADI_CHANNEL_2] = (regValue >> 6) & 0x03;
    

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Channel_State_GenericValidate(adi_adrv9001_Device_t *device,
                                                          adi_common_Port_e port,
                                                          adi_common_ChannelNumber_e channel)
{
    /*Check that ARM and Stream processors have been loaded before enabling*/
    if (device->devStateInfo.devState < ADI_ADRV9001_STATE_ARM_LOADED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device->devStateInfo.devState,
                         "Channel Enable/Disable is valid only after ARM and stream processors have been loaded");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Check for valid channel and port */
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Channel_Standby(adi_adrv9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel)
{
    uint8_t mailboxChannel = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_State_GenericValidate, device, port, channel);

    mailboxChannel = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, ADRV9001_ARM_STANDBY_OPCODE, &mailboxChannel, 1);

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_STANDBY_OPCODE,
        0,
        (uint32_t)ADI_ADRV9001_STANDBY_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_STANDBY_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Radio_Channel_Prime_Set(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              adi_common_Port_e port,
                                              uint8_t prime)
{
    uint8_t opCode = 0;
    uint8_t mailboxChannel = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_State_GenericValidate, device, port, channel);
    
    mailboxChannel = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);
    
    if (prime != 0)
    {
        opCode = ADRV9001_ARM_RADIOON_OPCODE;
    }
    else
    {
        opCode = ADRV9001_ARM_RADIOOFF_OPCODE;
    }

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, opCode, &mailboxChannel, 1);

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        opCode,
        0,
        (uint32_t)ADI_ADRV9001_RADIOONOFF_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_RADIOONOFF_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_Channel_Prime(adi_adrv9001_Device_t *device, 
                                   adi_common_Port_e port,
                                   adi_common_ChannelNumber_e channel)
{
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);
    
    // Validate current state
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_CALIBRATED &&
        currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelState,
                         "Error while attempting to prime channel. Channel must be in the CALIBRATED state to be primed.");
        ADI_API_RETURN(device)
    }
    
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED)
    {
        ADI_EXPECT(adi_adrv9001_Radio_Channel_Prime_Set, device, channel, port, 1);
    }
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_Unprime(adi_adrv9001_Device_t *device, 
                                     adi_common_Port_e port,
                                     adi_common_ChannelNumber_e channel)
{
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);
    
    // Validate current state
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED &&
        currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_CALIBRATED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelState,
                         "Error while attempting to un-prime channel. Channel must be in the PRIMED state to be un-primed.");
        ADI_API_RETURN(device)
    }
    
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_CALIBRATED)
    {
        ADI_EXPECT(adi_adrv9001_Radio_Channel_Prime_Set, device, channel, port, 0);
    }
    
    ADI_API_RETURN(device)
}

static int32_t adi_adrv9001_Channel_EnableRF_Set(adi_adrv9001_Device_t *device,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint8_t enable)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_State_GenericValidate, device, port, channel);

    enable = enable > 0 ? 1 : 0;

    /* Set the enable field for the specified channel */
    if (port == ADI_RX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx1EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    else if (port == ADI_RX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicRx2EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx1EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicTx2EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    /* TODO: Is ORX necessary? */
    else if (port == ADI_ORX && channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicOrx1EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    else if (port == ADI_ORX && channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2BbicOrx2EnableBfSet, device, ADRV9001_BF_CORE_2, enable);
    }
    else
    {
        /* If ADI_ADRV9001_RADIOCTRL_RANGE_CHECK is 1, this line should never be executed, therefore untestable */
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_Channel_EnableRf(adi_adrv9001_Device_t *device, 
                                      adi_common_Port_e port,
                                      adi_common_ChannelNumber_e channel)
{
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);

    // Validate current state
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED &&
        currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_RF_ENABLED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelState,
                         "Error while attempting to enable RF for channel. Channel must be in the PRIMED state to enable RF.");
        ADI_API_RETURN(device)
    }
    
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_RF_ENABLED)
    {
        ADI_EXPECT(adi_adrv9001_Channel_EnableRF_Set, device, port, channel, 1);
    }
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_DisableRf(adi_adrv9001_Device_t *device, 
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel)
{
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Port_Validate, device, port);
    ADI_PERFORM_VALIDATION(adi_adrv9001_Channel_Validate, device, channel);

    // Validate current state
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_RF_ENABLED &&
        currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelState,
                         "Error while attempting to disable RF for channel. Channel must be in the RF_ENABLED state to disable RF.");
        ADI_API_RETURN(device)
    }
    
    if (currentState.channelStates[port][channel] != ADI_ADRV9001_CHANNEL_PRIMED)
    {
        ADI_EXPECT(adi_adrv9001_Channel_EnableRF_Set, device, port, channel, 0);
    }
    
    ADI_API_RETURN(device)
}

static int32_t adi_adrv9001_Channel_DisableRF_Wait(adi_adrv9001_Device_t *device, 
                                                   adi_common_Port_e port,
                                                   adi_common_ChannelNumber_e channel, 
                                                   uint8_t numTries)
{
    uint8_t i = 0;
    adi_adrv9001_RadioState_t currentState = { 0 };
    
    ADI_EXPECT(adi_adrv9001_Radio_Channel_DisableRf, device, port, channel);
    for (i = 0; i < numTries; i++)
    {
        ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
        if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_PRIMED)
        {
            return ADI_COMMON_ACT_NO_ACTION;
        }
    }
    
    return ADI_COMMON_ACT_WARN_RERUN_FEATURE;
}

int32_t adi_adrv9001_Radio_Channel_ToStandby(adi_adrv9001_Device_t *device, 
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel)
{
    static const uint8_t NUM_TRIES = 5;
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_RF_ENABLED)
    {
        ADI_EXPECT(adi_adrv9001_Radio_Channel_DisableRf, device, port, channel);
        ADI_EXPECT(adi_adrv9001_Channel_DisableRF_Wait, device, port, channel, NUM_TRIES);
    }
    ADI_EXPECT(adi_adrv9001_Channel_Standby, device, port, channel);
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_ToCalibrated(adi_adrv9001_Device_t *device, 
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel)
{
    static const uint8_t NUM_TRIES = 5;
    adi_adrv9001_RadioState_t currentState = { 0 };
    
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    
    if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_STANDBY)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         currentState,
                         "Error moving channel to CALIBRATED state - channel is in STANDBY. Use the adi_adrv9001_InitCalsRun() function instead");
        ADI_API_RETURN(device)
    }
    else if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_CALIBRATED)
    {
        /* Nothing to do, already in CALIBRATED state */
    }
    else
    {
        if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_RF_ENABLED)
        {
            ADI_EXPECT(adi_adrv9001_Radio_Channel_DisableRf, device, port, channel);
            ADI_EXPECT(adi_adrv9001_Channel_DisableRF_Wait, device, port, channel, NUM_TRIES);
        }
        ADI_EXPECT(adi_adrv9001_Radio_Channel_Unprime, device, port, channel);
    }
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_ToPrimed(adi_adrv9001_Device_t *device,
                                      adi_common_Port_e port,
                                      adi_common_ChannelNumber_e channel)
{
    static const uint8_t NUM_TRIES = 5;
    adi_adrv9001_RadioState_t currentState = { 0 };
    
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    
    if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_STANDBY)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         currentState,
                         "Error moving channel to PRIMED state - channel is in STANDBY. Use the adi_adrv9001_InitCalsRun() function to move to CALIBRATED state first");
        ADI_API_RETURN(device)
    }
    else if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_CALIBRATED)
    {
        ADI_EXPECT(adi_adrv9001_Radio_Channel_Prime, device, port, channel);
    }
    else if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_PRIMED)
    {
        /* Nothing to do, already in PRIMED state */
    }
    else
    {
        ADI_EXPECT(adi_adrv9001_Radio_Channel_DisableRf, device, port, channel);
        ADI_EXPECT(adi_adrv9001_Channel_DisableRF_Wait, device, port, channel, NUM_TRIES);
    }
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_ToRfEnabled(adi_adrv9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel)
{
    adi_adrv9001_RadioState_t currentState = { 0 };
    
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);
    
    if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_STANDBY)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         currentState,
                         "Error moving channel to RF_ENABLED state - channel is in STANDBY. Use the adi_adrv9001_InitCalsRun() function to move to CALIBRATED state first");
        ADI_API_RETURN(device)
    }
    else if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_RF_ENABLED)
    {
        /* Nothing to do, already in RF_ENABLED state */
    }
    else 
    {
        if (currentState.channelStates[port][channel] == ADI_ADRV9001_CHANNEL_CALIBRATED)
        {
            ADI_EXPECT(adi_adrv9001_Radio_Channel_Prime, device, port, channel);
        }
        ADI_EXPECT(adi_adrv9001_Radio_Channel_EnableRf, device, port, channel);
    }
    
    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Radio_Channel_ToState(adi_adrv9001_Device_t *device, 
                                     adi_common_Port_e port, 
                                     adi_common_ChannelNumber_e channel, 
                                     adi_adrv9001_ChannelState_e state)
{
    switch (state)
    {
    case ADI_ADRV9001_CHANNEL_STANDBY:
        ADI_EXPECT(adi_adrv9001_Radio_Channel_ToStandby, device, port, channel);
        break;
    case ADI_ADRV9001_CHANNEL_CALIBRATED:
        ADI_EXPECT(adi_adrv9001_Radio_Channel_ToCalibrated, device, port, channel);
        break;
    case ADI_ADRV9001_CHANNEL_PRIMED:
        ADI_EXPECT(adi_adrv9001_Radio_Channel_ToPrimed, device, port, channel);
        break;
    case ADI_ADRV9001_CHANNEL_RF_ENABLED:
        ADI_EXPECT(adi_adrv9001_Radio_Channel_ToRfEnabled, device, port, channel);
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         state,
                         "Error, invalid argument. 'state' must be a valid adi_adrv9001_ChannelState_e");
    }
    
    ADI_API_RETURN(device)
}

/* TODO: Remove */
void adi_adrv9001_MailboxChannelMask(uint8_t channelMask, uint8_t *payload)
{
    uint8_t mailboxChannelMask = 0;
    static const uint8_t CHANNEL_ALL_MASK = 0x3F;

    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_RX1))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_RX1;
    }

    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_RX2))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_RX2;
    }

    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_TX1))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_TX1;
    }
        
    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_TX2))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_TX2;
    }

    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_ORX1))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_ORX1;
    }

    if (ADRV9001_BF_EQUAL(channelMask, ADI_ADRV9001_CHANNEL_ORX2))
    {
        mailboxChannelMask |= ADRV9001_MAILBOX_CHANNEL_ORX2;
    }

    payload[0] = mailboxChannelMask & CHANNEL_ALL_MASK;
}

static int32_t adi_adrv9001_Radio_PllLoopFilter_Set_Validate(adi_adrv9001_Device_t *device,
                                                       adi_adrv9001_PllName_e pllName,
                                                       adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig)
{
    static const uint8_t  MINIMUM_PLL_LOOP_FILTER_PHASE_MARGIN_DEGREES = 40;
    static const uint8_t  MAXIMUM_PLL_LOOP_FILTER_PHASE_MARGIN_DEGREES = 85;
    static const uint16_t MINIMUM_LOOP_FILTER_BANDWIDTH_KHZ = 50;
    static const uint16_t MAXIMUM_LOOP_FILTER_BANDWIDTH_KHZ = 1500;
    static const uint8_t  MINIMUM_POWER_SCALE_FACTOR = 0;
    static const uint8_t  MAXIMUM_POWER_SCALE_FACTOR = 10;

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, pllLoopFilterConfig);

    /*Check that PLL selected is valid*/
    ADI_RANGE_CHECK(device, pllName, ADI_ADRV9001_LO1_PLL, ADI_ADRV9001_AUX_PLL);

    /*Check that Loop Filter phase margin is between 40-85 degrees*/
    ADI_RANGE_CHECK(device,
                        pllLoopFilterConfig->phaseMargin_degrees,
                        MINIMUM_PLL_LOOP_FILTER_PHASE_MARGIN_DEGREES,
                        MAXIMUM_PLL_LOOP_FILTER_PHASE_MARGIN_DEGREES);

    /*Check that loop filter bandwidth is between 50Khz - 1500Khz*/
    ADI_RANGE_CHECK(device,
                        pllLoopFilterConfig->loopBandwidth_kHz,
                        MINIMUM_LOOP_FILTER_BANDWIDTH_KHZ,
                        MAXIMUM_LOOP_FILTER_BANDWIDTH_KHZ);

    /*Check that power scale factor is between 0-10*/
    ADI_RANGE_CHECK(device,
                        pllLoopFilterConfig->powerScale,
                        MINIMUM_POWER_SCALE_FACTOR,
                        MAXIMUM_POWER_SCALE_FACTOR);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_PllLoopFilter_Set(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig)
{
    uint8_t armData[4] = { 0 };
    uint8_t extData[3] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Radio_PllLoopFilter_Set_Validate, device, pllNameSel, pllLoopFilterConfig);

    /* Loading byte array with parsed bytes from pllLoopFilterConfig struct */
    armData[0] = pllLoopFilterConfig->phaseMargin_degrees;
    armData[1] = (uint8_t)(pllLoopFilterConfig->loopBandwidth_kHz & 0x00FF);
    armData[2] = (uint8_t)((pllLoopFilterConfig->loopBandwidth_kHz >> 8) & 0x00FF);
    armData[3] = pllLoopFilterConfig->powerScale;

    /* Write PLL Frequency to ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write,
                   device,
                   (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_SET,
                   &armData[0],
                   sizeof(armData));

    /* Executing the SET PLL Freq command */
    extData[0] = 0;
    extData[1] = ADRV9001_ARM_OBJECTID_PLL_LOOPFILTER;
    extData[2] = (uint8_t)pllNameSel;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
                   device,
                   (uint8_t)ADRV9001_ARM_SET_OPCODE,
                   &extData[0],
                   sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_SETLOOPFILTER_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_SETLOOPFILTER_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Radio_PllLoopFilter_Get(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig)
{
    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;

    uint8_t armData[6] = { 0 };
    uint8_t extData[3] = { 0 };

    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_EXPECT(device, pllLoopFilterConfig);

    /* Executing the GET PLL Freq command */
    extData[0] = 0;
    extData[1] = ADRV9001_ARM_OBJECTID_PLL_LOOPFILTER;
    extData[2] = (uint8_t)pllNameSel;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_GET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_GETLOOPFILTER_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_GETLOOPFILTER_INTERVAL_US);

    /* Read PLL Loop Filter from ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
                   device,
                   (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_GET,
                   &armData[0],
                   sizeof(armData),
                   ARM_MEM_READ_AUTOINCR);

    /*Deserialize ARM Data into pllLoopFilterConfig Structure*/
    pllLoopFilterConfig->phaseMargin_degrees = armData[0];
    pllLoopFilterConfig->loopBandwidth_kHz = (((uint16_t)armData[1]) |
                                              ((uint16_t)armData[2] << 8));
    pllLoopFilterConfig->powerScale = armData[3];
    pllLoopFilterConfig->effectiveLoopBandwidth_kHz = (((uint16_t)armData[4]) |
                                                       ((uint16_t)armData[5] << 8));

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_FrontEndGpioConfig_Set(adi_adrv9001_Device_t *device, adi_adrv9001_ArmGpioSignalMap_t armGpioSigMap[], uint32_t armGpioSigMapSize)
{
    uint8_t i = 0;
    
    /* Check device pointer is not null */
    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, armGpioSigMap, armGpioSigMapSize);
    
    for (i = 0; i < armGpioSigMapSize; i++)
    {
        ADI_EXPECT(adrv9001_ArmGpioPinSet, device, &armGpioSigMap[i]);
    } 
    
    ADI_API_RETURN(device);
}

uint8_t adi_adrv9001_Radio_MailboxChannel_Get(adi_common_Port_e port, adi_common_ChannelNumber_e channel)
{
    if (port == ADI_RX && channel == ADI_CHANNEL_1)
    {
        return ADRV9001_MAILBOX_CHANNEL_RX1;
    }
    else if (port == ADI_RX && channel == ADI_CHANNEL_2)
    {
        return ADRV9001_MAILBOX_CHANNEL_RX2;
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_1)
    {
        return ADRV9001_MAILBOX_CHANNEL_TX1;
    }
    else if (port == ADI_TX && channel == ADI_CHANNEL_2)
    {
        return ADRV9001_MAILBOX_CHANNEL_TX2;
    }
    else if (port == ADI_ORX && channel == ADI_CHANNEL_1)
    {
        return ADRV9001_MAILBOX_CHANNEL_ORX1;
    }
    else if (port == ADI_ORX && channel == ADI_CHANNEL_2)
    {
        return ADRV9001_MAILBOX_CHANNEL_ORX2;
    }
    else
    {
        return 0;
    }
}