/**
* \file
* \brief Contains Calibration features related function implementation defined in
* adi_adrv9001_cals.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_arm.h"
#include "adrv9001_init.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_reg_addr_macros.h"

#ifdef _RELEASE_BUILD_
#line __LINE__ "adi_adrv9001_cals.c"
#endif

int32_t adi_adrv9001_InitCalsRun(adi_adrv9001_Device_t *device,
                                 int32_t (*Mcs_Requested)(),
                                 adi_adrv9001_InitCals_t *initCals,
                                 uint32_t timeout_ms,
                                 uint8_t *errorFlag)
{
    uint8_t payloadMailbox[12] = { 0 };
    uint8_t payload[2] = { 0 };
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t cmdStatusByte = 0;
    uint8_t errFlag = 0;

    static const uint16_t TIMEOUT_MS_FACTOR = 1000;
    static const uint32_t ADRV9001_RX1_TX1  = 0; /*!< Rx1/Tx1 channels */
    static const uint32_t ADRV9001_RX2_TX2  = 1; /*!< Rx2/Tx2 channels */

    ADI_API_ENTRY_PTR_EXPECT(device, initCals);
    ADI_NULL_PTR_RETURN(&device->common, errorFlag);

    /* Bit mask info for non-channel related Init calibrations */
    payloadMailbox[0] = (uint8_t)(initCals->sysInitCalMask);
    payloadMailbox[1] = (uint8_t)(initCals->sysInitCalMask >> 8);
    payloadMailbox[2] = (uint8_t)(initCals->sysInitCalMask >> 16);
    payloadMailbox[3] = (uint8_t)(initCals->sysInitCalMask >> 24);

    /* Bit mask info for Rx1 and Tx1 channels */
    payloadMailbox[4] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX1_TX1]);
    payloadMailbox[5] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX1_TX1] >> 8);
    payloadMailbox[6] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX1_TX1] >> 16);
    payloadMailbox[7] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX1_TX1] >> 24);

    /* Bit mask info for Rx2 and Tx2 channels */
    payloadMailbox[8]  = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX2_TX2]);
    payloadMailbox[9]  = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX2_TX2] >> 8);
    payloadMailbox[10] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX2_TX2] >> 16);
    payloadMailbox[11] = (uint8_t)(initCals->chanInitCalMask[ADRV9001_RX2_TX2] >> 24);

    ADI_MSG_EXPECT("Failed to write ARM mem",
                   adi_adrv9001_arm_Memory_Write,
                   device,
                   ADRV9001_ADDR_ARM_MAILBOX_RUN_INIT,
                   &payloadMailbox[0],
                   ADI_ARRAY_LEN(payloadMailbox));

    /* channelMask is ignored for RUN_INIT */
    payload[0] = 0;

    /* Mode to select the Init calibration algorithms to run */
    payload[1] = (uint8_t)(initCals->calMode);

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, ADRV9001_ARM_RUNINIT_OPCODE, &payload[0], ADI_ARRAY_LEN(payload));

    recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(device,
                                                     Mcs_Requested,
                                                     ADRV9001_ARM_RUNINIT_OPCODE,
                                                     &cmdStatusByte,
                                                     (timeout_ms * TIMEOUT_MS_FACTOR),
                                                     ADI_ADRV9001_INITCALSWAIT_INTERVAL_US);

    errFlag = (cmdStatusByte >> 1);

    /* Don't update errorFlag if SPI error because errorFlag could be a random */
    /* value but update error flag for other recovery action types */
    if (recoveryAction == ADI_COMMON_ACT_ERR_RESET_INTERFACE)
    {
        *errorFlag = 0;
    }
    else
    {
        *errorFlag = errFlag;
    }

    /* ARM error handler to provide valid recovery action based on ARM error code */
    if (errFlag > 0)
    {
        ADI_EXPECT(adrv9001_ArmCmdErrorHandler,
                   device,
                   ADRV9001_ARMCMD_ERRCODE(ADRV9001_ARM_RUNINIT_OPCODE, 0, cmdStatusByte));
    }
    
    device->devStateInfo.devState = (adi_adrv9001_ApiStates_e)(device->devStateInfo.devState | ADI_ADRV9001_STATE_INITCALS_RUN);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_InitCalsBuildDefault(adi_adrv9001_InitCals_t *initCals)
{
    initCals->calMode = ADI_ADRV9001_INITCAL_RUN_ALL_ALGO_MODE;
    initCals->sysInitCalMask = ADI_ADRV9001_SYSTEM_ALL;

    int ii = 0;
    for (ii = 0; ii < ADI_ADRV9001_MAX_RX_ONLY; ii++)
    {
        initCals->chanInitCalMask[ii] = ADI_ADRV9001_RX_TX_ALL;
    }

    return ADI_COMMON_ACT_NO_ACTION;
}