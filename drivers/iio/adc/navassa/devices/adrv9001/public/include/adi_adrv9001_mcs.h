/**
* \file
* \brief ADRV9001 Multi-Chip Synchronization (MCS) functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_MCS_H_
#define _ADI_ADRV9001_MCS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common_macros.h"
#include "adi_adrv9001_user.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_mcs_types.h"

/**
 * \brief Sets up the transceiver to listen for incoming MCS pulses to synchronize the internal clock tree.
 *
 *  When working with multiple transceivers MCS is necessary. This function should
 *  be run after all transceivers have finished the adi_adrv9001_InitAnalog().
 *
 *  This function will reset the MCS state machine in the ADRV9001 device. Calling the function again will reset
 *  the state machine and expect the MCS sequence to start over.
 *
 *  Typical sequence:
 *  1) Initialize all ADRV9001 devices in system using adi_adrv9001_InitAnalog()
 *  2) Run adi_adrv9001_Mcs_Prepare().
 *  3) Send at least 4 MCS pulses, and verify mcsStatus is set for each MCS step
 *  4) Run adi_adrv9001_Mcs_AnalogStatus_Get() to verify mcsStatus
 *  5) Continue with init digital sequnece ..
 *
 * \pre Must be called after the device has been initialized and PLL lock status has been verified.
 * \pre Transceiver must NOT be transmitting or receiving
 *
 * \param[in] device        Pointer to the ADRV9001 device data structure containing settings
 * \param[in] pllType       The PLL to prepare for MCS
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */

int32_t adi_adrv9001_Mcs_Prepare(adi_adrv9001_Device_t *device, adi_adrv9001_PllType_e pllType);

/**
 * \brief Request ARM to prepare for MCS
 *
 * \todo Elaborate on difference between this and adi_adrv9001_Mcs_Prepare()
 *
 * \pre Must be in the STANDBY or IDLE state
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_PrepareArm(adi_adrv9001_Device_t *device);

/**
 * \brief Reads back the analog MCS status
 *
 *  After running adi_adrv9001_Mcs_Prepare(), this function can be used
 *  to verify that all the MCS pulses were received by the transceiver.
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 * \param[in]  pllType          The PLL to prepare for MCS
 * \param[out] mcsStatus        The analog mcsStatus word
 * \parblock
 *  mcsStatus | bit Description
 * -----------|--------------------------------------------------------
 *       [0]  | JESD MCS Status
 *       [1]  | Digital Clocks Sync Status
 *       [2]  | Clock Gen Sync Status
 *       [3]  | CLKPLL SDM Sync Status
 *       [4]  | Device Clock divider Sync Status
 *
 * A bit value of 1 indicates the corresponding synchronization occurred
 * A bit value of 0 indicates the corresponding synchronization did not occur
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_AnalogStatus_Get(adi_adrv9001_Device_t *device,
                                          adi_adrv9001_PllType_e pllType,
                                          uint32_t *mcsStatus);

/**
 * \brief Resets the MCS
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_Digital_Reset(adi_adrv9001_Device_t *device);

/**
 * \brief Select either internal or external MCS generation
 *
 * \param[in] device            Pointer to the ADRV9001 device data structure containing settings
 * \param[in] selectInternal    Whether to select internal (true) or external (false) MCS
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_Internal_Select(adi_adrv9001_Device_t *device, bool selectInternal);

/**
 * \brief Enable MCS capture
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_Capture_Enable(adi_adrv9001_Device_t *device);

/**
 * \brief Internally generate one MCS pulse
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_Internal_Sync(adi_adrv9001_Device_t *device);

/**
 * \brief Sets up the transceiver to generate internal pulses to synchronize the internal clock tree.
 *
 *  When working with multiple transceivers Multichip sync is necessary.  This function should
 *  be run after all transceivers have finished the adi_adrv9001_InitDigital().
 *
 *  This function will reset the MCS state
 *  machine in the ADRV9001 device.  Calling the function again, will reset
 *  the state machine and expect the MCS sequence to start over. Since clocks
 *  will be adjusted, the transceiver should not be transmitting or receiving
 *  when this function is called.
 *
 *  Typical sequence:
 *  1) Initialize all ADRV9001 devices in system using adi_adrv9001_InitDigital()
 *  2) Run adi_adrv9001_Mcs_DigitalInt_Set().
 *  3) Continue with init digital sequnece ..
 *
 * \pre This function is called after the device has been initialized and PLL lock status has
 * been verified.  ARM must be running before calling this function.
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 * \param[in]  action           The MCS action to perform
 * \parblock Must be one of the following:
 *     - ADI_ADRV9001_MCS_INT_DIGITAL_ALL
 *     - ADI_ADRV9001_MCS_INT_DIGITAL_CLK
 *     - ADI_ADRV9001_MCS_INT_DIGITAL_SSI
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_DigitalInt_Set(adi_adrv9001_Device_t *device, uint8_t numberOfPulses);

/**
 * \brief Reads back the multi-chip sync status
 *
 *  After running adi_adrv9001_Mcs_DigitalExt_Set(), this function can be used
 *  to verify that all the MCS pulses were received by the transceiver.
 *
 *  Typical sequence:
 *  1) Initialize all ADRV9001 devices in system using adi_adrv9001_initDigital()
 *  2) Run adi_adrv9001_Mcs_DigitalExt_Set()
 *  3) Run adi_adrv9001_Mcs_DigitalExt_Set()
 *  4) Send at least 1 MCS internal pulse, and verify mcsStatus is set for each
 *     MCS step
 *  5) Run adi_adrv9001_Mcs_DigitalStatus_Get() verify mcsStatus
 *  6) Run adi_adrv9001_Mcs_IntfExt_Set().
 *  7) Send at least 1 MCS pulse, and verify mcsStatus is set for each
 *     MCS step
 *  8) Run adi_adrv9001_Mcs_DigitalStatus_Get() to verify mcsStatus
 *  9) Continue with init sequence ... init cals, etc
 *
 * \pre This function is called after the device has been initialized and PLL lock status has
 * been verified.  ARM must be running before using the MCS functions.
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 * \param[out] mcsStatus        The digital mcsStatus word
 * \parblock
 *  mcsStatus | bit Description
 * -----------|----------------------------
 *       [0]  | MCS capture received
 *       [1]  | MCS second capture received
 *
 * A bit value of 1 indicates the corresponding synchronization occurred
 * A bit value of 0 indicates the corresponding synchronization did not occur
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_DigitalStatus_Get(adi_adrv9001_Device_t *device, uint32_t *mcsStatus);

/**
 * \brief Reads the MCS action requested by the ADRV9001
 *
 * \param[in]  device       Pointer to the ADRV9001 device data structure containing settings
 * \param[out] action       The requested MCS action
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Mcs_ActionRequested_Get(adi_adrv9001_Device_t *device, adi_adrv9001_McsAction_e *action);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_MCS_H_ */
