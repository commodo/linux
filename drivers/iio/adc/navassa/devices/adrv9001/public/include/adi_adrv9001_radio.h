/**
 * \file
 * \brief Contains ADRV9001 related function prototypes for adi_adrv9001_radioctrl.c
 *
 *  ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RADIO_H_
#define _ADI_ADRV9001_RADIO_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adrv9001_init_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Sets the Carrier frequency for the given channel.
*
* \pre This function can be called after the ARM has been initialized, and device must be in radioOff state.
*
* \param[in] device                 Pointer to the ADRV9001 device data structure containing settings
* \param[in] port                   The port that the channel refers to
* \param[in] channel                The channel of the specified port to prime
* \param[in] pllCalMode             The PLL calibration mode to use
* \param[in] mbSetState             State to determine whether Mail Box set command is allowed in primed state
* \param[in] carrierFrequency_Hz    Desired carrier frequency for the given channel in Hz (valid: 30MHz to 6 GHz)
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_Radio_CarrierFrequency_Set(adi_adrv9001_Device_t *device,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                adrv9001_PllCalMode_e pllCalMode,
                                                adi_adrv9001_MBSetInPrimedState_e mbSetState,
                                                uint64_t carrierFrequency_Hz);

/**
 * \brief Gets the Carrier frequency for the specified channel.
 *
 * \pre This function can be used after the device has been initialized, ARM binary loaded and the
 *      PLLs configured.
 *
 * \param[in]  device               Pointer to the ADRV9001 device data structure containing settings
 * \param[in]  port                 The port that the channel refers to
 * \param[in]  channel              The channel of the specified port to prime
 * \param[out] carrierFrequency_Hz  Desired carrier frequency for the given channel in Hz
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_Radio_CarrierFrequency_Get(adi_adrv9001_Device_t *device,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                uint64_t *carrierFrequency_Hz);

/**
 * \brief Checks if the PLLs are locked
 *
 * This function returns the status of the ADRV9001 PLLs via the pllLockStatus pointer. The 4 LSBs of the uint32_t value at
 * pllLockStatus represent the lock status of the CLK PLL, RF PLL and AUX PLL.
 *
 * \pre This may be called any time after the PLLs have been configured and are operational. This also requires the
 * lockdet_mode<1:0> to have a value of either 1(Run Lock Detect once) or 2(Run Lock Detect Continuously)
 *
 * \param[in]  device           Pointer to the ADRV9001 device data structure containing settings
 * \param[out] pllLockStatus    The current lock status of the ADRV9001 PLLs
 * \parblock             pllLockStatus |    Status
 *                      ---------------|----------------
 *                           bit 0     | CLK PLL Lock
 *                           bit 1     | CLK LP PLL Lock
 *                           bit 2     | LO1 PLL Lock
 *                           bit 3     | LO2 PLL Lock
 *                           bit 4     | AUX PLL Lock
 *
 * A bit value of 1 indicates the corresponding PLL is locked
 * A bit value of 0 indicates the corresponding PLL is unlocked.
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_PllStatus_Get(adi_adrv9001_Device_t *device, uint32_t *pllLockStatus);

/**
 * \brief Configure how the specified channel is enabled
 *
 * The ADRV9001 defaults to SPI mode on power up. This function is used to reconfigure
 * the signal path control mode of Rx, ORx, and Tx signal chains. SPI mode control can
 * be accomplished through adi_adrv9001_ChannelEnableSet(). Pin mode control is through
 * dedicated input pins to the ADRV9001 device.
 *
 * \pre This function should be called after initialization and loading the stream
 * processor.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to prime
 * \param[in] mode      The desired enable mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_ChannelEnableMode_Set(adi_adrv9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_ChannelEnableMode_e mode);

/**
 * \brief Read the current enable mode for the specified channel
 *
 * \pre This function should be called after initialization and loading the stream
 * processor.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to prime
 * \param[in] mode      The current enable mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_ChannelEnableMode_Get(adi_adrv9001_Device_t *device,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_ChannelEnableMode_e *mode);

/**
 * \brief Reads the current ARM radio state
 *
 * \pre This function can be used after the device has been initialized, ARM binary loaded and the
 *      PLLs configured.
 *
 * \param[in]  device       Pointer to the ADRV9001 device data structure containing settings
 * \param[out] radioState   The current radio state
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_State_Get(adi_adrv9001_Device_t *device, adi_adrv9001_RadioState_t *radioState);

/**
 * \brief Prime the specified channel
 *
 * This function transitions the specified channel from the CALIBRATED state to the PRIMED state.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to prime
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_Prime(adi_adrv9001_Device_t *device,
                                   adi_common_Port_e port,
                                   adi_common_ChannelNumber_e channel);

/**
 * \brief UnPrime the specified channel
 *
 * This function transitions the specified channel from the PRIMED state to the CALIBRATED state.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to un-prime
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_Unprime(adi_adrv9001_Device_t *device,
                                     adi_common_Port_e port,
                                     adi_common_ChannelNumber_e channel);

/**
 * \brief Enable RF operation for the specified channel if in SPI enable mode
 *
 * This function transitions the specified channel from the PRIMED state to the RF ENABLED state.
 *
 * \pre For this function to have any effect, the channel must be in SPI enable mode.
 *      See adi_adrv9001_ChannelEnableModeSet()
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port for which to enable RF operation
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_EnableRf(adi_adrv9001_Device_t *device,
                                      adi_common_Port_e port,
                                      adi_common_ChannelNumber_e channel);

/**
 * \brief Disable RF operation for the specified channel if in SPI enable mode
 *
 * This function transitions the specified channel from the RF ENABLED state to the PRIMED state.
 *
 * \pre For this function to have any effect, the channel must be in SPI enable mode.
 *      See adi_adrv9001_ChannelEnableModeSet()
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port for which to disable RF operation
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_DisableRf(adi_adrv9001_Device_t *device,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel);

/**
 * \brief Transition the specified channel to the STANDBY state
 *
 * This function will transition the specified channel to the STANDBY state from any state where it is valid to do so.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to transition to the STANDBY state
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_ToStandby(adi_adrv9001_Device_t *device,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel);

/**
 * \brief Transition the specified channel to the CALIBRATED state
 *
 * This function will transition the specified channel to the CALIBRATED state from any state where it is valid to do so.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to transition to the CALIBRATED state
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_ToCalibrated(adi_adrv9001_Device_t *device,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel);

/**
 * \brief Transition the specified channel to the PRIMED state
 *
 * This function will transition the specified channel to the PRIMED state from any state where it is valid to do so.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to transition to the PRIMED state
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_ToPrimed(adi_adrv9001_Device_t *device,
                                      adi_common_Port_e port,
                                      adi_common_ChannelNumber_e channel);

/**
 * \brief Transition the specified channel to the RF ENABLED state
 *
 * This function will transition the specified channel to the RF ENABLED state from any state where it is valid to do so.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to transition to the RF ENABLED state
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_ToRfEnabled(adi_adrv9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel);

/**
 * \brief Transition the specified channel to the specified state
 *
 * This function will transition the specified channel to the specified state from any state where it is valid to do so.
 *
 * \param[in] device    Pointer to the ADRV9001 device data structure containing settings
 * \param[in] port      The port that the channel refers to
 * \param[in] channel   The channel of the specified port to transition to the specified state
 * \param[in] state     The state to transition the specified channel to
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_Channel_ToState(adi_adrv9001_Device_t *device,
                                     adi_common_Port_e port,
                                     adi_common_ChannelNumber_e channel,
                                     adi_adrv9001_ChannelState_e state);

/**
 * \brief Configures the loop filter for the specified PLL
 *
 * \param[in] device                Pointer to the ADRV9001 device data structure containing settings
 * \param[in] pllName               The PLL for which to configure the loop filter
 * \param[in] pllLoopFilterConfig   The desired loop filter configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_PllLoopFilter_Set(adi_adrv9001_Device_t *device,
                                       adi_adrv9001_PllName_e pllName,
                                       adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig);

/**
 * \brief Gets the loop filter configuration for the specified PLL
 *
 * \pre This function can be used after the device has been initialized and the
 * PLL configured. The ARM firmware must also be loaded and running.
 *
 * \param[in] device                Pointer to the ADRV9001 device data structure containing settings
 * \param[in] pllName               The PLL for which to configure the loop filter
 * \param[in] pllLoopFilterConfig   The current loop filter configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Radio_PllLoopFilter_Get(adi_adrv9001_Device_t *device,
                                       adi_adrv9001_PllName_e pllName,
                                       adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig);

/**
 * \brief Sets the configuration for the Front End GPIO
 *
 * \param[in] device                Pointer to the ADRV9001 device data structure containing settings
 * \param[in] armGpioSigMap         Pointer to the ARM GPIO Signal Map configuration structure
 * \param[in] armGpioSigMapSize     Size of armGpioSigMap array
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_FrontEndGpioConfig_Set(adi_adrv9001_Device_t *device,
                                            adi_adrv9001_ArmGpioSignalMap_t armGpioSigMap[],
                                            uint32_t armGpioSigMapSize);


/**
 * \deprecated Use adi_adrv9001_Radio_MailboxChannel_Get() and the common channel/port enums instead
 * \brief Converts Channel mask bit position of TX/RX to the channel mask suitable for mailbox command
 *
 * \param  channelMask  An OR'd combination of adi_adrv9001_Channel_e specifying which
                        channels to configure the enable mode for
 * \param  payload      Array pointer where the mailbox command suitable channel mask is written
 *
 * \retval None
 */

void adi_adrv9001_MailboxChannelMask(uint8_t channelMask, uint8_t *payload);

/**
 * \brief Convert a channel and port combination to the appropriate mailbox channel value
 *
 * \param[in] channel   The channel of the specified port
 * \param[in] port      The port that the channel refers to
 *
 * \returns The mailbox channel value corresponding to the channel/port combination
 */
uint8_t adi_adrv9001_Radio_MailboxChannel_Get(adi_common_Port_e port, adi_common_ChannelNumber_e channel);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RADIO_H_ */
