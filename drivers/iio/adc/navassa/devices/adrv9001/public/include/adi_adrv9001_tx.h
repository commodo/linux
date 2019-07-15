/**
* \file
* \brief Contains ADRV9001 transmit related function prototypes for
*        adi_adrv9001_tx.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the FPGA9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TX_H_
#define _ADI_ADRV9001_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* System header files */

/* ADI specific header files */
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_types.h"

/**
 * \brief Configure the Tx attenuation for the specified channel
 *
 * \pre This function is called after the device is initialized
 *
 * \param[in] device	Pointer to the ADRV9001 data structure
 * \param[in] channel   The Tx channel for which to configure the attenuation
 * \param[in] config    The Tx attenuation configuration to apply
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_Attenuation_Configure(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_TxAttenuationConfig_t *config);

/**
 * \brief Inspect the Tx attenuation configuration for the specified channel
 *
 * \pre This function is called after the device is initialized
 *
 * \param[in] device	Pointer to the ADRV9001 data structure
 * \param[in] channel   The Tx channel for which to inspect the attenuation configuration
 * \param[in] config    The current Tx attenuation configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_Attenuation_Inspect(adi_adrv9001_Device_t *device,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenuationConfig_t *config);

/**
 * \brief Set the attenuation control mode
 *
 * \param[in] device	Pointer to the ADRV9001 data structure
 * \param[in] channel   The Tx channel for which to set the attenuation control mode
 * \param[in] mode      The desired Tx attenuation mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_AttenuationMode_Set(adi_adrv9001_Device_t *device,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenMode_e mode);

/**
 * \brief Get the current attenuation control mode
 *
 * \param[in] device	Pointer to the ADRV9001 data structure
 * \param[in] channel   The Tx channel for which to get the attenuation control mode
 * \param[in] mode      The current Tx attenuation mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_AttenuationMode_Get(adi_adrv9001_Device_t *device,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenMode_e *mode);

/**
 * \brief Set the Tx attenuation for the specified channel
 *
 * \pre This function may be called any time after device initialization
 * \pre This feature requires
 *      1. Initialization to be complete
 *      2. Tx Attenuation table to be loaded.
 *
 * \param[in] device	        Pointer to the ADRV9001 data structure
 * \param[in] channel           The Tx channel for which to set the attenuation
 * \param[in] attenuation_mdB   
 * \parblock
 * The desired attenuation in milli-dB.
 * 
 * Constraints:
 * | adi_adrv9001_TxAttenuationConfig_t.outputSignaling | Maximum Attenuation (mdB) | Resolution (mdB)  |
 * | :------------------------------------------------: | :-----------------------: | :--------------:  |
 * |                ADI_ADRV9001_FM_DM                  |           12000           |       500         |
 * |                      Other                         |           41950           |        50         |
 * 
 * Value must be a multiple of the Resolution (above) between 0 and the Maximum Attenuation (above).
 * 
 * The ADRV9001 attenuator block consists of an analog portion (coarse adjustments) and a digital portion (fine 
 * adjustments). Normally, both analog and digital attenuation are used. In FM_DM mode only the analog attenuation is
 * used. As a result, the resolution is decreased. The maximum attenuation is decreased because the resolution can't be
 * satisfied below that threshold.
 * \endparblock
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_Attenuation_Set(adi_adrv9001_Device_t *device,
                                        adi_common_ChannelNumber_e channel,
                                        uint16_t attenuation_mdB);

/**
 * \brief Get the Tx attenuation for the specified channel
 *
 * \pre This feature requires the initialization to be complete and the attenuation table to be loaded.
 * \pre The Tx data path must be powered up for the current attenuation value to be valid. If the Tx data path
 *  is powered down or the radio is off, the last Tx attenuation setting when the Tx output was previously active will be
 *  read back.
 *
 * \param[in]  device	        Pointer to the ADRV9001 data structure
 * \param[in]  channel          The Tx channel for which to get the attenuation
 * \param[out] attenuation_mdB  The current attenuation in milli-dB (Range: 0 to 41950 mdB)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_Attenuation_Get(adi_adrv9001_Device_t *device,
                                        adi_common_ChannelNumber_e channel,
                                        uint16_t *attenuation_mdB);

/**
 * \brief Enable or disable the Tx DAC full scale boost
 *
 * Enable or disable the Tx DAC full scale boost. Enabling the boost increases the DAC output power by 3dB.
 * Boost is disabled by default.
 *
 * \todo Fix this pre-condition message
 * \pre This function must be called before loading the ADRV9001 ARM processor
 *
 * \param[in] device	        Pointer to the ADRV9001 data structure
 * \param[in] channel           The Tx channel for which to set the DAC full scale boost enabledness
 * \param[in] boostEnable       Whether or not to enable the Tx DAC full scale boost
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_DacFullScaleBoost_Set(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              bool boostEnable);

/**
 * \brief Get the current Tx DAC full scale boost enable status
 *
 * \todo Fix this pre-condition message
 * \pre This function must be called before loading the ADRV9001 ARM processor
 *
 * \param[in]  device	        Pointer to the ADRV9001 data structure
 * \param[in]  channel          The Tx channel for which to get the DAC full scale boost enabledness
 * \param[out] boostEnabled     Whether or not the Tx DAC full scale boost is currently enabled
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_DacFullScaleBoost_Get(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              bool *boostEnabled);

/**
 * \brief Write the attenuation table for the specified Tx channels
 *
 * The full attenuation table can be loaded in a single call or the table can be loaded in several calls,
 * loading a subset of the table with each function call using the indexOffset parameter.
 *
 * \pre This function may be called any time after device initialization
 *
 * \todo Automatically set unused entries to max attenuation
 * \note All unused attenuation table entries should be written to max attenuation so that if an
 * index that doesn't have an entry is selected, the output is known and deterministic.
 *
 * \param[in] device	            Pointer to the ADRV9001 data structure
 * \param[in] channelMask           An OR'd combination of adi_common_ChannelNumber_e
 *                                  specifying the Tx Channels for which to write the atten table
 * \param[in] indexOffset           The attenuation table index at which to start writing (0 - 839).
 * \param[in] attenTableRows        Attenuation table entries to write
 * \param[in] arraySize             The number of attenuation table rows to write (1 - 840).
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_AttenuationTable_Write(adi_adrv9001_Device_t *device,
                                               uint32_t channelMask,
                                               uint32_t indexOffset,
                                               adi_adrv9001_TxAttenTableRow_t attenTableRows[],
                                               uint32_t arraySize);

/**
 * \brief Read the atten table entries for the specified Tx channel
 *
 * The full TxAtten table can be read in a single call or the table can be read in several calls,
 * reading a subset of the table with each function call using the txAttenIndexOffset parameter.
 *
 * \pre This function may be called any time after device initialization
 *
 * \param[in]  device	            Pointer to the ADRV9001 data structure
 * \param[in]  channel              The Tx channel from which to read the attenuation table
 * \param[in]  indexOffset          The attenuation table index at which to start reading (0 - 839).
 * \param[out] attenTableRows       Attenuation table entries to write
 * \param[in]  arraySize            The size of the attenTableRows array; the max number of atten table rows to read
 * \param[out] numAttenIndicesRead  The actual no. of atten indices read. Pass NULL if this info is not needed
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_AttenuationTable_Read(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              uint32_t indexOffset,
                                              adi_adrv9001_TxAttenTableRow_t attenTableRows[],
                                              uint32_t arraySize,
                                              uint16_t *numAttenIndicesRead);

/**
 * \brief Configure PA Protection for the specified Tx channel
 *
 * This function sets up the PA Protection functionality and enables Tx sample
 * power measurements. It does not enable the ability to change Tx Attenuation
 * automatically.
 *
 * FIXME: Vivek - Need to update more info when fully implemented
 *
 * \pre Complete normal ADRV9001 initialization and init cals before running this function.
 *
 * \param[in] device	    Pointer to the ADRV9001 data structure
 * \param[in] channel       The Tx channel for which to configure PA protection
 * \param[in] config        The desired PA protection configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_PaProtection_Configure(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_TxPaProtectCfg_t *config);

/**
 * \brief Inspect the PA protection configuration for the specified Tx channel
 *
 * FIXME: Vivek - Need to update more info when fully implemented
 *
 * \pre Complete normal ADRV9001 initialization and init cals before running this function.
 *
 * \param[in] device	    Pointer to the ADRV9001 data structure
 * \param[in] channel       The Tx channel for which to inspect the PA protection configuration
 * \param[in] config        The current PA protection configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_PaProtection_Inspect(adi_adrv9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_TxPaProtectCfg_t *config);

/**
 * \brief Set the Tx NCO test tone frequency for the specified Tx channel
 *
 *  The TxAttenuation is forced in this function to max analog output power, but the digital attenuation is backed
 *  off 6dB to make sure the digital filter does not clip and cause spurs in the tx spectrum. Ensure no other API's are
 *  called that change the Tx attenuation mode when using this function
 *
 *  When the Tx NCO test tone is disabled, the function sets the Tx attenuation to the SPI mode. User may need to use
 *  other functions to change the mode.
 *
 * \param[in] device	        Pointer to the ADRV9001 data structure
 * \param[in] channel           The Tx channel for which to set the NCO test tone frequency
 * \param[in] ncoFrequency_Hz   The desired NCO test tone frequency (Hz)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_NcoFrequency_Set(adi_adrv9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         int32_t ncoFrequency_Hz);

/**
 * \brief Get the Tx NCO test tone frequency for the specified Tx channel
 *
 * \param[in]  device	        Pointer to the ADRV9001 data structure
 * \param[in]  channel          The Tx channel for which to get the NCO test tone frequency
 * \param[out] ncoFrequency_Hz  The current NCO test tone frequency (Hz)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_NcoFrequency_Get(adi_adrv9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         int32_t *ncoFrequency_Hz);


/**
 * \brief Configure the slew rate limiter for the specified Tx channel
 *
 * \param[in] device	        Pointer to the ADRV9001 data structure
 * \param[in] channel           The Tx channel for which to configure the slew rate limiter
 * \param[in] config            The desired slew rate limiter configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_SlewRateLimiter_Configure(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SlewRateLimiterCfg_t *config);

/**
 * \brief Inspect the slew rate limiter configuration for the specified Tx channel
 *
 * \param[in]  device	        Pointer to the ADRV9001 data structure
 * \param[in]  channel          The Tx channel for which to configure the slew rate limiter
 * \param[out] config           The desired slew rate limiter configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_SlewRateLimiter_Inspect(adi_adrv9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_SlewRateLimiterCfg_t *config);

/**
 * \brief This function configures the PA ramp for the specified Tx channel
 *
 * \param[in] device      Pointer to the ADRV9001 device data structure
 * \param[in] channel     The Tx channel for which to configure the PA ramp
 * \param[in] paRampCfg   The desired PA ramp configuration to be written
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_PaRamp_Configure(adi_adrv9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_PaRampCfg_t *paRampCfg);

/**
 * \brief This function reads back the PA ramp configuration for the specified Tx channel
 *
 * \param[in] device       Pointer to the ADRV9001 device data structure
 * \param[in] channel      The Tx channel for which to configure the PA ramp
 * \param[out] paRampCfg   The actual PA ramp configuration read back
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Tx_PaRamp_Inspect(adi_adrv9001_Device_t *device,
                                       adi_common_ChannelNumber_e channel,
                                       adi_adrv9001_PaRampCfg_t *paRampCfg);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_TX_H_ */
