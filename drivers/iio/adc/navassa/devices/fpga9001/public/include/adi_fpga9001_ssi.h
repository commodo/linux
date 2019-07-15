/**
 * \file
 * \brief Functions for FPGA9001 CMOS Synchronous Serial Interface (SSI) configuration
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_SSI_H_
#define _ADI_FPGA9001_SSI_H_

#include <stdint.h>
#include "adi_adrv9001_types.h"
#include "adi_fpga9001_types.h"
#include "adi_fpga9001_ssi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Set the I Delay Tap Value for the specified CMOS SSI Lane
 *
 * \param[in] device    Pointer to the FPGA9001 device data structure
 * \param[in] ssiSel    Specifies which interface to configure
 * \param[in] laneSel   Specifies which lane within the specifed SSI to configure
 * \param[in] delay     The delay tap value to be set
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG    Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION         Function completed successfully, no action required
 */
int32_t adi_fpga9001_ssi_IDelayTapValue_Set(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                            uint8_t delay);

/**
 * \brief Get the I Delay Tap Value for the specified CMOS SSI Lane
 *
 * \param[in]  device   Pointer to the FPGA9001 device data structure
 * \param[in]  ssiSel   Specifies which interface to read from
 * \param[in]  laneSel  Specifies which lane to read from
 * \param[out] delay    The current delay tap value
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG    Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION         Function completed successfully, no action required
 */
int32_t adi_fpga9001_ssi_IDelayTapValue_Get(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            adi_fpga9001_CmosSsiLaneSel_e laneSel,
                                            uint8_t *delay);
    
    
/**
 * \brief Set the CMOS/LVDS SSI mode for the specified CMOS/LVDS SSI Lane
 *
 * \param[in] device    Pointer to the FPGA9001 device data structure
 * \param[in] ssiSel    Specifies which interface to read from
 * \param[in] mode      The desired mode
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG    Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_FULL    Recovery action for full reset
 * \retval ADI_COMMON_ACT_NO_ACTION         Function completed successfully, no action required
 */
int32_t adi_fpga9001_ssi_Mode_Set(adi_fpga9001_Device_t *device,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  adi_fpga9001_SsiMode_e mode);

/**
 * \brief Get the CMOS/LVDS SSI mode for the specified CMOS/LVDS SSI Lane
 *
 * \param[in]  device    Pointer to the FPGA9001 device data structure
 * \param[in]  ssiSel    Specifies which interface to read from
 * \param[out] mode      The current mode
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG    Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_FULL    Recovery action for full reset
 * \retval ADI_COMMON_ACT_NO_ACTION         Function completed successfully, no action required
 */
int32_t adi_fpga9001_ssi_Mode_Get(adi_fpga9001_Device_t *device,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  adi_fpga9001_SsiMode_e *mode);

/**
 * \brief Get the number of bytes per sample for the specified CMOS SSI Lane
 *
 * \param[in]  device    Pointer to the FPGA9001 device data structure
 * \param[in]  ssiSel    Specifies which interface to read from
 * \param[out] mode      The number of bytes per sample
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG    Recovery action for log reset
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_ERR_RESET_FULL    Recovery action for full reset
 * \retval ADI_COMMON_ACT_NO_ACTION         Function completed successfully, no action required
 */
int32_t adi_fpga9001_ssi_BytesPerSample_Get(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t *bytesPerSample);
    
#ifdef __cplusplus
}
#endif

#endif // !_ADI_FPGA9001_SSI_H_