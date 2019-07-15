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

#ifndef _ADI_ADRV9001EE01_MCS_H_
#define _ADI_ADRV9001EE01_MCS_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_adrv9001_mcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Execute the specified MCS action
 *
 * \param[in] adrv9001Ee01      Pointer to the ADRV9001 EE01 daughter card instance
 * \param[in] mcsAction         The type of MCS to execute
 * \param[in] bbicInitiated     Whether or not the BBIC initiated MCS; the ADRV9001 system may issue an interrupt
 *                              to request an MCS during initial calibrations - the ISR should set this to false
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Mcs_Execute(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                     adi_adrv9001_McsAction_e mcsAction,
                                     bool bbicInitiated);

/**
 * \brief Interrupt service routine for when ADRV9001 requests an MCS
 *
 * \param[in] adrv9001Ee01 Pointer to the ADRV9001 EE01 daughter card instance
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Mcs_IrqHandler(adi_adrv9001Ee01_Board_t *adrv9001Ee01);

/**
 * \brief Check if an MCS was requested and call adi_adrv9001Ee01_Mcs_IrqHandler if necessary
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Mcs_Requested();
    
/**
 * \brief Global pointer to a board
 * 
 * A pointer to adi_adrv9001Ee01_Mcs_Requested is passed to ADRV9001 device functions from within 
 * adi_adrv9001Ee01_Initialize. adi_adrv9001Ee01_Mcs_Requested requires a board context variable, which is not
 * available in device functions, so it uses this global variable instead
 */
extern adi_adrv9001Ee01_Board_t *boardRef;
    
#ifdef __cplusplus
}
#endif

#endif // !_ADI_ADRV9001EE01_MCS_H_
