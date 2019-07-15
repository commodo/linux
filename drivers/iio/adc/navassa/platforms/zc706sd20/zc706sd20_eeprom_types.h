/**
* \file
* \brief Functions to read I2C EEPROM info based on FMC EEPROM format
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ZC706SD20_EEPROM_TYPES_H__
#define _ZC706SD20_EEPROM_TYPES_H__

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fru.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief FMC FRU board information
 *
 * \note Based on fru_tools/fru.h::BOARD_INFO - modified for use with eRPC
 */
typedef struct board_info {
    unsigned char language_code;
    unsigned int mfg_date;
    char manufacturer[128];
    char product_name[128];
    char serial_number[128];
    char part_number[128];
    char FRU_file_ID[128];
    char pcb_rev[128];                  /*!< ADI custom field 0 */
    char pcb_id[128];                   /*!< ADI custom field 1 */
    char bom_rev[128];                  /*!< ADI custom field 2 */
    char uses_lvds[128];                /*!< ADI custom field 3 */
//    char *tuning;
} board_info_t;

/**
 * \brief FMC FRU EEPROM information
 *
 * \note Based on fru_tools/fru.h::FRU_DATA - modified for use with eRPC
 */
typedef struct adi_fru_data{
//    char *Internal_Area;
//    char *Chassis_Info;
    board_info_t Board_Area;
//    char *Product_Info;
//    multirecord_info_t MultiRecord_Area;
} fru_data_t;

#ifdef __cplusplus
}
#endif
#endif
