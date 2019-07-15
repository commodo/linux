/**
* \file
* \brief Contains ADI common interface.
*
* ADI common lib Version: $ADI_COMMON_LIB_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_COMMON_H_
#define _ADI_COMMON_H_

#include "adi_common_error.h"
#include "adi_common_hal.h"
#include "adi_common_log.h"
#include "adi_common_types.h"
#include "adi_common_libc.h"

uint8_t adi_channel_to_index(adi_common_ChannelNumber_e channel);

#endif  /* _ADI_COMMON_H_ */
