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

#include "adi_common.h"

uint8_t adi_channel_to_index(adi_common_ChannelNumber_e channel)
{
    switch (channel)
    {
    case ADI_CHANNEL_1: return 0;
    case ADI_CHANNEL_2: return 1;
    default: return -2;
    }
}