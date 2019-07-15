/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Support for linux layer I2C functions
 */

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/
#ifndef __ZC706SD20_I2C_H__
#define __ZC706SD20_I2C_H__

#include <stdint.h>
#include "adi_platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t zc706sd20_I2cOpen(void *devHalCfg);
int32_t zc706sd20_I2cClose(void *devHalCfg);
int32_t zc706sd20_I2cInit(void *devHalCfg);
int32_t zc706sd20_I2cWrite(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes);
int32_t zc706sd20_I2cRead(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes, uint8_t rxData[], uint32_t numRxBytes);
    
/* Temporary functions for evaluation purposes only
 * In the future, the above functions will be used with a set of adi_hal_x functions*/
int32_t zc706sd20_i2cOpen(uint8_t busNum, uint8_t slaveAddress);
int32_t zc706sd20_i2cWrite(const uint8_t wrData[], uint32_t numWrBytes);
int32_t zc706sd20_i2cRead(const uint8_t wrData[], uint32_t numWrBytes, uint8_t rdData[], uint32_t numRdBytes);
int32_t zc706sd20_i2cClose(void);

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_I2C_H__ */
