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

#ifdef __GNUC__
#include <unistd.h>
#endif /* __GNUC__ */

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "zc706sd20_i2c.h"

/**
* \brief Function to open/allocate any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cOpen(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
	adi_hal_I2cCfg_t *i2cCfg = NULL;

    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__
    if (i2cCfg->fd <= 0)
    {
        i2cCfg->fd = open(i2cCfg->drvName, O_RDWR);

        if (i2cCfg->fd < 0)
        {
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
    }
#endif /* __GNUC__ */
    return halError;
}

/**
* \brief Function to close/deallocate any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cClose(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__
    if (i2cCfg->fd != 0)
    {
        halError = close(i2cCfg->fd);
        if (halError < 0)
        {
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
        
        i2cCfg->fd = 0;
    }
#endif /* __GNUC__ */

    return halError;
}

/**
* \brief Function to init any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cInit(void *devHalCfg)
{
    return (int32_t)ADI_HAL_OK;
}

/**
* \brief Function to write to an I2C device from the BBIC
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Byte array of data to write to the I2C device.  First byte 
*               should be the I2C register address followed by one or more data bytes.
* \param numTxBytes Number of bytes in the txData array
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cWrite(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    int retVal = 0;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    if ((numTxBytes > 0) &&
        (txData != NULL))
    {
        if (i2cCfg->fd > 0)
        {
#ifdef __GNUC__
            retVal = write(i2cCfg->fd, &txData[0], numTxBytes);
            if (retVal != (int)numTxBytes)
            {
                perror("I2C : Failed to Write to device");
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
#endif
        }
        else
        {
            perror("I2C : Failed to Write to device, file descriptor invalid");
            return (int32_t)ADI_HAL_GEN_SW;
        }
    }
    
    return halError;
}

/**
* \brief Function to read from an I2C device to the BBIC
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Byte array of data to write to the I2C device. Depending on the
*               I2C device, this might just be 1 byte containing the register 
*               address to read
* \param numTxBytes Number of bytes in the txData array
* \param rxData Byte array to return the read back data
* \param numRxBytes Number of bytes to read back, and size of rxData array
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cRead(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes, uint8_t rxData[], uint32_t numRxBytes)
{
	int32_t halError = (int32_t)ADI_HAL_OK;
    int retVal = 0;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    if (i2cCfg->fd > 0)
    {
        /* Write Register address */
        if ((numTxBytes > 0) && 
            (txData != NULL))
        {
#ifdef __GNUC__
            retVal = write(i2cCfg->fd, &txData[0], numTxBytes);
            if (retVal != (int)numTxBytes)
            {
                perror("I2C : Failed to Write to I2C device");
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
#endif
        }

        /* Read data */
        if ((numRxBytes > 0) &&
            (rxData != NULL))
        {
#ifdef __GNUC__
            retVal = read(i2cCfg->fd, &rxData[0], numRxBytes);
            if (retVal != (int)numRxBytes)
            {
                perror("I2C : Failed to Read from I2C device");
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
#endif
        }
    }
    else
    {
        perror("I2C : Failed to Write to device, file descriptor invalid");
        return (int32_t)ADI_HAL_GEN_SW;
    }
        
    
    return halError;
}

/* FIXME: Remove
 * Temporary functions for evaluation purposes only
 * In the future, the above functions will be used with a set of adi_hal_x functions*/
static int i2cFd = -1;
static uint8_t slaveAddressHolder = 0;
static uint8_t busNumHolder = 0;
int32_t zc706sd20_i2cOpen(uint8_t busNum, uint8_t slaveAddress)
{
    char busNumChar = 0;
    char devName[50] = { 0 };
    
    if (!(busNum >= 0 && busNum <= 7))
    {
        return (int32_t)ADI_HAL_I2C_FAIL;
    }
	slaveAddressHolder = slaveAddress;
	busNumHolder = busNum;
    
    busNumChar = busNum + '0';
    sprintf(devName, "/dev/i2c-%c", busNumChar);
    
    /* Open the device driver */
    if (i2cFd < 0)
    {
        i2cFd = open(devName, O_RDWR);

        if (i2cFd < 0)
        {
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
    }
    
    /* Set the slave address of the peripheral to talk to */
	if (ioctl(i2cFd, I2C_SLAVE, slaveAddress) < 0)
    {
        return (int32_t)ADI_HAL_I2C_FAIL;
    }
    
    return (int32_t)ADI_HAL_OK;
}

int32_t zc706sd20_i2cWrite(const uint8_t wrData[], uint32_t numWrBytes)
{
    int retVal = 0;
    
    if ((numWrBytes > 0) &&
        (wrData != NULL))
    {
        if (i2cFd > 0)
        {
            retVal = write(i2cFd, &wrData[0], numWrBytes);
            if (retVal != (int)numWrBytes)
            {
                perror("I2C : Failed to Write to device");
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
            return (int32_t)ADI_HAL_OK;
        }
        else
        {
            printf("I2C : Failed to Write to device, file descriptor invalid\n");
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
    }
    
    return (int32_t)ADI_HAL_I2C_FAIL;
}

int32_t zc706sd20_i2cRead(const uint8_t wrData[], uint32_t numWrBytes, uint8_t rdData[], uint32_t numRdBytes)
{
	uint8_t outbuf[1];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset[1];

	outbuf[0] = wrData[0];
		
	msgs[0].addr = slaveAddressHolder;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = outbuf;

	msgs[1].addr = slaveAddressHolder;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = numRdBytes;
	msgs[1].buf = rdData;

	msgset[0].msgs = msgs;
	msgset[0].nmsgs = 2;
	
	
    if (i2cFd > 0)
    {
        /* Write Register address */
	    if ((numWrBytes > 0) && 
	        (wrData != NULL))
	    {
		    if ((numRdBytes > 0) &&
		    	(rdData != NULL))
		    {
			    
			    ioctl(i2cFd, I2C_RDWR, &msgset);
			    rdData = msgset->msgs->buf;
		    }
	    }
        else
        {
            printf("Invalid parameter rdData. Can't be NULL");
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
    }
    else
    {
        printf("I2C : Failed to Write to device, file descriptor invalid\n");
        return (int32_t)ADI_HAL_GEN_SW;
    }
        
    
    return (int32_t)ADI_HAL_OK;
}

int32_t zc706sd20_i2cClose(void)
{
    if (i2cFd > 0)
    {
        close(i2cFd);
        i2cFd = -1;
    }
    
    return (int32_t)ADI_HAL_OK;
}