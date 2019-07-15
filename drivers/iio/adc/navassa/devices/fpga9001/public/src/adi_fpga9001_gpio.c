
/**
 * \file: adi_fpga9001_gpio.c
 * \brief Functions to configure and control the FPGA9001 GPIO pins
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_gpio.h"
#include "adi_fpga9001.h"
#include "fpga9001_bf_axi_adrv9001.h"

static int32_t adi_fpga9001_GpioModeSetValidate(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e gpioMode)
{
    ADI_RANGE_CHECK(device, gpioMode, ADI_FPGA9001_GPIO_MODE_NORMAL, ADI_FPGA9001_GPIO_MODE_JTAG);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioModeSet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e gpioMode)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioModeSetValidate, device, gpioMode);
    ADI_EXPECT(fpga9001_AxiAdrv9001ModeBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioMode);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_GpioModeGetValidate(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e *gpioMode)
{
    ADI_NULL_PTR_RETURN(&device->common, gpioMode);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioModeGet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e *gpioMode)
{
    uint8_t jtagStatus = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioModeGetValidate, device, gpioMode);

    ADI_EXPECT(fpga9001_AxiAdrv9001ModeBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &jtagStatus);

    *gpioMode = ADI_FPGA9001_GPIO_MODE_NORMAL;

    if (jtagStatus > 0)
    {
        *gpioMode = ADI_FPGA9001_GPIO_MODE_JTAG;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_GpioDirSetValidate(adi_fpga9001_Device_t *device, adi_fpga9001_GpioPinDir_e gpioPinDir, uint16_t pinSelMask)
{
    ADI_RANGE_CHECK(device, gpioPinDir, ADI_FPGA9001_GPIO_PIN_INPUT, ADI_FPGA9001_GPIO_PIN_OUTPUT);
    ADI_RANGE_CHECK(device, pinSelMask, ADI_FPGA9001_GPIO_00, ADI_FPGA9001_GPIO_ALL);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioDirSet(adi_fpga9001_Device_t *device, adi_fpga9001_GpioPinDir_e gpioPinDir, uint16_t pinSelMask)
{
    uint8_t idx = 0;
    uint16_t mask = 1;

    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioDirSetValidate, device, gpioPinDir, pinSelMask);
    
    for (idx = 0; idx < ADI_FPGA9001_NUM_GPIO_PINS; idx++)
    {
        mask = (1 << idx);
        if ((pinSelMask & mask) > 0)
        {
            switch (mask)
            {
            /* FIXME: This is a hack until the GPIO interface is refactored more extensively */
            case ADI_FPGA9001_GPIO_00:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl00BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_01:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl01BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_02:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl02BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_03:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl03BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_04:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl04BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_05:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl05BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_06:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl06BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_07:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl07BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_08:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl08BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_09:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl09BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_10:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl10BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            case ADI_FPGA9001_GPIO_11:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl11BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioPinDir << 4);
                break;
            default:
                /* Should never actually reach here */
                ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, "pinSelMask", "Invalid pinSelMask");
            }
        }
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_GpioDirGetValidate(adi_fpga9001_Device_t *device, uint16_t *pinDirMask)
{
    ADI_NULL_PTR_RETURN(&device->common, pinDirMask);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioDirGet(adi_fpga9001_Device_t *device, uint16_t *pinDirMask)
{
    uint8_t idx = 0;
    uint8_t regVal = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioDirGetValidate, device, pinDirMask);

    *pinDirMask = 0;
    for (idx = 0; idx < ADI_FPGA9001_NUM_GPIO_PINS; idx++)
    {
        switch (idx)
        {
        /* FIXME: This is a hack until the GPIO interface is refactored more extensively */
        case 0:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl00BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 1:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl01BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 2:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl02BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 3:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl03BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 4:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl04BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 5:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl05BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 6:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl06BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 7:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl07BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 8:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl08BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 9:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl09BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 10:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl10BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        case 11:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl11BfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &regVal);
            break;
        default:
            /* Should never actually reach here */
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, "pinSelMask", "Invalid pinSelMask");
        }
        
        *pinDirMask |= ((regVal & 0x10) == 0x10) ? (1 << idx) : 0;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_GpioWriteValidate(adi_fpga9001_Device_t *device, uint16_t wrData)
{
    ADI_RANGE_CHECK(device, wrData, 0, ADI_FPGA9001_GPIO_ALL);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioWrite(adi_fpga9001_Device_t *device, uint16_t wrData)
{
    uint8_t idx = 0;
    uint8_t regVal = 0;

    static const uint8_t GPIO_OUTPUT = 0x10;

    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioWriteValidate, device, wrData);

    for (idx = 0; idx < ADI_FPGA9001_NUM_GPIO_PINS; idx++)
    {
        regVal = (wrData >> idx) & 1;
        if (regVal)
        {
            switch (idx)
            {
                /* FIXME: This is a hack until the GPIO interface is refactored more extensively */
            case 0:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl00BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 1:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl01BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 2:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl02BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 3:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl03BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 4:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl04BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 5:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl05BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 6:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl06BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 7:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl07BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 8:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl08BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 9:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl09BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 10:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl10BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            case 11:
                ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioctl11BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, GPIO_OUTPUT | regVal);
                break;
            default:
                /* Should never actually reach here */
                ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, "pinSelMask", "Invalid pinSelMask");
            }
        }
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_GpioReadValidate(adi_fpga9001_Device_t *device, uint16_t *rdData)
{
    ADI_NULL_PTR_RETURN(&device->common, rdData);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_GpioRead(adi_fpga9001_Device_t *device, uint16_t *rdData)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_GpioReadValidate, device, rdData);

    ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioDataBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, rdData);

    ADI_API_RETURN(device);
}
