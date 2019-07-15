/**
 * \file
 * \brief Contains functions to allow control of the General Purpose IO functions on the ADRV9001 device
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_GPIO_TYPES_H_
#define _ADI_ADRV9001_GPIO_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
//#include "adi_adrv9001_data_interface_types.h"


/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/
/**
 *  \brief Enum to select desired GPIO pin used by the API
 */
    
typedef enum adi_adrv9001_GpioPinSel
{
    ADI_ADRV9001_GPIO_00_DO_NOT_ASSIGN = 0, /*!< Select GPIO_00*/
    ADI_ADRV9001_GPIO_01, /*!< Select GPIO_01*/
    ADI_ADRV9001_GPIO_02, /*!< Select GPIO_02*/
    ADI_ADRV9001_GPIO_03, /*!< Select GPIO_03*/
    ADI_ADRV9001_GPIO_04, /*!< Select GPIO_04*/
    ADI_ADRV9001_GPIO_05, /*!< Select GPIO_05*/
    ADI_ADRV9001_GPIO_06, /*!< Select GPIO_06*/
    ADI_ADRV9001_GPIO_07, /*!< Select GPIO_07*/
    ADI_ADRV9001_GPIO_08, /*!< Select GPIO_08*/
    ADI_ADRV9001_GPIO_09, /*!< Select GPIO_09*/
    ADI_ADRV9001_GPIO_10, /*!< Select GPIO_11*/
    ADI_ADRV9001_GPIO_11, /*!< Select GPIO_12*/
    ADI_ADRV9001_GPIO_12, /*!< Select GPIO_13*/
    ADI_ADRV9001_GPIO_13, /*!< Select GPIO_14*/
    ADI_ADRV9001_GPIO_14, /*!< Select GPIO_15*/
    ADI_ADRV9001_GPIO_15, /*!< Select GPIO_16*/
    ADI_ADRV9001_GPIO_INVALID    /*!< Invalid GPIO*/
} adi_adrv9001_GpioPinSel_e;

/**
 *  \brief Enum to set the low voltage GPIO mode
 */
typedef enum adi_adrv9001_GpioOutputSel
{
    ADI_ADRV9001_GPIO_ARM_1_0 = 0,      /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_3_2 = 1,      /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_5_4 = 2,      /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_7_6 = 3,      /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_9_8 = 4,      /*!< Allows internal ARM processor to output on GPIO pins */    
    ADI_ADRV9001_GPIO_ARM_11_10 = 5,    /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_13_12 = 6,    /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_ARM_15_14 = 7,    /*!< Allows internal ARM processor to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_1_0 = 8,      /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_3_2 = 9,      /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_5_4 = 10,     /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_7_6 = 11,     /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_9_8 = 12,     /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_11_10 = 13,   /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_13_12 = 14,   /*!< Allows SPI to output on GPIO pins */
    ADI_ADRV9001_GPIO_SPI_15_14 = 15,   /*!< Allows SPI to output on GPIO pins */    
    ADI_ADRV9001_GPIO_RX1_1_0 = 16,     /*!< Allows Rx1 Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_RX1_3_2 = 17,     /*!< Allows Rx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_RX1_5_4 = 18,     /*!< Allows Rx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_RX1_7_6 = 19,     /*!< Allows Rx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_RX2_1_0 = 20,     /*!< Allows Rx2 Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_RX2_3_2 = 21,     /*!< Allows Rx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_RX2_5_4 = 22,     /*!< Allows Rx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_RX2_7_6 = 23,     /*!< Allows Rx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX1_1_0 = 24,     /*!< Allows Tx1 Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_TX1_3_2 = 25,     /*!< Allows Tx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX1_5_4 = 26,     /*!< Allows Tx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX1_7_6 = 27,     /*!< Allows Tx1 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX2_1_0 = 28,     /*!< Allows Tx2 Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_TX2_3_2 = 29,     /*!< Allows Tx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX2_5_4 = 30,     /*!< Allows Tx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_TX2_7_6 = 31,     /*!< Allows Tx2 Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_MAIN_1_0 = 32,    /*!< Allows Main Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_MAIN_3_2 = 33,    /*!< Allows Main Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_MAIN_5_4 = 34,    /*!< Allows Main Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_MAIN_7_6 = 35,    /*!< Allows Main Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_ANALOG_1_0 = 36,  /*!< Allows Analog Control to output on GPIO pins */  
    ADI_ADRV9001_GPIO_ANALOG_3_2 = 37,  /*!< Allows Analog Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_ANALOG_5_4 = 38,  /*!< Allows Analog Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_ANALOG_7_6 = 39,  /*!< Allows Analog Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_ANALOG_9_8 = 40,  /*!< Allows Analog Control to output on GPIO pins */
    ADI_ADRV9001_GPIO_ANALOG_11_10 = 41 /*!< Allows Analog Control to output on GPIO pins */
} adi_adrv9001_GpioOutputSel_e;
 
/**
*  \brief Enum to select desired Analog GPIO pins used by the API
*/
typedef enum adi_adrv9001_GpioAnaPinSel
{
    ADI_ADRV9001_GPIO_ANA_00 = 0, /*!< Select GPIO_ANA_00*/
    ADI_ADRV9001_GPIO_ANA_01, /*!< Select GPIO_ANA_01*/
    ADI_ADRV9001_GPIO_ANA_02, /*!< Select GPIO_ANA_02*/
    ADI_ADRV9001_GPIO_ANA_03, /*!< Select GPIO_ANA_03*/
    ADI_ADRV9001_GPIO_ANA_04, /*!< Select GPIO_ANA_04*/
    ADI_ADRV9001_GPIO_ANA_05, /*!< Select GPIO_ANA_05*/
    ADI_ADRV9001_GPIO_ANA_06, /*!< Select GPIO_ANA_06*/
    ADI_ADRV9001_GPIO_ANA_07, /*!< Select GPIO_ANA_07*/    
    ADI_ADRV9001_GPIO_ANA_08, /*!< Select GPIO_ANA_08*/
    ADI_ADRV9001_GPIO_ANA_09, /*!< Select GPIO_ANA_09*/
    ADI_ADRV9001_GPIO_ANA_10, /*!< Select GPIO_ANA_10*/
    ADI_ADRV9001_GPIO_ANA_11, /*!< Select GPIO_ANA_11*/
    ADI_ADRV9001_GPIO_ANA_INVALID  /*!< Invalid Analog Gpio*/
} adi_adrv9001_GpioAnaPinSel_e;
    
/**
* \brief Enum for selecting the GP_INT channel
*/
typedef enum adi_adrv9001_gpMaskSelect
{
    ADI_ADRV9001_GPINT,
    ADI_ADRV9001_GPINT_NUMBER_OF_CHANNELS /* Keep this ENUM last as a reference to the total number of gp channel enum values */
}adi_adrv9001_gpMaskSelect_e;	

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/

/**
* \brief GP_INT status general structure
*/
typedef struct adi_adrv9001_gpIntStatus
{
    uint32_t gpIntStatus;
    uint32_t gpIntMask;
    uint32_t gpIntActiveSources;
    uint32_t gpIntSaveIrqMask;
} adi_adrv9001_gpIntStatus_t;

/**
* \brief Data structure holding the GP interrupt mask values
*/
typedef struct adi_adrv9001_gpMaskArray
{
    uint32_t gpIntMask;
}adi_adrv9001_gpMaskArray_t;

/**
 *  \brief Data struct to hold the GPIO output source control
 */
typedef struct adi_adrv9001_GpioOutSourceCtrl
{
    adi_adrv9001_GpioOutputSel_e gpioPin1Pin0;
    adi_adrv9001_GpioOutputSel_e gpioPin3Pin2;
    adi_adrv9001_GpioOutputSel_e gpioPin5Pin4;
    adi_adrv9001_GpioOutputSel_e gpioPin7Pin6;
    adi_adrv9001_GpioOutputSel_e gpioPin9Pin8;
    adi_adrv9001_GpioOutputSel_e gpioPin11Pin10;
    adi_adrv9001_GpioOutputSel_e gpioPin13Pin12;
    adi_adrv9001_GpioOutputSel_e gpioPin15Pin14;
} adi_adrv9001_GpioOutSourceCtrl_t; 

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_GPIO_TYPES_H_ */
