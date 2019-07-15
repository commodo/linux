/**
 * \file
 * \brief Contains ADRV9001 shared resource related private data prototypes for
 *        adrv9001_shared_resource_manager.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_SHARED_RESOURCE_MANAGER_TYPES_H_
#define _ADRV9001_SHARED_RESOURCE_MANAGER_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Macros defining the maximum semaphore count for a given feature */
#define ADRV9001_FEATURE_RX_GAIN_CTRL_PIN_MAX_SEMAPHORE_COUNT 2
#define ADRV9001_FEATURE_RX_PIN_CTRL_ATTN_MAX_SEMAPHORE_COUNT 2
#define ADRV9001_FEATURE_RX_EXT_CTRL_WORD_OUTPUT_MAX_SEMAPHORE_COUNT 1
#define ADRV9001_FEATURE_TX_ATTEN_CTRL_PIN_MAX_SEMAPHORE_COUNT 2
#define ADRV9001_FEATURE_AUX_DAC_OUT_MAX_SEMAPHORE_COUNT 1
#define ADRV9001_FEATURE_TX_TO_ORX_MAPPING_MAX_SEMAPHORE_COUNT 1
#define ADRV9001_FEATURE_ARM_GPIO_PIN_MAX_SEMAPHORE_COUNT 1
/***Please add shared resource maximum semaphore count above this line****/
    
typedef enum adrv9001_SharedResourceType
{
    ADRV9001_SHARED_RESOURCE_GPIO = 0,        /*!< Select GPIO as the Shared Resource type*/
    ADRV9001_SHARED_RESOURCE_GPIO_ANALOG = 1, /*!< Select GPIO Analog as the Shared Resource type*/
    /***Please add shared resource type above this line****/
    ADRV9001_NUM_SHARED_RESOURCE_TYPES        
} adrv9001_SharedResourceType_e;

typedef enum adrv9001_SharedResourceID
{
    /******START OF GPIO********/
    ADRV9001_GPIO_00_DO_NOT_ASSIGN = 0,       /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_00, do not assign*/
    ADRV9001_GPIO_01,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_01*/
    ADRV9001_GPIO_02,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_02*/
    ADRV9001_GPIO_03,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_03*/
    ADRV9001_GPIO_04,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_04*/
    ADRV9001_GPIO_05,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_05*/
    ADRV9001_GPIO_06,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_06*/
    ADRV9001_GPIO_07,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_07*/
    ADRV9001_GPIO_08,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_08*/
    ADRV9001_GPIO_09,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_09*/
    ADRV9001_GPIO_10,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_11*/
    ADRV9001_GPIO_11,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_12*/
    ADRV9001_GPIO_12,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_13*/
    ADRV9001_GPIO_13,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_14*/
    ADRV9001_GPIO_14,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_15*/
    ADRV9001_GPIO_15,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_16*/
 
    /******START OF GPIO Analog********/
    ADRV9001_GPIO_ANA_00,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_00*/
    ADRV9001_GPIO_ANA_01,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_01*/
    ADRV9001_GPIO_ANA_02,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_02*/
    ADRV9001_GPIO_ANA_03,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_03*/
    ADRV9001_GPIO_ANA_04,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_04*/
    ADRV9001_GPIO_ANA_05,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_05*/
    ADRV9001_GPIO_ANA_06,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_06*/
    ADRV9001_GPIO_ANA_07,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_07*/
    ADRV9001_GPIO_ANA_08,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_08*/
    ADRV9001_GPIO_ANA_09,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_09*/
    ADRV9001_GPIO_ANA_10,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_10*/
    ADRV9001_GPIO_ANA_11,           /*!< Unique Shared Resource ID for ADI_ADRV9001_GPIO_ANA_11*/
    /***Please add shared resources above this line***/
    ADRV9001_NUM_SHARED_RESOURCES, 
    ADRV9001_SHARED_RESOURCE_INVALID
} adrv9001_SharedResourceID_e;


typedef enum adrv9001_FeatureID
{
    ADRV9001_FEATURE_UNUSED = 0,               /*!< Default feature ID associated with a shared resource when unused*/
    ADRV9001_FEATURE_RX_GAIN_CTRL_PIN,         /*!< Associates Rx Gain Ctrl Pin feature with a shared resource*/
    ADRV9001_FEATURE_RX_PIN_CTRL_ATTN,         /*!< Associates Rx Parallel Pin Ctrl Attenuation feature with a shared resource*/
    ADRV9001_FEATURE_RX_EXT_CTRL_WORD_OUTPUT,  /*!< Associates Rx External Ctrl Word Output feature with a shared resource*/
    ADRV9001_FEATURE_TX_ATTEN_CTRL_PIN,        /*!< Associates Tx Attenuation Ctrl Pin feature with a shared resource*/

    /* FIXME: Vivek- Not sure if this is needed */
    //ADRV9001_FEATURE_SPI2_CTRL_PIN,            /*!< Associates SPI2 Ctrl Pin feature with a shared resource */

    ADRV9001_FEATURE_ARM_GPIO_PIN,             /*!< Associates ARM TDD Ctrl feature with a shared resource */

    /* FIXME: Vivek- Not sure if this is needed */
    //ADRV9001_FEATURE_RX_EXT_SLICER_CTRL,       /*!< Associates Rx External Slicer Ctrl Word Input feature with a shared resource*/
    //ADRV9001_FEATURE_RX_INT_SLICER_CTRL_OUT,   /*!< Associates Rx Internal Slicer Ctrl Word Input feature with a shared resource*/

    ADRV9001_FEATURE_AUX_DAC_OUT,              /*!< Associates AuxDAC feature with a shared resource*/
    ADRV9001_FEATURE_TX_TO_ORX_MAPPING,        /*!< Associates Tx-ORx mapping feature with a shared resource*/
    /***Please add Feature IDs which use shared resources above this line***/
    ADRV9001_NUM_FEATURES,
    ADRV9001_FEATURE_INVALID
} adrv9001_FeatureID_e;


typedef struct adrv9001_SharedResourceLut
{
    adrv9001_SharedResourceType_e sharedResourceType;  /*!< Specifies the type of shared resource for a Look Up Table entry*/
    int32_t sharedResource;                                /*!< Specifies the shared resource value associated with the type for a Look Up Table entry*/
    adrv9001_SharedResourceID_e sharedResourceId;      /*!< Specifies the unique shared resource ID associated with the shared resource for a Look Up Table entry*/
} adrv9001_SharedResourceLut_t;

typedef struct adrv9001_SharedResourcePool
{
    adrv9001_FeatureID_e featureID; /*!< Holds the feature ID of a shared resource pool member */
    uint8_t semaphoreCount;         /*!< Holds the semaphore count of a shared resource pool member */
} adrv9001_SharedResourcePool_t;

#ifdef __cplusplus
}
#endif

#endif /* ! _ADRV9001_SHARED_RESOURCE_MANAGER_TYPES_H_ */