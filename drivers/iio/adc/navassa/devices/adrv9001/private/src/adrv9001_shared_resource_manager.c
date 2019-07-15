/**
 * \file
 * \brief Contains ADRV9001 shared resource related private function implementations
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adrv9001_shared_resource_manager.h"
#include "adi_common_error_types.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_error.h"
#include "adi_common_macros.h"

int32_t adrv9001_SharedResourceMgrReset(adi_adrv9001_Device_t* device)
{
    uint32_t sharedResourceIndex = 0;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_EXPECT(device);

    for (sharedResourceIndex = 0; sharedResourceIndex < ADRV9001_NUM_SHARED_RESOURCES; sharedResourceIndex++)
    {
        device->devStateInfo.sharedResourcePool[sharedResourceIndex].featureID = ADRV9001_FEATURE_UNUSED;
        device->devStateInfo.sharedResourcePool[sharedResourceIndex].semaphoreCount = 0;
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceAvailabilityCheck(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID, uint8_t *sharedResourceAvailable)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_EXPECT(device);

    /*Range check that sharedResourceID is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceID,
        1,
        (ADRV9001_NUM_SHARED_RESOURCES - 1));

    /*Return success if shared resource pool index of shared resource ID has a feature other than UNUSED associated with it*/
    if (device->devStateInfo.sharedResourcePool[sharedResourceID].featureID == ADRV9001_FEATURE_UNUSED)
    {
        *sharedResourceAvailable = (uint8_t)true;
    }
    else
    {
        *sharedResourceAvailable = (uint8_t)false;
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceIdGet(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                     int32_t sharedResource, adrv9001_SharedResourceID_e *sharedResourceID)
{
    static const adrv9001_SharedResourceLut_t sharedResourceLut[] = {
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_00_DO_NOT_ASSIGN, ADRV9001_GPIO_00_DO_NOT_ASSIGN },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_01, ADRV9001_GPIO_01 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_02, ADRV9001_GPIO_02 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_03, ADRV9001_GPIO_03 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_04, ADRV9001_GPIO_04 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_05, ADRV9001_GPIO_05 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_06, ADRV9001_GPIO_06 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_07, ADRV9001_GPIO_07 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_08, ADRV9001_GPIO_08 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_09, ADRV9001_GPIO_09 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_10, ADRV9001_GPIO_10 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_11, ADRV9001_GPIO_11 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_12, ADRV9001_GPIO_12 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_13, ADRV9001_GPIO_13 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_14, ADRV9001_GPIO_14 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO, ADI_ADRV9001_GPIO_15, ADRV9001_GPIO_15 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_00, ADRV9001_GPIO_ANA_00 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_01, ADRV9001_GPIO_ANA_01 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_02, ADRV9001_GPIO_ANA_02 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_03, ADRV9001_GPIO_ANA_03 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_04, ADRV9001_GPIO_ANA_04 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_05, ADRV9001_GPIO_ANA_05 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_06, ADRV9001_GPIO_ANA_06 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_07, ADRV9001_GPIO_ANA_07 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_08, ADRV9001_GPIO_ANA_08 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_09, ADRV9001_GPIO_ANA_09 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_10, ADRV9001_GPIO_ANA_10 },
                                                                        { ADRV9001_SHARED_RESOURCE_GPIO_ANALOG, ADI_ADRV9001_GPIO_ANA_11, ADRV9001_GPIO_ANA_11 }
                                                                    };
    /*****************Please add lut val for shared resources above this line*******************/

    uint32_t sharedResourceIndex = 0;
    uint8_t validSharedResourceTypeFlag = 0;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, sharedResourceID);

    /*Range Check that shared resource type is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceType,
        0,
        (ADRV9001_NUM_SHARED_RESOURCE_TYPES - 1));

    for (sharedResourceIndex = 0; sharedResourceIndex < ADRV9001_NUM_SHARED_RESOURCES; sharedResourceIndex++)
    {
        if (sharedResourceLut[sharedResourceIndex].sharedResourceType == sharedResourceType)
        {
            if (sharedResourceLut[sharedResourceIndex].sharedResource == sharedResource)
            {
                *sharedResourceID = sharedResourceLut[sharedResourceIndex].sharedResourceId;
                validSharedResourceTypeFlag = 1;
                break;
            }
        }
    }

    /* Check not a valid shared resource ID found */
    if (validSharedResourceTypeFlag == 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, sharedResourceType,
                    "Requested Shared Resource is not valid");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceFeatureGet(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                          adrv9001_FeatureID_e *featureID)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, featureID);

    /*Range check that sharedResourceID is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceID,
        1,
        (ADRV9001_NUM_SHARED_RESOURCES - 1));

    /*Return the feature associated with shared resource pool*/
    *featureID = device->devStateInfo.sharedResourcePool[sharedResourceID].featureID;

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceFeatureMaxSemaphoreCntGet(adi_adrv9001_Device_t *device, adrv9001_FeatureID_e featureID, uint8_t *maxSemaphoreCount)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, maxSemaphoreCount);

    /*Range Check that feature ID is valid*/
    ADI_RANGE_CHECK(device,
        featureID,
        1,
        (ADRV9001_NUM_FEATURES - 1));

    switch (featureID)
    {
    case(ADRV9001_FEATURE_RX_GAIN_CTRL_PIN):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_RX_GAIN_CTRL_PIN_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_RX_PIN_CTRL_ATTN):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_RX_PIN_CTRL_ATTN_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_RX_EXT_CTRL_WORD_OUTPUT):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_RX_GAIN_CTRL_PIN_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_TX_ATTEN_CTRL_PIN):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_TX_ATTEN_CTRL_PIN_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_ARM_GPIO_PIN):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_ARM_GPIO_PIN_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_AUX_DAC_OUT):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_AUX_DAC_OUT_MAX_SEMAPHORE_COUNT;
            break;
        }
    case(ADRV9001_FEATURE_TX_TO_ORX_MAPPING):
        {
            *maxSemaphoreCount = (uint8_t)ADRV9001_FEATURE_TX_TO_ORX_MAPPING_MAX_SEMAPHORE_COUNT;
            break;
        }
    default:
        {
        *maxSemaphoreCount = 1;
        break;
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceAcquire(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                       adrv9001_FeatureID_e featureID, uint8_t *resourceAcquisitionStatus)
{
    uint8_t sharedResourceAvailable = (uint8_t)false;
    adrv9001_FeatureID_e currentFeatureID = ADRV9001_FEATURE_UNUSED;
    uint8_t maxSemaphoreCount = 0;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, resourceAcquisitionStatus);

    /*Range check that featureID is valid*/
    ADI_RANGE_CHECK(device,
        featureID,
        1,
        (ADRV9001_NUM_FEATURES - 1));


    /*Range check that sharedResourceID is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceID,
        1,
        (ADRV9001_NUM_SHARED_RESOURCES - 1));

    /*Initialize resourceAcquisitionStatus to false and set to true only if shared resource available*/
    *resourceAcquisitionStatus = false;

    /*sharedResourceID check is done in Shared Resource Available function*/
    /*Acquire resource if it is not already acquired by another feature or already acquired by the requesting feature*/
    ADI_MSG_EXPECT("Error checking for shared resource availability",
        adrv9001_SharedResourceAvailabilityCheck, device,
        sharedResourceID, &sharedResourceAvailable);

    if (sharedResourceAvailable == true)
    {
        device->devStateInfo.sharedResourcePool[sharedResourceID].featureID = featureID;
        device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount++;
        *resourceAcquisitionStatus = true;
    }
    else
    {
        ADI_MSG_EXPECT("Error acquiring shared resource feature",
            adrv9001_SharedResourceFeatureGet, device, sharedResourceID, &currentFeatureID);

        if (currentFeatureID == featureID)
        {
            ADI_EXPECT(adrv9001_SharedResourceFeatureMaxSemaphoreCntGet, device, featureID, &maxSemaphoreCount) ;

            if (device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount < maxSemaphoreCount)
            {
                device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount++;
                *resourceAcquisitionStatus = true;
            }
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourceRelease(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                       adrv9001_FeatureID_e featureID, uint8_t *resourceReleaseStatus)
{
    uint8_t sharedResourceAvailable = false;
    adrv9001_FeatureID_e currentFeatureID = ADRV9001_FEATURE_UNUSED;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, resourceReleaseStatus);

    /*Range check that featureID is valid*/
    ADI_RANGE_CHECK(device,
        featureID,
        1,
        (ADRV9001_NUM_FEATURES - 1));


    /*Range check that sharedResourceID is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceID,
        1,
        (ADRV9001_NUM_SHARED_RESOURCES - 1));

    /*Initialize resourceReleaseStatus to false and set to true only if shared resource is released successfully*/
    *resourceReleaseStatus = false;

    /*sharedResourceID check is done in Shared Resource Available function*/
    /*Release the shared resource only if the requested feature ID matches the current feature ID or is not in use currently*/
    ADI_MSG_EXPECT("Error checking for shared resource availability",
        adrv9001_SharedResourceAvailabilityCheck, device, sharedResourceID, &sharedResourceAvailable);

    if (sharedResourceAvailable == true)
    {
        *resourceReleaseStatus = true;
    }
    else
    {
        ADI_MSG_EXPECT("Error acquiring shared resource feature",
            adrv9001_SharedResourceFeatureGet, device, sharedResourceID, &currentFeatureID);

        if (currentFeatureID == featureID)
        {
            if (device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount > 0)
            {
                device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount--;
            }

            if (device->devStateInfo.sharedResourcePool[sharedResourceID].semaphoreCount == 0)
            {
                device->devStateInfo.sharedResourcePool[sharedResourceID].featureID = ADRV9001_FEATURE_UNUSED;
            }

            *resourceReleaseStatus = true;
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourcesAcquire(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                        int32_t sharedResourceArr[], uint32_t numSharedResources, adrv9001_FeatureID_e featureID,
                                        uint8_t *resourceAcquisitionStatus)
{
    uint32_t sharedResourceIndex = 0;
    adrv9001_SharedResourceID_e sharedResourceID = ADRV9001_SHARED_RESOURCE_INVALID;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, resourceAcquisitionStatus);

    /*Range check that featureID is valid*/
    ADI_RANGE_CHECK(device,
        featureID,
        1,
        (ADRV9001_NUM_FEATURES - 1));

    /*Range Check that shared resource type is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceType,
        0,
        (ADRV9001_NUM_SHARED_RESOURCE_TYPES - 1));

    /*Loop through each shared resource.
      1) Get Shared Resource unique ID
      2) Acquire Shared Resource
      3) If the resource is in use by another feature than the one which requested return a failed status*/
    for (sharedResourceIndex = 0; sharedResourceIndex < numSharedResources; sharedResourceIndex++)
    {
        ADI_MSG_EXPECT("Error attempting to resolve shared resource ID for the requested share resource",
            adrv9001_SharedResourceIdGet, device, sharedResourceType, sharedResourceArr[sharedResourceIndex], &sharedResourceID);

        ADI_MSG_EXPECT("Error attempting to acquire requested share resource",
            adrv9001_SharedResourceAcquire, device, sharedResourceID, featureID, resourceAcquisitionStatus);

        if (*resourceAcquisitionStatus == false)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, resourceAcquisitionStatus,
                        "Error acquiring requested shared resource. Please check if the resource is already in use by another feature.");
            ADI_ERROR_RETURN(device->common.error.newAction);
            break;
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_SharedResourcesRelease(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                        int32_t sharedResourceArr[], uint32_t numSharedResources, adrv9001_FeatureID_e featureID,
                                        uint8_t *resourceReleaseStatus)
{
    uint32_t sharedResourceIndex = 0;
    adrv9001_SharedResourceID_e sharedResourceID = ADRV9001_SHARED_RESOURCE_INVALID;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, resourceReleaseStatus);

    /*Range check that featureID is valid*/
    ADI_RANGE_CHECK(device,
        featureID,
        1,
        (ADRV9001_NUM_FEATURES - 1));

    /*Range Check that shared resource type is valid*/
    ADI_RANGE_CHECK(device,
        sharedResourceType,
        0,
        (ADRV9001_NUM_SHARED_RESOURCE_TYPES - 1));

    /*Loop through each shared resource.
    1) Get Shared Resource unique ID
    2) Release Shared Resource
    3) If the resource is in use by another feature than the one which requested return a failed status*/
    for (sharedResourceIndex = 0; sharedResourceIndex < numSharedResources; sharedResourceIndex++)
    {
        ADI_MSG_EXPECT("Error attempting to resolve shared resource ID for the requested share resource",
            adrv9001_SharedResourceIdGet, device, sharedResourceType, sharedResourceArr[sharedResourceIndex], &sharedResourceID);

        ADI_MSG_EXPECT("Error attempting to release requested share resource",
            adrv9001_SharedResourceRelease, device, sharedResourceID, featureID, resourceReleaseStatus);

        if (*resourceReleaseStatus == false)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, resourceReleaseStatus,
                        "Error releasing requested shared resource. Please check if the resource is already in use by another feature.");
            ADI_ERROR_RETURN(device->common.error.newAction);
            break;
        }
    }

    ADI_API_RETURN(device);
}