/**
 * \file
 * \brief Contains ADRV9001 shared resource related private function prototypes for
 *        adrv9001_shared_resource_manager.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

# ifndef _ADRV9001_SHARED_RESOURCE_MANAGER_H_
#define _ADRV9001_SHARED_RESOURCE_MANAGER_H_

#include "adrv9001_shared_resource_manager_types.h"
#include "adi_adrv9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Resets the shared resource pool for a given adrv9001 device
*
*  Each shared resource is associated with a feature. On reset, all the
*  shared resources are assigned to 'UNUSED' and are ready for use by a given feature.
*
* \pre This function is required to be called during initialization
*
* \param device Pointer to the adrv9001 device data structure
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceMgrReset(adi_adrv9001_Device_t* device);

/**
* \brief Check if a requested shared resource is available for use
*
*  Each shared resource is associated with a feature. Returns a success if the shared resource
*  is unused and a failure if already in use.
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to test shared resource availability
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceID is the shared resource whose availability is being enquired
* \param sharedResourceAvailable result - true or false is returned depending
*                                         on availability of shared resource
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceAvailabilityCheck(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID, uint8_t *sharedResourceAvailable);

/**
* \brief Returns the shared resource ID for a shared resource
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to test shared resource availability
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceType is the shared resource type(GPIO, MEMORY etc) for which ID is being requested
* \param sharedResource is the enum value of the shared resource for which ID is being requested
* \param sharedResourceID result shared resource ID is updated in this variable
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceIdGet(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                     int32_t sharedResource, adrv9001_SharedResourceID_e *sharedResourceID);

/**
* \brief Returns the feature currently consuming the shared resource.
*
*  Each shared resource is associated with a feature in the resource pool.
*  This function returns the feature currently associated with the shared resource
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to enquire shared
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceID is the shared resource whose availability is under query
* \param featureID pointer to the feature associated with the requested shared resource ID
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceFeatureGet(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                          adrv9001_FeatureID_e *featureID);

/**
* \brief Attempts to acquire the shared resource requested from the adrv9001 device data structure shared resource pool
*
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to acquire a shared resource
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceID is the shared resource whose availability is under query
* \param featureID feature requesting the shared resource
* \param resourceAcquisitionStatus stores true if resource acquisition was successful,
*        false otherwise
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceAcquire(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                       adrv9001_FeatureID_e featureID, uint8_t *resourceAcquisitionStatus);

/**
* \brief Attempts to acquire multiple shared resources for a requested feature from the
*        adrv9001 device data structure shared resource pool
*
*  This function returns a failure even if a single resource acquisition fails
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to acquire multiple shared resources for
*      the same feature
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceType is the enum of the shared resource type to be acquired
* \param sharedResourceArr is the array of shared resources to be acquired
* \param numSharedResources is the size of sharedResourceArr
* \param featureID feature requesting the shared resource
* \param resourceAcquisitionStatus stores true if resource acquisition was successful in all cases, false if any resource acquisition failed.
*
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourcesAcquire(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                        int32_t sharedResourceArr[], uint32_t numSharedResources, adrv9001_FeatureID_e featureID,
                                        uint8_t *resourceAcquisitionStatus);
/**
* \brief Attempts to release the shared resource under consumption by the given feature
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to test shared resource availability
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceID is the shared resource whose availability is under query
* \param featureID feature requesting the shared resource
* \param resourceReleaseStatus stores true if resource release was successful,
*        false otherwise
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourceRelease(adi_adrv9001_Device_t* device, adrv9001_SharedResourceID_e sharedResourceID,
                                       adrv9001_FeatureID_e featureID, uint8_t *resourceReleaseStatus);

/**
* \brief Attempts to release multiple shared resources for a requested feature from the
*        adrv9001 device data structure shared resource pool
*
*  This function returns a failure even if a single resource release fails
*
* \pre This function is a private function used internally by the API and can be called
*      anytime after initialization by the API to release multiple shared resources for
*      the same feature
*
* \param device Pointer to the adrv9001 device data structure
* \param sharedResourceType is the enum of the shared resource type to be released
* \param sharedResourceArr is the array of shared resources to be acquired
* \param numSharedResources is the size of sharedResourceArr
* \param featureID feature requesting the shared resource
* \param resourceReleaseStatus "stores true if resource release was successful in all cases, false if any resource release failed."
*
*
* \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
int32_t adrv9001_SharedResourcesRelease(adi_adrv9001_Device_t* device, adrv9001_SharedResourceType_e sharedResourceType,
                                        int32_t sharedResourceArr[], uint32_t numSharedResources, adrv9001_FeatureID_e featureID,
                                        uint8_t *resourceReleaseStatus);

#ifdef __cplusplus
}
#endif

#endif // ! _ADRV9001_SHARED_RESOURCE_MANAGER_H_
