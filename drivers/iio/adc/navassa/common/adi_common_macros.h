/**
* \file
* \brief Contains ADI Transceiver general purpose macros.
* 
* ADI common lib Version: $ADI_COMMON_LIB_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_COMMON_MACROS_H_
#define _ADI_COMMON_MACROS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ADI_COMMON_VERBOSE 1
    
/* yes, we do need the extra parentheses, in case there is a typecast before multiplication */
#define KILO_TO_BASE_UNIT(intVal_k) ((intVal_k) * 1000) 
#define MEGA_TO_BASE_UNIT(intVal_M) ((intVal_M) * 1000 * 1000)
#define GIGA_TO_BASE_UNIT(intVal_G) ((intVal_M) * 1000 * 1000 * 1000)

#define ADI_PERFORM_VALIDATION(rangeCheckFcn, devicePtr, ...) \
{ \
    if(ADI_VALIDATE_PARAMS > 0) \
    { \
        ADI_NULL_DEVICE_PTR_RETURN(devicePtr); \
        ADI_FUNCTION_ENTRY_LOG(&devicePtr->common, ADI_COMMON_LOG_API); \
        ADI_EXPECT(rangeCheckFcn, devicePtr, ##__VA_ARGS__); \
    } \
    else \
    {} \
}

#define ADI_ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

#ifdef __cplusplus
}
#endif
#endif  /* _ADI_COMMON_MACROS_H_ */
