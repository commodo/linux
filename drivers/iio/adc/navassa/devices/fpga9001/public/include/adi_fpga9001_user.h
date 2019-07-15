/**
* \file
* \brief FPGA9001 user-configurable options
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_USER_H_
#define _ADI_FPGA9001_USER_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************
* FPGA Interface Macros
*******************************************/
    
#ifndef ADI_FPGA9001_VERBOSE
#define ADI_FPGA9001_VERBOSE 1                  /*Use strings 0 not use, 1 use */
#endif /* !ADI_FPGA9001_VERBOSE */
    
#ifndef ADI_FPGA9001_VARIABLE_USAGE
#define ADI_FPGA9001_VARIABLE_USAGE 1           /*Use strings 0 not use, 1 use */
#endif /* !ADI_FPGA9001_VARIABLE_USAGE */
    
#ifndef ADI_FPGA9001_LOGGING
#define ADI_FPGA9001_LOGGING 0x7F               /*LogLevel Set to API, Messages, Warnings, Error*/
#endif /* !ADI_FPGA9001_LOGGING */
    
#ifndef FPGA9001_BITFIELD_VALUE_CHECK
#define FPGA9001_BITFIELD_VALUE_CHECK 0         /*Enable bitfield value checks in setters and getters*/
#endif // !FPGA9001_BITFIELD_VALUE_CHECK

#ifndef FPGA9001_BITFIELD_ADDR_CHECK
#define FPGA9001_BITFIELD_ADDR_CHECK 1          /*Enable bitfield address checks in setters and getters*/
#endif // !FPGA9001_BITFIELD_ADDR_CHECK


#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_USER_H_ */