/*!
* \file
* \brief TDD types
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_TDD_TYPES_H_
#define _ADI_FPGA9001_TDD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
   
#include "adi_adrv9001_types.h"

/**
* The TDD timing, sequence, and framing parameters are illustrated in the following timing diagram.
* 
* \image html "FPGA TDD timing diagram.png" "FPGA TDD timing diagram"
*/
    
/**
* \brief Timing of enable assert/deassert within a TDD frame
* 
* The TDD counter starts at 1, so primaryAssert must be greater than 0. 
* The secondary enable event is optional - set secondaryAssert = secondaryDeassert = 0 to ignore.
*/
typedef struct adi_fpga9001_TddTiming
{
    uint32_t primaryAssert;         /*!< Number of tdd_clk cycles after start of TDD frame to perform primary assert */
    uint32_t primaryDeassert;       /*!< Number of tdd_clk cycles after start of TDD frame to perform primary deassert */
    uint32_t secondaryAssert;       /*!< Number of tdd_clk cycles after start of TDD frame to perform secondary assert */
    uint32_t secondaryDeassert;     /*!< Number of tdd_clk cycles after start of TDD frame to perform secondary deassert */
} adi_fpga9001_TddTiming_t;
    
/**
* \brief Sequencing of TDD frame enabledness
* 
* If numberFrames is 0 these parameters have no effect.
*/
typedef struct adi_fpga9001_TddSequence
{
    uint32_t pauseFrames;       /*!< Number of TDD frames to wait before commencing enable/disable sequencing */
    uint32_t activeFrames;      /*!< Number of consecutive TDD frames to apply primary and secondary enable/disable region */
    uint32_t inactiveFrames;    /*!< Number of consecutive TDD frames to not apply primary and secondary enable/disable region */
} adi_fpga9001_TddSequence_t;

typedef struct adi_fpga9001_TddDatapathControl
{
    bool enable;        /*!< Enable the datapath */
    uint32_t start;     /*!< Start Value for Datapath Control Module. Effectively the Datapath Delay value */
} adi_fpga9001_TddDatapathControl_t;

typedef struct adi_fpga9001_TddChannel
{
    adi_fpga9001_TddTiming_t pinTiming;                 /*!< Controls the timing of the GPIO enable pin transitions */
    adi_fpga9001_TddSequence_t pinSequence;             /*!< Controls the sequencing of the GPIO enable pin */
    adi_fpga9001_TddTiming_t dmaTiming;                 /*!< Controls the timing of the DMA enabledness; Does not apply to txGpioControls */
    adi_fpga9001_TddSequence_t dmaSequence;             /*!< Controls the sequencing of the DMA enabledness; Does not apply to txGpioControls */
    adi_fpga9001_TddDatapathControl_t datapathControl;  /*!< Additional datapath gate/delay; Does not apply to txGpioControls */
} adi_fpga9001_TddChannel_t;

typedef struct adi_fpga9001_TddFraming
{
    bool enableSyncExtTrig;     /*!< Set this value to 1 to enable external triggering. Otherwise, set to 0 */
    uint32_t framePeriod;       /*!< Duration of TDD frame in tdd_clk cycles */
    uint32_t numberFrames;      /*!< Desired number of TDD frames in sequence; set to 0 for infinite */
} adi_fpga9001_TddFraming_t;

typedef struct adi_fpga9001_TddConfig
{
    adi_fpga9001_TddFraming_t framing;

    adi_fpga9001_TddChannel_t rxControls[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t txControls[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t orxControls[ADI_ADRV9001_NUM_CHANNELS];
    
    adi_fpga9001_TddChannel_t txGpioControls[ADI_ADRV9001_NUM_CHANNELS];
} adi_fpga9001_TddConfig_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_TDD_TYPES_H_ */
