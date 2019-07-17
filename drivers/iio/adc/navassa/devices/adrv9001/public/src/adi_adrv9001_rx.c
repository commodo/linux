/**
* \file
* \brief Contains Rx features related function implementation defined in
* adi_adrv9001_rx.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* Header file corresponding to the C file */
#include "adi_adrv9001_rx.h"

/* ADI specific header files */
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_radio.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_rxb.h"
#include "adrv9001_init.h"
#include "adrv9001_rx.h"

/* Header files related to libraries */


/* System header files */


#ifdef _RELEASE_BUILD_
    #line __LINE__ "adi_adrv9001_rx.c"
#endif

/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

#define ADI_ADRV9001_NUM_GPIOS_IN_RX_GAIN_CTRL_PIN_FEATURE 2U
#define ADI_ADRV9001_NUM_GPIOS_IN_RX_EXT_CTRL_WORD_OUTPUT_FEATURE 8U
#define ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX  8U

/*********************************************************************************************************/
static int32_t adi_adrv9001_Rx_GainCtrlMode_Set_Validate(adi_adrv9001_Device_t *device, 
                                                         adi_common_ChannelNumber_e channel,
                                                         adi_adrv9001_RxGainCtrlMode_e gainCtrlMode)
{
    ADI_API_ENTRY_EXPECT(device);
    ADI_RANGE_CHECK(device, gainCtrlMode, ADI_ADRV9001_MGC, ADI_ADRV9001_AGC);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainCtrlMode_Set(adi_adrv9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_RxGainCtrlMode_e gainCtrlMode)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e rxChannelBitfieldAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainCtrlMode_Set_Validate, device, channel, gainCtrlMode);

    ADI_EXPECT(adrv9001_RxBitfieldAddressGet, device, channel, &rxChannelBitfieldAddr);

    /* FIXME: BITFIELD_MOD Vivek - In Tokelau, auto-generated adrv9010_RxAgcSetupBfSet() has *bfValue as 'uint8_t;
     * In Navassa, this is 'int8_t'. Changing manually the BF API "adrv9001_NvsRegmapRxbAgcSetupBfSet" 
     * prototype to 'uint8_t' for bfValue. This kind of change has been manually done in for few APIs before in Navassa.
     * These changes will have to be made in future whenever a new YODA file is available */

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSetupBfSet, device, rxChannelBitfieldAddr, gainCtrlMode);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_GainCtrlMode_Get_Validate(adi_adrv9001_Device_t *device, 
                                                         adi_common_ChannelNumber_e channel,
                                                         adi_adrv9001_RxGainCtrlMode_e *gainCtrlMode)
{
    ADI_API_ENTRY_PTR_EXPECT(device, gainCtrlMode);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainCtrlMode_Get(adi_adrv9001_Device_t *device,
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_RxGainCtrlMode_e *gainCtrlMode)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;
    uint8_t bfValue = 0;
        
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainCtrlMode_Get_Validate, device, channel, gainCtrlMode);

    switch (channel)
    {
    case(ADI_CHANNEL_1):
        baseAddr = ADRV9001_BF_RXB1_CORE;
        break;
    case(ADI_CHANNEL_2):
        baseAddr = ADRV9001_BF_RXB2_CORE;
        break;
    default:
        /* If ADI_VALIDATE_PARAMS is 1, this line should never be executed, therefore untestable */
        ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    }
        
    /* FIXME: Vivek - In Tokelau, auto-generated adrv9010_RxAgcSetupBfGet() has *bfValue as 'uint8_t;
     * In Navassa, this is 'int8_t'. Chaning manually the BF API "adrv9001_NvsRegmapRxbAgcSetupBfGet" 
     * prototype to 'uint8_t' for bfValue. This kind of change has been manually done in for few APIs before in Navassa.
     * These changes will have to be made in future whenever a new YODA file is available */
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSetupBfGet, device, baseAddr, &bfValue);
    *gainCtrlMode = (adi_adrv9001_RxGainCtrlMode_e)bfValue;
    
    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_GainTable_Write_Validate(adi_adrv9001_Device_t *device,
                                                        uint32_t rxChannelMask,
                                                        uint8_t  gainIndexOffset,
                                                        adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                                        uint32_t arraySize)
{
    static const uint8_t MAX_EXT_CTRL_WORD = 3;
    static const uint8_t MAX_ADC_TIA_GAIN  = 63;
    static const int16_t MIN_DIG_GAIN = -360; /*Dig gain is in the range -18dB to 50db*/
    static const int16_t MAX_DIG_GAIN = 1000; /*Dig gain is in the range -18dB to 50db*/

    uint16_t  gainIndex = 0;

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, gainTableRows, arraySize);

    /*Check that the gain index offset is within range*/
    ADI_RANGE_CHECK(device, gainIndexOffset, ADI_ADRV9001_MIN_RX_GAIN_TABLE_INDEX, ADI_ADRV9001_START_RX_GAIN_INDEX);

    /*Check the no. of gain indices parameter is valid*/
    if (arraySize > ((gainIndexOffset - ADI_ADRV9001_MIN_RX_GAIN_TABLE_INDEX) + 1))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         arraySize,
                         "gainTableRow arraySize exceeds the limit. Valid range 0 to (gainIndexOffset - ADI_ADRV9001_MIN_RX_GAIN_TABLE_INDEX +1)");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Check that rxChannelMask is valid*/
    ADI_RANGE_CHECK(device, rxChannelMask, ADI_CHANNEL_1, (ADI_CHANNEL_1 | ADI_CHANNEL_2));

    /*Check that Rx Profile is valid*/
    if ((device->devStateInfo.profilesValid & ADI_ADRV9001_RX_PROFILE_VALID) == 0)
    {
        ADI_ERROR_REPORT(&device->common,
                            ADI_COMMON_ERRSRC_API,
                            ADI_COMMON_ERR_INV_PARAM,
                            ADI_COMMON_ACT_ERR_CHECK_PARAM,
                            rxChannelMask,
                            "Rx profile is invalid");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Range check individual gain table row entries*/
    for (gainIndex = 0; gainIndex < arraySize; gainIndex++)
    {
        /*Check that {TIA GAIN, ADC CTRL} are 0-63 */
        ADI_RANGE_CHECK(device, gainTableRows[gainIndex].adcTiaGain, 0, MAX_ADC_TIA_GAIN);

        /*Check that EXT_CTRL is in the range {0,1,2,3}*/
        ADI_RANGE_CHECK(device, gainTableRows[gainIndex].extControl, 0, MAX_EXT_CTRL_WORD);

        /*Check that digital gain is in the range -18dB to 50dB*/
        ADI_RANGE_CHECK(device, gainTableRows[gainIndex].digGain, MIN_DIG_GAIN, MAX_DIG_GAIN);
    }

    ADI_API_RETURN(device);
}


int32_t adi_adrv9001_Rx_GainTable_Write(adi_adrv9001_Device_t *device,
                                        uint32_t channelMask,
                                        uint8_t  gainIndexOffset,
                                        adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                        uint32_t arraySize)
{  
    uint32_t baseIndex = 0;
    uint32_t baseAddress = 0;
    uint16_t numGainIndicesToWrite = 0;
    
    /*Maximum Array Size = Max Gain Table Size x Bytes Per Gain Table Entry*/
    static uint8_t armDmaData[((ADI_ADRV9001_MAX_GAIN_TABLE_INDEX - ADI_ADRV9001_MIN_GAIN_TABLE_INDEX) + 1) * ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX] = { 0 };
    static const uint8_t DEC_POWER_CONFIG1 = 0x09; /* dec_power_log_shift | dec_power_enable_meas */
    static const uint8_t DIGITAL_GAIN_CONFIG2 = 0x01; /* digital_gain_enable */
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainTable_Write_Validate, device, channelMask, gainIndexOffset, gainTableRows, arraySize);

    /*Calculate base index for the config*/
    numGainIndicesToWrite = arraySize;
    baseIndex = (gainIndexOffset - (numGainIndicesToWrite - 1));

    /*Format Gain Table Entries*/
    ADI_EXPECT(adrv9001_RxGainTableFormat, device, gainTableRows, &armDmaData[0], numGainIndicesToWrite);

    /*Resolve the RX Channel SRAM to program*/
    /*If Rx1 Channel Mask Set by user for this config, load Rx1 gain table*/
    if (ADRV9001_BF_EQUAL(channelMask, ADI_CHANNEL_1))
    {
        /*Resolve Rx1 Gain Table SRAM load start address*/
        baseAddress = (uint32_t)ADI_ADRV9001_RX1_GAIN_TABLE_BASEADDR + (baseIndex * ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX);
        /*Write to the SRAM via ARM DMA*/
        ADI_MSG_EXPECT("Error writing to ARM DMA while loading Rx Gain Table @ base address 0x73300000",
                           adrv9001_DmaMemWrite,
                           device,
                           baseAddress,
                           &armDmaData[0],
                           (uint32_t)(numGainIndicesToWrite * ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX));
 
        ADRV9001_SPIWRITEBYTE(device,
                              "RX1_DEC_POWR_CONFIG_1", 
                              (ADRV9001_ADDR_CH1_RXB + ADRV9001_ADDR_RXB_DEC_POWER_CONFIG1_OFFSET),
                              DEC_POWER_CONFIG1);
        
        ADRV9001_SPIWRITEBYTE(device,
                              "RX1_DIGITAL_GAIN_CONFIG2", 
                              (ADRV9001_ADDR_CH1_RX + ADRV9001_ADDR_RX_DIGITAL_GAIN_CONFIG2_OFFSET),
                              DIGITAL_GAIN_CONFIG2);
    }

    /*If Rx2 Channel Mask Set by user for this config, load Rx2 gain table*/
    if (ADRV9001_BF_EQUAL(channelMask, ADI_CHANNEL_2))
    {
        /*Resolve Rx2 Gain Table SRAM load start address*/
        baseAddress = (uint32_t)ADI_ADRV9001_RX2_GAIN_TABLE_BASEADDR + (baseIndex * ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX);
        /*Write to the SRAM via ARM DMA*/
        ADI_MSG_EXPECT("Error writing to ARM DMA while loading Rx Gain Table @ base address 0x73400000",
                           adrv9001_DmaMemWrite,
                           device,
                           baseAddress,
                           &armDmaData[0],
                           (uint32_t)(numGainIndicesToWrite * ADI_ADRV9001_NUM_BYTES_PER_RX_GAIN_INDEX));
        
        ADRV9001_SPIWRITEBYTE(device,
                              "RX2_DEC_POWR_CONFIG_1", 
                              (ADRV9001_ADDR_CH2_RXB + ADRV9001_ADDR_RXB_DEC_POWER_CONFIG1_OFFSET),
                              DEC_POWER_CONFIG1);
        
        ADRV9001_SPIWRITEBYTE(device,
                              "RX2_DIGITAL_GAIN_CONFIG2", 
                              (ADRV9001_ADDR_CH2_RX + ADRV9001_ADDR_RX_DIGITAL_GAIN_CONFIG2_OFFSET),
                              DIGITAL_GAIN_CONFIG2);

    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_MinMaxGainIndex_Set_Validate(adi_adrv9001_Device_t *device,
                                                            uint32_t rxChannelMask,
                                                            uint8_t minGainIndex,
                                                            uint8_t maxGainIndex)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);
    
    /*Check that rxChannelMask is valid*/
    ADI_RANGE_CHECK(device, rxChannelMask, ADI_CHANNEL_1, (ADI_CHANNEL_1 | ADI_CHANNEL_2));

    /*Check that requested min gain index does not exceed max gain index*/
    if (minGainIndex >= maxGainIndex)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device,
                         "minGainIndex should be less than maxGainIndex");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_MinMaxGainIndex_Set(adi_adrv9001_Device_t *device,
                                            uint32_t rxChannelMask,
                                            uint8_t minGainIndex,
                                            uint8_t maxGainIndex)
{
    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device); 

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_MinMaxGainIndex_Set_Validate, device, rxChannelMask, minGainIndex, maxGainIndex);

    /*Update device gain table min and max gain indices*/
    if ((rxChannelMask & (uint32_t)ADI_ADRV9001_RX1) == (uint32_t)ADI_ADRV9001_RX1)
    {
        device->devStateInfo.gainIndexes.rx1MaxGainIndex = maxGainIndex;
        device->devStateInfo.gainIndexes.rx1MinGainIndex = minGainIndex;
    }

    if ((rxChannelMask & (uint32_t)ADI_ADRV9001_RX2) == (uint32_t)ADI_ADRV9001_RX2)
    {
        device->devStateInfo.gainIndexes.rx2MaxGainIndex = maxGainIndex;
        device->devStateInfo.gainIndexes.rx2MinGainIndex = minGainIndex;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_GainTable_Read_Validate(adi_adrv9001_Device_t *device,
                                                       adi_common_ChannelNumber_e channel,
                                                       uint8_t  gainIndexOffset,
                                                       adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                                       uint32_t arraySize,
                                                       uint16_t *numGainIndicesRead)
{ 
    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, gainTableRows, arraySize);
    
    /* numGainIndicesRead is the actual no.of gain indices read from SRAM(output).A NULL can be passed
    * if the value of no.of gain indices actually read is not required.
    */
    // ADI_NULL_PTR_RETURN(&device->common, numGainIndicesRead);
    
    /*Check that the channel requested is valid*/
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    if (arraySize > ((ADI_ADRV9001_MAX_GAIN_TABLE_INDEX - ADI_ADRV9001_MIN_GAIN_TABLE_INDEX) + 1))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         arraySize,
                         "Invalid arraySize parameter requested for Rx Channel Gain Table Read. Valid range is 0 to (ADI_ADRV9001_MAX_GAIN_TABLE_INDEX - ADI_ADRV9001_MIN_GAIN_TABLE_INDEX) + 1)");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Check that gainIndexOffset is correct*/
    if (channel == ADI_CHANNEL_1)
    {
        ADI_RANGE_CHECK(device,
                            gainIndexOffset,
                            device->devStateInfo.gainIndexes.rx1MinGainIndex,
                            device->devStateInfo.gainIndexes.rx1MaxGainIndex);
    }
    else if (channel == ADI_CHANNEL_2)
    {
        ADI_RANGE_CHECK(device,
                            gainIndexOffset,
                            device->devStateInfo.gainIndexes.rx2MinGainIndex,
                            device->devStateInfo.gainIndexes.rx2MaxGainIndex);
    }
    else
    {
        /* Should never reach here, included only for style purposes */
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         port,
                         "Invalid port or channel requested for gain table read. Port must be RX or ORX. Channel must be CHANNEL_1 or CHANNEL_2");
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainTable_Read(adi_adrv9001_Device_t *device,
                                       adi_common_ChannelNumber_e channel,
                                       uint8_t  gainIndexOffset,
                                       adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                       uint32_t arraySize,
                                       uint16_t *numGainIndicesRead)
{                                     
    static const uint32_t NUM_BYTES_PER_GAIN_INDEX = 8;
    static const uint32_t ARM_DMA_AUTO_INCR = 0;

    uint32_t baseAddress = 0;
    uint32_t baseIndex = 0;
    uint16_t maxGainIndices = 0;
    uint16_t numGainIndicesToRead = 0;
    /*Maximum Array Size = Max Gain Table Size x Bytes Per Gain Table Entry*/
    static uint8_t armDmaData[ADI_ADRV9001_GAIN_TABLE_ARRAY_SIZE] = { 0 };

    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, gainTableRows, arraySize);
    /* numGainIndicesRead is the actual no.of gain indices read from SRAM(output).A NULL can be passed
    * if the value of no.of gain indices actually read is not required.
    */
    //ADI_NULL_PTR_RETURN(&device->common, numGainIndicesRead);	
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainTable_Read_Validate, device, 
                           channel, gainIndexOffset, gainTableRows, arraySize, numGainIndicesRead);

    /*Calculate no. of indices to read and the base address for the config*/
    if (channel == ADI_CHANNEL_1)
    {
        maxGainIndices = (gainIndexOffset - device->devStateInfo.gainIndexes.rx1MinGainIndex) + 1;
        baseAddress = (uint32_t)ADI_ADRV9001_RX1_GAIN_TABLE_BASEADDR;
    }
    else if (channel == ADI_CHANNEL_2)
    {
        maxGainIndices = (gainIndexOffset - device->devStateInfo.gainIndexes.rx2MinGainIndex) + 1;
        baseAddress = (uint32_t)ADI_ADRV9001_RX2_GAIN_TABLE_BASEADDR;
        
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid Channel requested for gain table read. Valid channels include Rx1-Rx2");
        ADI_API_RETURN(device);
    }
    
    if (arraySize >= maxGainIndices)
    {
        numGainIndicesToRead = maxGainIndices;
    }
    else
    {
        numGainIndicesToRead = arraySize;
    }
    baseIndex = (gainIndexOffset - (numGainIndicesToRead - 1));
    baseAddress += baseIndex * NUM_BYTES_PER_GAIN_INDEX;

    /*Read Gain Table Data for the requested channel via ARM DMA*/
    ADI_MSG_EXPECT("Error Reading Gain Table ARM DMA",
                       adrv9001_DmaMemRead,
                       device,
                       baseAddress,
                       &armDmaData[0],
                       (numGainIndicesToRead * NUM_BYTES_PER_GAIN_INDEX),
                       ARM_DMA_AUTO_INCR);

    /*Parse gain table data obtained in ARM DMA data format to an rx gain table row entry datastructure memory*/
    ADI_MSG_EXPECT("Error parsing gain table data",
                       adrv9001_RxGainTableParse,
                       device,
                       &gainTableRows[0],
                       &armDmaData[0],
                       numGainIndicesToRead);
    
    /* numGainIndicesRead is the actual no.of gain indices read from SRAM(output).A NULL can be passed
     * if the value of no.of gain indices actually read is not required.
     */
    /*Update no. of gain indices read*/
    if (numGainIndicesRead != NULL)
    {
        *numGainIndicesRead = numGainIndicesToRead;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_Gain_Set_Validate(adi_adrv9001_Device_t *device,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint8_t gainIndex)
{
    /* Check for valid channel */
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    
    /*Check that Rx profile is valid*/
    if ((device->devStateInfo.profilesValid & ADI_ADRV9001_RX_PROFILE_VALID) == 0)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Rx gain index set requested for an Rx Channel but Rx profile is invalid in the device structure");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /*Check that gain indices are within range for the channel selected*/
    if (channel == ADI_CHANNEL_1)
    {
        ADI_RANGE_CHECK(device,
                            gainIndex,
                            device->devStateInfo.gainIndexes.rx1MinGainIndex,
                            device->devStateInfo.gainIndexes.rx1MaxGainIndex);
    }

    if (channel == ADI_CHANNEL_2)
    {
        ADI_RANGE_CHECK(device,
                            gainIndex,
                            device->devStateInfo.gainIndexes.rx2MinGainIndex,
                            device->devStateInfo.gainIndexes.rx2MaxGainIndex);
    }

    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Rx_Gain_Set(adi_adrv9001_Device_t *device,
                                 adi_common_ChannelNumber_e channel,
                                 uint8_t gainIndex)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Gain_Set_Validate, device, channel, gainIndex);

    /*Update manual gain index setting for the requested channel */
    if (channel == ADI_CHANNEL_1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainIndexBfSet, device, ADRV9001_BF_RXB1_CORE, gainIndex);
    }

    if (channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainIndexBfSet, device, ADRV9001_BF_RXB2_CORE, gainIndex);
    }

    ADI_API_RETURN(device)
}

static int32_t adi_adrv9001_Rx_Gain_Get_Validate(adi_adrv9001_Device_t *device, 
                                                 adi_common_ChannelNumber_e channel, 
                                                 uint8_t *gainIndex)
{
    ADI_NULL_PTR_RETURN(&device->common, gainIndex);
    
    /*Check that the requested channel is valid*/
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    
    /*Check that Rx profile is valid in current config*/
    if ((device->devStateInfo.profilesValid & ADI_ADRV9001_RX_PROFILE_VALID) == 0)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Rx Gain index read requested but Rx profile is invalid in device structure");
    }

    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Rx_MgcGain_Get(adi_adrv9001_Device_t *device, 
                                    adi_common_ChannelNumber_e channel, 
                                    uint8_t *gainIndex)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Gain_Get_Validate, device, channel, gainIndex);
    
    /* Check the Rx channel for which the gain to be set */
    if (channel == ADI_CHANNEL_1)
    {
        /* FIXME: Vivek - In Tokelau, auto-generated adrv9010_RxAgcGainIndexBfGet() has *bfValue as 'uint8_t;
         * In Navassa, this is 'int8_t'. Chaning manually the BF API "adrv9001_NvsRegmapRxbAgcGainIndexBfGet" 
         * prototype to 'uint8_t' for bfValue. This kind of change has been manually done in for few APIs before in Navassa.
         * These changes will have to be made in future whenever a new YODA file is available */

        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainIndexBfGet, device, ADRV9001_BF_RXB1_CORE, gainIndex);
    }

    if (channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainIndexBfGet, device, ADRV9001_BF_RXB2_CORE, gainIndex);
    }

    ADI_API_RETURN(device)
}

int32_t adi_adrv9001_Rx_Gain_Get(adi_adrv9001_Device_t *device, adi_common_ChannelNumber_e channel, uint8_t *gainIndex)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_Gain_Get_Validate, device, channel, gainIndex);

    /* Check the Rx channel for which the manual gain to be set */
    if (channel == ADI_CHANNEL_1)
    {
        /* FIXME: Vivek - In Tokelau, auto-generated adrv9010_RxCurrentGainIndexBfGet() has *bfValue as 'uint8_t;
         * In Navassa, this is 'int8_t'. Chaning manually the BF API "adrv9001_NvsRegmapRxbCurrentGainIndexBfGet" 
         * prototype to 'uint8_t' for bfValue. This kind of change has been manually done in for few APIs before in Navassa.
         * These changes will have to be made in future whenever a new YODA file is available */

        ADI_EXPECT(adrv9001_NvsRegmapRxbCurrentGainIndexBfGet, device, ADRV9001_BF_RXB1_CORE, gainIndex);
    }

    if (channel == ADI_CHANNEL_2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbCurrentGainIndexBfGet, device, ADRV9001_BF_RXB2_CORE, gainIndex);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_Rssi_Read(adi_adrv9001_Device_t *device, 
                                  adi_common_ChannelNumber_e channel, 
                                  adi_adrv9001_RxRssiStatus_t *rxRssiStatus)
{
    uint8_t channelMask = 0;
    uint8_t armExtData[2] = { 0 };
    uint8_t armReadBack[24] = { 0 };

    /* Variables to read  linearPower(of type 'double') and dBPower(of type float)*/
    uint32_t dBPowerTemp  = 0;
    uint64_t linearPowerTemp = 0;
    float    *dBPowerPtr  = NULL;
    double   *linearPowerPtr = NULL;

    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;

    ADI_PERFORM_VALIDATION(adrv9001_RxRssiReadValidate, device, rxRssiStatus, channel);

    /* Range check is already performed. So at this point, if channel is not RX1 then it must be RX2 */
    if (ADRV9001_BF_EQUAL(channel, ADI_ADRV9001_RX1))
    {
        channelMask |= ADRV9001_MAILBOX_CHANNEL_RX1;
    }
    else
    {
        channelMask |= ADRV9001_MAILBOX_CHANNEL_RX2;
    }

    armExtData[0] = channelMask;
    armExtData[1] = ADRV9001_ARM_OBJECTID_RSSI;

    /* send ARM GET opcode */
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
                   device,
                   (uint8_t)ADRV9001_ARM_GET_OPCODE,
                   &armExtData[0],
                   sizeof(armExtData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        armExtData[1],
        ADI_ADRV9001_READ_RSSI_TIMEOUT_US,
        ADI_ADRV9001_READ_RSSI_INTERVAL_US);

#if ADI_ADRV9001_SW_TEST > 0
    static const uint32_t CODECHECK_PARAM_READRSSI_GET_ERR2 = 2;
    /* SW Test */
    if (device->devStateInfo.swTest == CODECHECK_PARAM_READRSSI_GET_ERR2)
    {
        cmdStatusByte = 2;
    }
#endif

    /* read the ARM memory to get RSSI status */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read, 
                   device,
                   ADRV9001_ADDR_ARM_MAILBOX_GET,
                   &armReadBack[0],
                   sizeof(armReadBack),
                   ARM_MEM_READ_AUTOINCR)

    /*
     * Comment (MH):
     * Why do we need floating point here?
     * Cat't this just return the Value in milli dB?
     */

    /* Read linearPower */
    linearPowerTemp = (uint64_t)((uint64_t)armReadBack[0] |
                                ((uint64_t)armReadBack[1] << 8) |
                                ((uint64_t)armReadBack[2] << 16) |
                                ((uint64_t)armReadBack[3] << 24) |
                                ((uint64_t)armReadBack[4] << 32) |
                                ((uint64_t)armReadBack[5] << 40) |
                                ((uint64_t)armReadBack[6] << 48) |
                                ((uint64_t)armReadBack[7] << 56));

    linearPowerPtr = (double*)&linearPowerTemp;
    rxRssiStatus->linearPower = *linearPowerPtr;

    /* Read dBPower */
    dBPowerTemp = (uint32_t)((uint32_t)armReadBack[8] |
                            ((uint32_t)armReadBack[9] << 8) |
                            ((uint32_t)armReadBack[10] << 16) |
                            ((uint32_t)armReadBack[11] << 24));

    dBPowerPtr = (float*)&dBPowerTemp;
    rxRssiStatus->dBPower = *dBPowerPtr;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_DecimatedPower_Get(adi_adrv9001_Device_t *device, 
                                           adi_common_ChannelNumber_e channel,
                                           uint16_t *rxDecPower_mdBFS)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;
    uint8_t bfValue = 0;

    static const uint8_t RX_DEC_POWER_MULT_mdB = 250; /* 250 = 1000 * 0.25dB */
    static const uint8_t DEC_MAX_POWER = 252;

    ADI_API_ENTRY_PTR_EXPECT(device, rxDecPower_mdBFS);

    if (channel == ADI_CHANNEL_1)
    {
        baseAddr = ADRV9001_BF_RXB1_CORE;
    }
    else if (channel == ADI_CHANNEL_2)
    {
        baseAddr = ADRV9001_BF_RXB2_CORE;
    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid channel");

        ADI_API_RETURN(device); 
    }

    /* 
     * This register contains the decimated power readback value for the requested Rx channel datapath. Resolution is 250mdB. 
     * This register must be written(any value) to generate a strobe that will latch the value to be read back.
     */		 
    ADRV9001_SPIWRITEBYTE(device,
                          "RX_DECIMATED_PWR", 
                          (baseAddr + ADRV9001_ADDR_RX_DECIMATED_PWR_OFFSET),
                          0x0F);

    /* Read decimated power for the given channel */
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerBfGet, 
                   device, 
                   baseAddr, 
                   &bfValue);

    /* 
     * Checking the DECIMATED Power range. The valid range is from 0 to -63 dB.
     * With 0.25 dB step, the bfValue can't be greater than 252. 
     * (i.e. with bfValue == 100 -> DEC power is (100 / 4) = 25 dB,
     * negative sign is implicit so it is -25 dB. */
    if (bfValue > DEC_MAX_POWER) 
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_SPI_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_INTERFACE,
                         channel,
                         "Decimated Power is out of range.");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    
    *rxDecPower_mdBFS = (((uint16_t)bfValue) * RX_DEC_POWER_MULT_mdB);

    ADI_API_RETURN(device); 
}

static int32_t adi_adrv9001_Rx_InterfaceGain_Configure_Validate(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainCtrl)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&device->common, rxInterfaceGainCtrl);

    /*Check that gainControlMode selected is valid*/
    ADI_RANGE_CHECK(device,
        rxInterfaceGainCtrl->gainControlMode,
        ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC,
        ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL);

    /*Check that updateInstance selected is valid*/
    ADI_RANGE_CHECK(device,
        rxInterfaceGainCtrl->updateInstance,
        ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NEXT_FRAME,
        ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NOW);

    /*Check that gain selected is valid*/
    ADI_RANGE_CHECK(device,
        rxInterfaceGainCtrl->gain,
        ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB,
        ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_InterfaceGain_Configure(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainCtrl)
{
    uint8_t armData[4] = { 0 };
    uint8_t extData[5] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_InterfaceGain_Configure_Validate, device, channel, rxInterfaceGainCtrl);

    armData[0] = (uint8_t)(rxInterfaceGainCtrl->updateInstance);
    armData[1] = (uint8_t)(rxInterfaceGainCtrl->gainControlMode);
    armData[2] = (uint8_t)(rxInterfaceGainCtrl->gain);
    armData[3] = rxInterfaceGainCtrl->signalPar_dB;

    /* Write RX interface gain control parameters to ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, device, ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);

    /* Executing the SET RX interface gain control command */
    extData[1] = ADRV9001_ARM_OBJECTID_RX_INTERFACE_GAIN_CONTROL;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_InterfaceGain_Set_Validate(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGain_e gain)
{
    adrv9001_RxInterfaceGainCtrl_t rxInterfaceGainCtrl = { 0 };

    /* Check device pointer is not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_EXPECT(adi_adrv9001_Rx_InterfaceGain_Inspect, device, channel, &rxInterfaceGainCtrl);

    /* adi_adrv9001_RxInterfaceGain_Set() is allowed only in Manual mode */
    if (rxInterfaceGainCtrl.gainControlMode != ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            rxInterfaceGainCtrl->gainControlMode,
            "Invalid gainControlMode - must be in manual mode to set gain");

        ADI_API_RETURN(device);
    }

    /*Check that gain selected is valid*/
    ADI_RANGE_CHECK(device, gain, ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB, ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_InterfaceGain_Set(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGain_e gain)
{
    uint8_t armData[4] = { 0 };
    uint8_t extData[5] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_InterfaceGain_Set_Validate, device, channel, gain);

    armData[0] = (uint8_t)gain;

    /* Write RX interface gain control parameters to ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, device, (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);

    /* Executing the SET RX interface gain command */
    extData[1] = ADRV9001_ARM_OBJECTID_RX_INTERFACE_GAIN;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_InterfaceGain_Inspect_Validate(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainCtrl)
{
    /* Check device pointer and rxInterfaceGainCtrl are not null */
    ADI_API_ENTRY_PTR_EXPECT(device, rxInterfaceGainCtrl);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_InterfaceGain_Inspect(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainCtrl)
{
    uint8_t armReadBack[4] = { 0 };
    uint8_t extData[5] = { 0 };

    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_InterfaceGain_Inspect_Validate, device, channel, rxInterfaceGainCtrl);

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);

    /* Executing the SET RX interface gain control command */
    extData[1] = ADRV9001_ARM_OBJECTID_RX_INTERFACE_GAIN_CONTROL;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_GET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_INTERVAL_US);

    /* read the ARM memory to get RSSI status */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
        device,
        ADRV9001_ADDR_ARM_MAILBOX_GET,
        &armReadBack[0],
        sizeof(armReadBack),
        ARM_MEM_READ_AUTOINCR)

    rxInterfaceGainCtrl->updateInstance  = (adrv9001_RxInterfaceGainCtrl_Update_e)armReadBack[0];
    rxInterfaceGainCtrl->gainControlMode = (adrv9001_RxInterfaceGainCtrl_Type_e)armReadBack[1];
    rxInterfaceGainCtrl->gain            = (adrv9001_RxInterfaceGain_e)armReadBack[2];
    rxInterfaceGainCtrl->signalPar_dB    = armReadBack[3];

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_InterfaceGain_Get_Validate(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGain_e *gain)
{
    /* Check device pointer and gain pointer are not null */
    ADI_API_ENTRY_PTR_EXPECT(device, gain);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_InterfaceGain_Get(adi_adrv9001_Device_t *device,
    adi_common_ChannelNumber_e channel,
    adrv9001_RxInterfaceGain_e *gain)
{
    uint8_t armReadBack[4] = { 0 };
    uint8_t extData[5] = { 0 };

    static const uint8_t ARM_MEM_READ_AUTOINCR = 1;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_InterfaceGain_Get_Validate, device, channel, gain);

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);

    /* Executing the SET RX interface gain control command */
    extData[1] = ADRV9001_ARM_OBJECTID_RX_INTERFACE_GAIN;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_GET_OPCODE,
        extData[1],
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_RX_INTERFACE_CONTROL_INTERVAL_US);

    /* read the ARM memory to get Rx interface Gain */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
        device,
        ADRV9001_ADDR_ARM_MAILBOX_GET,
        &armReadBack[0],
        sizeof(armReadBack),
        ARM_MEM_READ_AUTOINCR)

    *gain = (adrv9001_RxInterfaceGain_e)armReadBack[0];

    ADI_API_RETURN(device);
}
