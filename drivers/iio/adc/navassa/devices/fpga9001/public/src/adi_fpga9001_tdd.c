/**
 * \file
 * \brief Contains top level TDD fpga9001 related functions
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdbool.h>
#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_tdd.h"
#include "adi_fpga9001.h"

#include "fpga9001_bf_tdd_dp_ctrl.h"
#include "fpga9001_bf_axi_tdd_enable.h"
#include "fpga9001_bf_axi_tdd_frame.h"
#include "fpga9001_decode_bf_enum.h"

static int32_t adi_fpga9001_Channel_Validate(adi_fpga9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Port_Validate(adi_fpga9001_Device_t *device,
                                          adi_common_Port_e port)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_ORX);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ManualEnable_Set_Validate(adi_fpga9001_Device_t *device,
                                                      adi_common_Port_e port,
                                                      adi_common_ChannelNumber_e channel)
{
    ADI_EXPECT(adi_fpga9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_fpga9001_Port_Validate, device, port);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ManualEnable_Set(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         bool enable)
{
    fpga9001_BfAxiTddEnableChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ManualEnable_Set_Validate, device, port, channel);
    
    baseAddr = fpga9001_TddEnableChanAddr_Get(port, channel);
    
    /* TODO: Move fpga9001_AxiTddEnableDevEnableBfSet to adi_fpga9001_Tdd_Configure */
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryenableBfSet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddEnablePinmanualassertBfSet, device, baseAddr, enable);
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ManualEnable_Get_Validate(adi_fpga9001_Device_t *device,
                                                      adi_common_Port_e port,
                                                      adi_common_ChannelNumber_e channel,
                                                      bool *enable)
{
    ADI_EXPECT(adi_fpga9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_fpga9001_Port_Validate, device, port);
    
    ADI_NULL_PTR_RETURN(&device->common, enable);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ManualEnable_Get(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         bool *enable)
{
    fpga9001_BfAxiTddEnableChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ManualEnable_Get_Validate, device, port, channel, enable);
    
    baseAddr = fpga9001_TddEnableChanAddr_Get(port, channel);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinmanualassertBfGet, device, baseAddr, (uint8_t*)enable);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddFraming_Set(adi_fpga9001_Device_t *device, adi_fpga9001_TddFraming_t *framing)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;

    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfSet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfSet, device, baseAddr, false);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddtriggerenableBfSet, device, baseAddr, framing->enableSyncExtTrig);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfSet, device, baseAddr, framing->framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfSet, device, baseAddr, framing->numberFrames);
        
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddFraming_Get(adi_fpga9001_Device_t *device, adi_fpga9001_TddFraming_t *framing)
{
    uint8_t bfVal = 0;
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;

    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfGet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfGet, device, baseAddr, false);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddtriggerenableBfGet, device, baseAddr, &bfVal);
    framing->enableSyncExtTrig = (bool)bfVal;
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfGet, device, baseAddr, &framing->framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, baseAddr, &framing->numberFrames);
        
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTiming_Set_Validate(adi_fpga9001_Device_t *device,
                                               fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                               adi_fpga9001_TddTiming_t *tddTiming,
                                               uint32_t framePeriod)
{
    /* Counter starts at 1, so primaryAssert must be greater than 0 */
    if (tddTiming->primaryAssert == 0)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         tddTiming->primaryAssert,
                         "Invalid parameter value. tddTiming->primaryAssert must be greater than 0");
    }
    
    /* Assert must be strictly less than Deassert */
    if (tddTiming->primaryAssert >= tddTiming->primaryDeassert)
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_API, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         tddTiming->primaryAssert,
                         "Invalid parameter value. tddTiming->primaryAssert must be less than tddTiming->primaryDeassert");
    }
    
    /* Enable pin must deassert before the end of the TDD frame */
    if (tddTiming->primaryDeassert >= framePeriod)
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_API, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         tddTiming->primaryDeassert,
                         "Invalid parameter value. tddTiming->primaryDeassert must be less than framePeriod");
    }
    
    /* Secondary event is optional */
    if (tddTiming->secondaryAssert > 0)
    {
        /* Assert must be strictly less than Deassert */
        if (tddTiming->secondaryAssert >= tddTiming->secondaryDeassert)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             tddTiming->secondaryAssert,
                             "Invalid parameter value. tddTiming->secondaryAssert must be less than tddTiming->secondaryDeassert");
        }
        
        /* Enable pin must deassert before the end of the TDD frame */
        if (tddTiming->secondaryDeassert >= framePeriod)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             tddTiming->secondaryDeassert,
                             "Invalid parameter value. tddTiming->secondaryDeassert must be less than framePeriod");
        }
        
        /* Secondary event cannot overlap primary event */
        if (tddTiming->secondaryAssert <= tddTiming->primaryDeassert)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             tddTiming->secondaryAssert,
                             "Invalid parameter value. tddTiming->secondaryAssert must be greater than tddTiming->primaryDeassert");
        }
    }
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinTiming_Set(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryenableBfSet, device, baseAddr, true);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryassertBfSet, device, baseAddr, timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimarydeassertBfSet, device, baseAddr, timing->primaryDeassert);
    
    if (timing->secondaryAssert != 0 && timing->secondaryDeassert != 0)
    {
        ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryenableBfSet, device, baseAddr, true);
        ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryassertBfSet, device, baseAddr, timing->secondaryAssert);
        ADI_EXPECT(fpga9001_AxiTddEnablePinsecondarydeassertBfSet, device, baseAddr, timing->secondaryDeassert);
    }
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinTiming_Get(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryassertBfGet, device, baseAddr, &timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimarydeassertBfGet, device, baseAddr, &timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryassertBfGet, device, baseAddr, &timing->secondaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondarydeassertBfGet, device, baseAddr, &timing->secondaryDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDmaTiming_Set(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_AxiTddEnableDmaprimaryenableBfSet, device, baseAddr, true);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaprimaryassertBfSet, device, baseAddr, timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaprimarydeassertBfSet, device, baseAddr, timing->primaryDeassert);
    
    if (timing->secondaryAssert != 0 && timing->secondaryDeassert != 0)
    {
        ADI_EXPECT(fpga9001_AxiTddEnableDmasecondaryenableBfSet, device, baseAddr, true);
        ADI_EXPECT(fpga9001_AxiTddEnableDmasecondaryassertBfSet, device, baseAddr, timing->secondaryAssert);
        ADI_EXPECT(fpga9001_AxiTddEnableDmasecondarydeassertBfSet, device, baseAddr, timing->secondaryDeassert);
    }
      
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDmaTiming_Get(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_AxiTddEnableDmaprimaryassertBfGet, device, baseAddr, &timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaprimarydeassertBfGet, device, baseAddr, &timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnableDmasecondaryassertBfGet, device, baseAddr, &timing->secondaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnableDmasecondarydeassertBfGet, device, baseAddr, &timing->secondaryDeassert);
      
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddSequence_Set_Validate(adi_fpga9001_Device_t *device,
                                                 fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                                 adi_fpga9001_TddSequence_t *sequence,
                                                 uint32_t numberFrames)
{
    /* pauseFrames, activeFrames, and inactiveFrames only apply for a non-zero numberFrames */
    if (numberFrames > 0)
    {
        /* Pause must leave room for at least 1 frame to be active */
        if (sequence->pauseFrames >= (numberFrames - 1))
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             sequence->pauseFrames,
                             "Invalid parameter value. sequence->pauseFrames must be less than numberFrames");
        }
        
        /* At least 1 frame must be active */
        if (sequence->activeFrames == 0)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             sequence->activeFrames,
                             "Invalid parameter value. sequence->activeFrames must be greater than 0");
        }
        
        /* Pause + active frames cannot exceed numberFrames */
        if (sequence->pauseFrames + sequence->activeFrames > numberFrames)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             NULL,
                             "Invalid parameter values. sequence->pauseFrames + sequence->activeFrames must be less than numberFrames");
        }
        
        /* Ensure TDD operation ends after an integer number of active-inactive repetitions */
        if ((numberFrames - sequence->pauseFrames) % (sequence->activeFrames + sequence->inactiveFrames) != 0)
        {
            ADI_ERROR_REPORT(&device->common, 
                             ADI_COMMON_ERRSRC_API, 
                             ADI_COMMON_ERR_INV_PARAM, 
                             ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                             NULL,
                             "Invalid parameter values. (sequence->activeFrames + sequence->inactiveFrames) must be a multiple of (numberFrames - sequence->pauseFrames)");
        }
    }
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinSequence_Set(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinframespauseBfSet, device, baseAddr, sequence->pauseFrames);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesinactiveBfSet, device, baseAddr, sequence->inactiveFrames);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesactiveBfSet, device, baseAddr, sequence->activeFrames);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinSequence_Get(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinframespauseBfGet, device, baseAddr, &sequence->pauseFrames);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesinactiveBfGet, device, baseAddr, &sequence->inactiveFrames);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesactiveBfGet, device, baseAddr, &sequence->activeFrames);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDmaSequence_Set(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframespauseBfSet, device, baseAddr, sequence->pauseFrames);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframesinactiveBfSet, device, baseAddr, sequence->inactiveFrames);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframesactiveBfSet, device, baseAddr, sequence->activeFrames);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDmaSequence_Get(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframespauseBfGet, device, baseAddr, &sequence->pauseFrames);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframesinactiveBfGet, device, baseAddr, &sequence->inactiveFrames);
    ADI_EXPECT(fpga9001_AxiTddEnableDmaframesactiveBfGet, device, baseAddr, &sequence->activeFrames);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDatapathControl_Set(adi_fpga9001_Device_t *device,
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                               adi_fpga9001_TddDatapathControl_t *dpControl)
{
    ADI_EXPECT(fpga9001_TddDpCtrlEnableBitBfSet, device, baseAddr, dpControl->enable);
    ADI_EXPECT(fpga9001_TddDpCtrlStartValueBfSet, device, baseAddr, dpControl->start);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddDatapathControl_Get(adi_fpga9001_Device_t *device,
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                               adi_fpga9001_TddDatapathControl_t *dpControl)
{
    uint8_t bfVal = 0;
    ADI_EXPECT(fpga9001_TddDpCtrlEnableBitBfGet, device, baseAddr, &bfVal);
    dpControl->enable = (bool)bfVal;
    ADI_EXPECT(fpga9001_TddDpCtrlStartValueBfGet, device, baseAddr, &dpControl->start);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Configure_Validate(adi_fpga9001_Device_t *device,
                                                      fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                                      adi_fpga9001_TddChannel_t *tddChannel)
{
    uint32_t framePeriod = 0;
    uint32_t numberFrames = 0;
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfGet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, &framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, &numberFrames);
    
    ADI_EXPECT(fpga9001_TddTiming_Set_Validate, device, baseAddr, &tddChannel->pinTiming, framePeriod);
    ADI_EXPECT(fpga9001_TddSequence_Set_Validate, device, baseAddr, &tddChannel->pinSequence, numberFrames);

    /* Ignore DMA for GPIO baseAddr */
    if (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0 &&
        baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1)
    {
        ADI_EXPECT(fpga9001_TddTiming_Set_Validate, device, baseAddr, &tddChannel->dmaTiming, framePeriod);
        ADI_EXPECT(fpga9001_TddSequence_Set_Validate, device, baseAddr, &tddChannel->dmaSequence, numberFrames);    
    }
        
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Configure(adi_fpga9001_Device_t *device,
                                             fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                             adi_fpga9001_TddChannel_t *tddChannel)
{
    fpga9001_BfTddDpCtrlChanAddr_e dpCtrlAddr = FPGA9001_BF_RX_DP_CTRL_00;

    ADI_PERFORM_VALIDATION(fpga9001_TddChannel_Configure_Validate, device, baseAddr, tddChannel);
    
    dpCtrlAddr = fpga9001_TddDpCtrlChanAddr_Get(baseAddr);
    
    ADI_EXPECT(fpga9001_TddPinTiming_Set, device, baseAddr, &tddChannel->pinTiming);
    ADI_EXPECT(fpga9001_TddPinSequence_Set, device, baseAddr, &tddChannel->pinSequence);

    /* AUX (txGpioControls) has no associated DMA/datapath */
    if (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0 &&
        baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1)
    {
        ADI_EXPECT(fpga9001_TddDmaTiming_Set, device, baseAddr, &tddChannel->dmaTiming);
        ADI_EXPECT(fpga9001_TddDmaSequence_Set, device, baseAddr, &tddChannel->dmaSequence);

        ADI_EXPECT(fpga9001_TddDatapathControl_Set, device, dpCtrlAddr, &tddChannel->datapathControl);
    }
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Inspect(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddChannel_t *tddChannel)
{
    fpga9001_BfTddDpCtrlChanAddr_e dpCtrlAddr = fpga9001_TddDpCtrlChanAddr_Get(baseAddr);

    ADI_EXPECT(fpga9001_TddPinTiming_Get, device, baseAddr, &tddChannel->pinTiming);
    ADI_EXPECT(fpga9001_TddPinSequence_Get, device, baseAddr, &tddChannel->pinSequence);

    /* AUX (txGpioControls) has no associated DMA/datapath */
    if (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0 &&
        baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1)
    {
        ADI_EXPECT(fpga9001_TddDmaTiming_Get, device, baseAddr, &tddChannel->dmaTiming);
        ADI_EXPECT(fpga9001_TddDmaSequence_Get, device, baseAddr, &tddChannel->dmaSequence);

        ADI_EXPECT(fpga9001_TddDatapathControl_Get, device, dpCtrlAddr, &tddChannel->datapathControl);
    }
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Tdd_Configure_Validate(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, tddConfig);
    
    if (tddConfig->framing.framePeriod == 0)
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_API, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         tddConfig->framing.framePeriod,
                         "Invalid parameter value. tddConfig->framing.framePeriod must be greater than 0");
    }
    /* TODO: MMCM must be locked */
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_Configure(adi_fpga9001_Device_t *device, adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_Tdd_Configure_Validate, device, tddConfig);
    
    ADI_EXPECT(fpga9001_TddFraming_Set, device, &tddConfig->framing);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0, &tddConfig->rxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1, &tddConfig->rxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0, &tddConfig->orxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1, &tddConfig->orxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0, &tddConfig->txControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1, &tddConfig->txControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0, &tddConfig->txGpioControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1, &tddConfig->txGpioControls[1]);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_Inspect(adi_fpga9001_Device_t *device, adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, tddConfig);
    
    ADI_EXPECT(fpga9001_TddFraming_Get, device, &tddConfig->framing);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0, &tddConfig->rxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1, &tddConfig->rxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0, &tddConfig->orxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1, &tddConfig->orxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0, &tddConfig->txControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1, &tddConfig->txControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0, &tddConfig->txGpioControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1, &tddConfig->txGpioControls[1]);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ProgrammedEnable_Set(adi_fpga9001_Device_t *device, bool enable)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;
    uint32_t numberFrames = 0;
    
    ADI_API_ENTRY_EXPECT(device);
    /* TODO: MMCM must be locked */
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframecountersresetBfSet, device, baseAddr, true);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfSet, device, baseAddr, enable);
    
    /* Only set TddNumberFramesEnable if TddNumberFrames > 0*/
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, baseAddr, &numberFrames);
    if (numberFrames > 0)
    {
        ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfSet, device, baseAddr, enable);
    }
    
    /* FIXME: To disable, is it necessary to disable individual enable control modules and datapath control modules, as in Tokelau? */
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ProgrammedEnable_Get(adi_fpga9001_Device_t *device, bool *enable)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;
    uint8_t bfValue = 0;
    
    ADI_API_ENTRY_PTR_EXPECT(device, enable);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfGet, device, baseAddr, &bfValue);
    *enable = (bool)bfValue;
    
    ADI_API_RETURN(device);
}
