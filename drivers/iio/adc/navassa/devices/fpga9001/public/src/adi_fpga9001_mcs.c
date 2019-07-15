/**
* \file
* \brief FPGA9001 Multi-Chip Synchronization (MCS) functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/
#include "adi_adrv9001_user.h"
#include "adi_fpga9001_mcs.h"
#include "fpga9001_bf_axi_adrv9001.h"


static int32_t adi_fpga9001_Mcs_Start_Validate(adi_fpga9001_Device_t *device, uint8_t numberOfPulses, uint8_t mcsPeriod)
{
    /* TODO: Should max be 6 (most required by ADRV9001) or 15 (most FPGA can do) */
    ADI_RANGE_CHECK(device, numberOfPulses, 1, 15);
    /* TODO: mcsPeriod limits? */    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mcs_Start(adi_fpga9001_Device_t *device, uint8_t numberOfPulses, uint8_t mcsPeriod)
{
    uint8_t mcsRequest = 1;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_Mcs_Start_Validate, device, numberOfPulses, mcsPeriod);
    
    /* Wait for any previous MCS sequences to complete */
    do
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001McsRequestBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &mcsRequest);
    } while (0 != mcsRequest);
    
    /* Configure the sequence */
    ADI_EXPECT(fpga9001_AxiAdrv9001McsNumBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, numberOfPulses);
    ADI_EXPECT(fpga9001_AxiAdrv9001McsPeriodBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, mcsPeriod);
    
    /* Start the sequence */
    ADI_EXPECT(fpga9001_AxiAdrv9001McsRequestBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, true);
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Mcs_Requested_Validate(adi_fpga9001_Device_t *device, bool *requested)
{
    ADI_NULL_PTR_RETURN(&device->common, requested);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mcs_Requested(adi_fpga9001_Device_t *device, bool *requested)
{
    uint8_t mcsRequested = 0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_Mcs_Requested_Validate, device, requested);
    
    ADI_EXPECT(fpga9001_AxiAdrv9001GpintSbBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &mcsRequested);
    *requested = (bool)mcsRequested;
    
    /* Must write 1 to manually clear sticky bit */
    ADI_EXPECT(fpga9001_AxiAdrv9001GpintSbBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, true);
    
    ADI_API_RETURN(device);
}