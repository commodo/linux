/**
* \file
* \brief Private core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001ee01.h"

#include "adrv9001ee01.h"

#include "adi_adrv9001_rx.h"

#include "adi_fpga9001.h"
#include "adi_fpga9001_hal.h"

#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"

#include "fpga9001_bf_axi_adrv9001.h"

int32_t adi_adrv9001Ee01_SelectFpgaBin(adi_adrv9001Ee01_Board_t *adrv9001Ee01, adi_adrv9001_Init_t *adrv9001Init)
{
    if (adrv9001Init->rx.rxChannelCfg[0].profile.rxSsiConfig.ssiType == ADI_ADRV9001_SSI_TYPE_CMOS)
    {
        ADI_EXPECT(adi_fpga9001_SwitchBin, adrv9001Ee01->fpga9001Device, ADI_FPGA9001_CMOS);
    }
    else if (adrv9001Init->rx.rxChannelCfg[0].profile.rxSsiConfig.ssiType == ADI_ADRV9001_SSI_TYPE_LVDS)
    {
        ADI_EXPECT(adi_fpga9001_SwitchBin, adrv9001Ee01->fpga9001Device, ADI_FPGA9001_LVDS);
    }
    else
    {
        ADI_ERROR_REPORT(&adrv9001Ee01->fpga9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         adrv9001Init,
                         "Invalid SSI type. Must be LVDS or CMOS");
    }

    ADI_EXPECT(adi_adrv9001_HwClose, adrv9001Ee01->adrv9001Device);

    adi_adrv9001_SpiSettings_t spiSettings = {
        .msbFirst = 1,
        .enSpiStreaming = 0,
        .autoIncAddrUp = 1,
        .fourWireMode = 1,
        .cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
    };

    ADI_EXPECT(adi_adrv9001_HwOpen, adrv9001Ee01->adrv9001Device, &spiSettings);

    ADI_API_RETURN(adrv9001Ee01->fpga9001Device);
}

int32_t adi_fpga9001_Initialize(adi_fpga9001_Device_t *fpga9001Device,
                                adi_adrv9001_Init_t *init,
                                adi_fpga9001_SsiMode_e *ssiMode,
                                adi_fpga9001_MmcmCfg_t *desiredMmcmCfg,
                                adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockDivisor)
{
    uint8_t i = 0;
    adi_fpga9001_Version_t version = { 0 };

    static const adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };

    ADI_API_ENTRY_EXPECT(fpga9001Device);

    ADI_EXPECT(adi_fpga9001_VersionGet, fpga9001Device, &version);

    /* Checking only major and minor; ignoring patch version check */
    if (version.major == 0 && (version.minor == 3 || version.minor == 4))
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001ResetbBfSet, fpga9001Device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x1);

        /* Configure MMCM registers; THis function will reset the FPGA at the end */
        ADI_EXPECT(adi_fpga9001_Mmcm_Configure, fpga9001Device, desiredMmcmCfg, init->clocks.deviceClock_kHz, adrv9001DeviceClockDivisor);

        for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
        {
            if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.numLaneSel == ADI_ADRV9001_SSI_1_LANE)
            {
                switch (init->rx.rxChannelCfg[i].profile.rxSsiConfig.ssiDataFormatSel)
                {
                case ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_2I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_8I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_16I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_16I16Q;
                    break;
                default:
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.numLaneSel == ADI_ADRV9001_SSI_2_LANE)
            {
                if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.ssiType == ADI_ADRV9001_SSI_TYPE_LVDS)
                {
                    if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.ssiDataFormatSel == ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA)
                    {
                        *ssiMode = ADI_FPGA9001_LVDS_2LANE_16I16Q;
                    }
                    else
                    {
                        ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                        ADI_API_RETURN(fpga9001Device);
                    }
                }
                else
                {
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.numLaneSel == ADI_ADRV9001_SSI_4_LANE)
            {
                if (init->rx.rxChannelCfg[i].profile.rxSsiConfig.ssiDataFormatSel == ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA)
                {
                    *ssiMode = ADI_FPGA9001_CMOS_4LANE_16I16Q;
                }
                else
                {
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else
            {
                ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                ADI_API_RETURN(fpga9001Device);
            }

            ADI_EXPECT(adi_fpga9001_ssi_Mode_Set, fpga9001Device, ADI_RX, channels[i], *ssiMode);
        }

        for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
        {
            if (init->tx.txProfile[i].txSsiConfig.numLaneSel == ADI_ADRV9001_SSI_1_LANE)
            {
                switch (init->tx.txProfile[i].txSsiConfig.ssiDataFormatSel)
                {
                case ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_2I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_8I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_16I;
                    break;
                case ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA:
                    *ssiMode = ADI_FPGA9001_CMOS_1LANE_16I16Q;
                    break;
                default:
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else if (init->tx.txProfile[i].txSsiConfig.numLaneSel == ADI_ADRV9001_SSI_2_LANE)
            {
                if (init->tx.txProfile[i].txSsiConfig.ssiType == ADI_ADRV9001_SSI_TYPE_LVDS)
                {
                    if (init->tx.txProfile[i].txSsiConfig.ssiDataFormatSel == ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA)
                    {
                        *ssiMode = ADI_FPGA9001_LVDS_2LANE_16I16Q;
                    }
                    else
                    {
                        ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                        ADI_API_RETURN(fpga9001Device);
                    }
                }
                else
                {
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else if (init->tx.txProfile[i].txSsiConfig.numLaneSel == ADI_ADRV9001_SSI_4_LANE)
            {
                if (init->tx.txProfile[i].txSsiConfig.ssiDataFormatSel == ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA)
                {
                    *ssiMode = ADI_FPGA9001_CMOS_4LANE_16I16Q;
                }
                else
                {
                    ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                    ADI_API_RETURN(fpga9001Device);
                }
            }
            else
            {
                ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Profile not supported by FPGA");
                ADI_API_RETURN(fpga9001Device);
            }

            ADI_EXPECT(adi_fpga9001_ssi_Mode_Set, fpga9001Device, ADI_TX, channels[i], *ssiMode);
        }
    }
    else
    {
        ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Unrecognized FPGA version");
        ADI_API_RETURN(fpga9001Device);
    }

    ADI_API_RETURN(fpga9001Device);
}

int32_t adi_fpga9001_SsiChannelReset(adi_fpga9001_Device_t *fpga9001Device)
{
    static const uint32_t SSI_CONFIG_TX1_FPGA_BYTE1_ADDR = 0x4305600c;
    static const uint32_t SSI_CONFIG_TX2_FPGA_BYTE1_ADDR = 0x4305700c;
    static const uint32_t SSI_CONFIG_RX1_FPGA_BYTE1_ADDR = 0x4305400c;
    static const uint32_t SSI_CONFIG_RX2_FPGA_BYTE1_ADDR = 0x4305500c;
    static const uint16_t SSI_CONFIG_SERDES_RESET = 0x1;

    ADI_EXPECT(adi_fpga9001_RegisterWrite, fpga9001Device, SSI_CONFIG_TX1_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_RegisterWrite, fpga9001Device, SSI_CONFIG_TX2_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_RegisterWrite, fpga9001Device, SSI_CONFIG_RX1_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_RegisterWrite, fpga9001Device, SSI_CONFIG_RX2_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);

    ADI_API_RETURN(fpga9001Device);
}

adi_adrv9001_SsiMode_e adrv9001_SsiMode_Get(adi_fpga9001_SsiMode_e mode)
{
    switch (mode)
    {
    case ADI_FPGA9001_CMOS_1LANE_16I16Q:
        return ADI_ADRV9001_CMOS_1LANE_16I16Q;
    case ADI_FPGA9001_CMOS_1LANE_16I:
        return ADI_ADRV9001_CMOS_1LANE_16I;
    case ADI_FPGA9001_CMOS_1LANE_8I:
        return ADI_ADRV9001_CMOS_1LANE_8I;
    case ADI_FPGA9001_CMOS_1LANE_2I:
        return ADI_ADRV9001_CMOS_1LANE_2I;
    case ADI_FPGA9001_CMOS_4LANE_16I16Q:
        return ADI_ADRV9001_CMOS_4LANE_16I16Q;
    case ADI_FPGA9001_LVDS_2LANE_16I16Q:
        return ADI_ADRV9001_LVDS_2LANE_16I16Q;
    default:
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
}

int32_t adi_adrv9001_InitRxGainParameters(adi_adrv9001_Device_t *adrv9001Device,
                                          adi_adrv9001_Init_t *init,
                                          adi_adrv9001_ResourceCfg_t *adrv9001ResourceCfg)
{
    uint8_t gainIndex = 0;

    adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };

    uint32_t i = 0;

    if (adrv9001Device->devStateInfo.profilesValid & ADI_ADRV9001_RX_PROFILE_VALID)
    {
        for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
        {
            /* For each rx channel enabled */
            if (ADRV9001_BF_EQUAL(init->rx.rxInitChannelMask, channels[i]))
            {
                gainIndex = adrv9001ResourceCfg->adrv9001RadioConfig->radioCtrlInit.gainIndex[i];

                ADI_EXPECT(adi_adrv9001_Rx_GainCtrlMode_Set, adrv9001Device, channels[i], ADI_ADRV9001_MGC);

                ADI_EXPECT(adi_adrv9001_Rx_Gain_Set, adrv9001Device, channels[i], gainIndex);
            }
        }
    }

    ADI_API_RETURN(adrv9001Device);
}