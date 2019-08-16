/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
//#define DEBUG
//#define _DEBUG

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "adrv9002.h"

static adi_adrv9001_SpiSettings_t spiSettings = {
	.msbFirst = 1,
	.enSpiStreaming = 0,
	.autoIncAddrUp = 1,
	.fourWireMode = 1,
	.cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
};

adi_adrv9001_Init_t adrv9001Init_0 =
{
	.clocks =
	{
		.deviceClock_kHz = 40000,
		.clkPllVcoFreq_kHz = 8847360,
		.clkPllHsDiv = ADI_ADRV9001_HSDIV_4,
		.clkPllMode = ADI_ADRV9001_CLK_PLL_HP_MODE,
		.clk1105Div = ADI_ADRV9001_INTERNAL_CLOCK_DIV_2,
		.armClkDiv = ADI_ADRV9001_INTERNAL_CLOCK_DIV_6,
		.padRefClkDrv = 0,
		.extLo1OutFreq_kHz = 0,
		.extLo2OutFreq_kHz = 0,
		.rfPll1LoMode = ADI_ADRV9001_INT_LO1,
		.rfPll2LoMode = ADI_ADRV9001_INT_LO1,
		.ext1LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.ext2LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.rx1RfInputSel = ADI_ADRV9001_RX_A,
		.rx2RfInputSel = ADI_ADRV9001_RX_A,
		.extLo1Divider = 4,
		.extLo2Divider = 4,
		.rfPllPhaseSyncMode = ADI_ADRV9001_RFPLLMCS_NOSYNC,
		.rx1LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.rx2LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.tx1LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.tx2LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.rx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.rx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.loGen1Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO,
		.loGen2Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO
	},
	.rx =
	{
		.rxInitChannelMask = 255,
		.rxChannelCfg = {
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 20000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 40000,
				.rxCorner3dBLp_kHz = 40000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_RX1,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 1,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_A,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 20000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 40000,
				.rxCorner3dBLp_kHz = 40000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_RX2,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 1,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_C,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ORX1,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_B,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ORX2,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_D,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ILB1,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_B,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ILB2,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_D,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ELB1,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_B,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		},
		{
			.profile =
			{
				.primarySigBandwidth_Hz = 10000000,
				.rxOutputRate_Hz = 15360000,
				.rxInterfaceSampleRate_Hz = 15360000,
				.rxOffsetLo_kHz = 0,
				.rxSignalOnLo = 0,
				.outputSignaling = ADI_ADRV9001_RX_IQ,
				.filterOrder = 1,
				.filterOrderLp = 1,
				.hpAdcCorner = 50000000,
				.adcClk_kHz = 2211840,
				.rxCorner3dB_kHz = 100000,
				.rxCorner3dBLp_kHz = 100000,
				.tiaPower = 0,
				.tiaPowerLp = 0,
				.channelType = ADI_ADRV9001_ELB2,
				.adcType = ADI_ADRV9001_ADC_HP,
				.rxDpProfile =
				{
					.rxNbDecTop =
					{
						.scicBlk23En = 0,
						.scicBlk23DivFactor = 1,
						.scicBlk23LowRippleEn = 0,
						.decBy2Blk35En = 0,
						.decBy2Blk37En = 0,
						.decBy2Blk39En = 0,
						.decBy2Blk41En = 0,
						.decBy2Blk43En = 0,
						.decBy3Blk45En = 0,
						.decBy2Blk47En = 0
					},
					.rxWbDecTop =
					{
						.decBy2Blk25En = 0,
						.decBy2Blk27En = 0,
						.decBy2Blk29En = 0,
						.decBy2Blk31En = 1,
						.decBy2Blk33En = 1,
						.wbLpfBlk33p1En = 0
					},
					.rxDecTop =
					{
						.decBy3Blk15En = 1,
						.decBy2Hb3Blk17p1En = 0,
						.decBy2Hb4Blk17p2En = 0,
						.decBy2Hb5Blk19p1En = 0,
						.decBy2Hb6Blk19p2En = 0
					},
					.rxSincHBTop =
					{
						.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
						.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
						.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
						.isGainCompEnabled = 0,
						.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
						.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
					},
					.rxNbDem =
					{
						.dpInFifo =
						{
							.dpInFifoEn = 0,
							.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
							.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
						},
						.rxNbNco =
						{
							.rxNbNcoEn = 0,
							.rxNbNcoConfig =
							{
								.freq = 0,
								.sampleFreq = 0,
								.phase = 0,
								.realOut = 0
							}
						},
						.rxWbNbCompPFir =
						{
							.bankSel = ADI_ADRV9001_PFIR_BANK_D,
							.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
							.rxWbNbCompPFirEn = 1
						},
						.resamp =
						{
							.rxResampEn = 0,
							.resampPhaseI = 0,
							.resampPhaseQ = 0
						},
						.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
						.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
						.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
						.dpArmSel = ADI_ADRV9001_DP_SEL
					}
				},
				.rxSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.refClockGpioEn = false,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = true,
					.lvdsRxClkInversionEn = false,
					.rfLvdsDiv = 9,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false
				}
			}
		} }
	},
	.tx =
	{
		.txInitChannelMask = 3,
		.txProfile = {
		{
			.primarySigBandwidth_Hz = 10000000,
			.txInputRate_Hz = 15360000,
			.txInterfaceSampleRate_Hz = 15360000,
			.validDataDelay = 0,
			.txBbf3dBCorner_kHz = 50000,
			.outputSignaling = ADI_ADRV9001_TX_IQ,
			.txPdBiasCurrent = 1,
			.txPdGainEnable = 0,
			.txPrePdRealPole_kHz = 1000000,
			.txPostPdRealPole_kHz = 530000,
			.txBbfPowerMode = 7,
			.txExtLoopBackType = 0,
			.txExtLoopBackForInitCal = 0,
			.frequencyDeviation_Hz = 0,
			.txDpProfile =
			{
				.txPreProc =
				{
					.txPreProcSymbol0 = 0,
					.txPreProcSymbol1 = 0,
					.txPreProcSymbol2 = 0,
					.txPreProcSymbol3 = 0,
					.txPreProcSymMapDivFactor = 1,
					.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_MODE1,
					.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_A,
					.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_B
				},
				.txWbIntTop =
				{
					.txInterpBy2Blk30En = 0,
					.txInterpBy2Blk28En = 0,
					.txInterpBy2Blk26En = 0,
					.txInterpBy2Blk24En = 1,
					.txInterpBy2Blk22En = 1,
					.txWbLpfBlk22p1En = 0
				},
				.txNbIntTop =
				{
					.txInterpBy2Blk20En = 0,
					.txInterpBy2Blk18En = 0,
					.txInterpBy2Blk16En = 0,
					.txInterpBy2Blk14En = 0,
					.txInterpBy2Blk12En = 0,
					.txInterpBy3Blk10En = 0,
					.txInterpBy2Blk8En = 0,
					.txScicBlk32En = 0,
					.txScicBlk32DivFactor = 1
				},
				.txIntTop =
				{
					.interpBy3Blk44p1En = 1,
					.sinc3Blk44En = 0,
					.sinc2Blk42En = 0,
					.interpBy3Blk40En = 1,
					.interpBy2Blk38En = 0,
					.interpBy2Blk36En = 0
				},
				.txIntTopFreqDevMap =
				{
					.rrc2Frac = 0,
					.mpll = 0,
					.nchLsw = 0,
					.nchMsb = 0,
					.freqDevMapEn = 0,
					.txRoundEn = 1
				},
				.txIqdmDuc =
				{
					.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
					.iqdmDev = 0,
					.iqdmDevOffset = 0,
					.iqdmScalar = 0,
					.iqdmThreshold = 0,
					.iqdmNco =
					{
						.freq = 0,
						.sampleFreq = 0,
						.phase = 0,
						.realOut = 0
					}
				}
			},
			.txSsiConfig =
			{
				.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
				.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
				.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
				.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
				.lsbFirst = 0,
				.qFirst = 0,
				.refClockGpioEn = true,
				.lvdsBitInversion = 0,
				.lvdsUseLsbIn12bitMode = 0,
				.lvdsTxFullRefClkEn = false,
				.lvdsRxClkInversionEn = false,
				.rfLvdsDiv = 9,
				.cmosTxDdrNegStrobeEn = false,
				.cmosDdrPosClkEn = false,
				.cmosDdrClkInversionEn = false,
				.cmosDdrEn = false
			}
		},
		{
			.primarySigBandwidth_Hz = 10000000,
			.txInputRate_Hz = 15360000,
			.txInterfaceSampleRate_Hz = 15360000,
			.validDataDelay = 0,
			.txBbf3dBCorner_kHz = 50000,
			.outputSignaling = ADI_ADRV9001_TX_IQ,
			.txPdBiasCurrent = 1,
			.txPdGainEnable = 0,
			.txPrePdRealPole_kHz = 1000000,
			.txPostPdRealPole_kHz = 530000,
			.txBbfPowerMode = 7,
			.txExtLoopBackType = 0,
			.txExtLoopBackForInitCal = 0,
			.frequencyDeviation_Hz = 0,
			.txDpProfile =
			{
				.txPreProc =
				{
					.txPreProcSymbol0 = 0,
					.txPreProcSymbol1 = 0,
					.txPreProcSymbol2 = 0,
					.txPreProcSymbol3 = 0,
					.txPreProcSymMapDivFactor = 1,
					.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_MODE1,
					.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_C,
					.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_D
				},
				.txWbIntTop =
				{
					.txInterpBy2Blk30En = 0,
					.txInterpBy2Blk28En = 0,
					.txInterpBy2Blk26En = 0,
					.txInterpBy2Blk24En = 1,
					.txInterpBy2Blk22En = 1,
					.txWbLpfBlk22p1En = 0
				},
				.txNbIntTop =
				{
					.txInterpBy2Blk20En = 0,
					.txInterpBy2Blk18En = 0,
					.txInterpBy2Blk16En = 0,
					.txInterpBy2Blk14En = 0,
					.txInterpBy2Blk12En = 0,
					.txInterpBy3Blk10En = 0,
					.txInterpBy2Blk8En = 0,
					.txScicBlk32En = 0,
					.txScicBlk32DivFactor = 1
				},
				.txIntTop =
				{
					.interpBy3Blk44p1En = 1,
					.sinc3Blk44En = 0,
					.sinc2Blk42En = 0,
					.interpBy3Blk40En = 1,
					.interpBy2Blk38En = 0,
					.interpBy2Blk36En = 0
				},
				.txIntTopFreqDevMap =
				{
					.rrc2Frac = 0,
					.mpll = 0,
					.nchLsw = 0,
					.nchMsb = 0,
					.freqDevMapEn = 0,
					.txRoundEn = 1
				},
				.txIqdmDuc =
				{
					.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
					.iqdmDev = 0,
					.iqdmDevOffset = 0,
					.iqdmScalar = 0,
					.iqdmThreshold = 0,
					.iqdmNco =
					{
						.freq = 0,
						.sampleFreq = 0,
						.phase = 0,
						.realOut = 0
					}
				}
			},
			.txSsiConfig =
			{
				.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
				.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
				.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
				.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
				.lsbFirst = 0,
				.qFirst = 0,
				.refClockGpioEn = true,
				.lvdsBitInversion = 0,
				.lvdsUseLsbIn12bitMode = 0,
				.lvdsTxFullRefClkEn = false,
				.lvdsRxClkInversionEn = false,
				.rfLvdsDiv = 9,
				.cmosTxDdrNegStrobeEn = false,
				.cmosDdrPosClkEn = false,
				.cmosDdrClkInversionEn = false,
				.cmosDdrEn = false
			}
		} }
	},
	.sysConfig =
	{
		.duplexMode = ADI_ADRV9001_TDD_MODE,
		.fhModeOn = 0,
		.monitorModeOn = 0,
		.numDynamicProfile = 1,
		.extMcsOn = 0,
		.adcTypeMonitor = ADI_ADRV9001_ADC_HP,
		.pllModulus =
		{
			.modulus = { 8388593, 8388593, 8388593, 8388593, 8388593 },
			.dmModulus = { 8388593, 8388593 }
		}
	},
	.pfirBuffer =
	{
		.pfirRxWbNbChFilterCoeff_A =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 475, 312, -782, -39, 1201, -777, -1182, 1981, 177, -2874, 1941, 2393, -4416, 225, 5594, -4581, -3668, 8650, -1992, -9342, 9646, 4213, -15137, 6404, 13615, -18199, -2610, 23969, -15142, -17198, 31204, -3269, -34604, 30213, 17955, -49337, 16361, 45636, -53954, -12567, 72920, -40769, -54562, 89506, -4148, -102269, 83183, 57280, -142874, 41767, 139213, -158628, -45955, 231679, -125964, -193870, 320642, -4532, -442087, 390927, 347244, -1055854, 429729, 4391599, 4391599, 429729, -1055854, 347244, 390927, -442087, -4532, 320642, -193870, -125964, 231679, -45955, -158628, 139213, 41767, -142874, 57280, 83183, -102269, -4148, 89506, -54562, -40769, 72920, -12567, -53954, 45636, 16361, -49337, 17955, 30213, -34604, -3269, 31204, -17198, -15142, 23969, -2610, -18199, 13615, 6404, -15137, 4213, 9646, -9342, -1992, 8650, -3668, -4581, 5594, 225, -4416, 2393, 1941, -2874, 177, 1981, -1182, -777, 1201, -39, -782, 312, 0 }
		},
		.pfirRxWbNbChFilterCoeff_B =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxWbNbChFilterCoeff_C =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 475, 312, -782, -39, 1201, -777, -1182, 1981, 177, -2874, 1941, 2393, -4416, 225, 5594, -4581, -3668, 8650, -1992, -9342, 9646, 4213, -15137, 6404, 13615, -18199, -2610, 23969, -15142, -17198, 31204, -3269, -34604, 30213, 17955, -49337, 16361, 45636, -53954, -12567, 72920, -40769, -54562, 89506, -4148, -102269, 83183, 57280, -142874, 41767, 139213, -158628, -45955, 231679, -125964, -193870, 320642, -4532, -442087, 390927, 347244, -1055854, 429729, 4391599, 4391599, 429729, -1055854, 347244, 390927, -442087, -4532, 320642, -193870, -125964, 231679, -45955, -158628, 139213, 41767, -142874, 57280, 83183, -102269, -4148, 89506, -54562, -40769, 72920, -12567, -53954, 45636, 16361, -49337, 17955, 30213, -34604, -3269, 31204, -17198, -15142, 23969, -2610, -18199, 13615, 6404, -15137, 4213, 9646, -9342, -1992, 8650, -3668, -4581, 5594, 225, -4416, 2393, 1941, -2874, 177, 1981, -1182, -777, 1201, -39, -782, 312, 0 }
		},
		.pfirRxWbNbChFilterCoeff_D =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_A =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_B =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_C =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_D =
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxNbPulShp = {
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.taps = 128,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		{
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.taps = 128,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		} },
		.pfirRxMagLowTiaLowSRHp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -3, 14, -41, 101, -226, 550, -2320, 36621, -2320, 550, -226, 101, -41, 14, -3, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -3, 14, -41, 101, -226, 550, -2320, 36621, -2320, 550, -226, 101, -41, 14, -3, 0, 0, 0 }
		} },
		.pfirRxMagLowTiaHighSRHp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -286, 1454, -2530, -112, 7546, -13629, 2643, 42599, 2643, -13629, 7546, -112, -2530, 1454, -286, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -286, 1454, -2530, -112, 7546, -13629, 2643, 42599, 2643, -13629, 7546, -112, -2530, 1454, -286, 0, 0, 0 }
		} },
		.pfirRxMagHighTiaHighSRHp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, 4, -15, -8, 153, -598, 1822, -6763, 43576, -6763, 1822, -598, 153, -8, -15, 4, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, 4, -15, -8, 153, -598, 1822, -6763, 43576, -6763, 1822, -598, 153, -8, -15, 4, 0, 0, 0 }
		} },
		.pfirRxMagLowTiaLowSRLp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -3, 12, -39, 99, -235, 612, -2651, 37177, -2651, 612, -235, 99, -39, 12, -3, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -3, 12, -39, 99, -235, 612, -2651, 37177, -2651, 612, -235, 99, -39, 12, -3, 0, 0, 0 }
		} },
		.pfirRxMagLowTiaHighSRLp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -294, 1513, -2699, 37, 7994, -14896, 2372, 44716, 2372, -14896, 7994, 37, -2699, 1513, -294, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, -294, 1513, -2699, 37, 7994, -14896, 2372, 44716, 2372, -14896, 7994, 37, -2699, 1513, -294, 0, 0, 0 }
		} },
		.pfirRxMagHighTiaHighSRLp = {
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, 8, -30, 28, 119, -701, 2582, -10236, 49230, -10236, 2582, -701, 119, 28, -30, 8, 0, 0, 0 }
		},
		{
			.numCoeff = 21,
			.coefficients = { 0, 0, 0, 8, -30, 28, 119, -701, 2582, -10236, 49230, -10236, 2582, -701, 119, 28, -30, 8, 0, 0, 0 }
		} },
		.pfirTxMagComp1 =
		{
			.numCoeff = 21,
			.coefficients = { 48, -275, 811, -1487, 1508, 398, -4858, 10219, -9078, -1482, 41161, -1482, -9078, 10219, -4858, 398, 1508, -1487, 811, -275, 48 }
		},
		.pfirTxMagComp2 =
		{
			.numCoeff = 21,
			.coefficients = { 48, -275, 811, -1487, 1508, 398, -4858, 10219, -9078, -1482, 41161, -1482, -9078, 10219, -4858, 398, 1508, -1487, 811, -275, 48 }
		},
		.pfirTxMagCompNb = {
		{
			.numCoeff = 13,
			.coefficients = { 80, -381, 568, 598, -3813, 5518, 27630, 5518, -3813, 598, 568, -381, 80 }
		},
		{
			.numCoeff = 13,
			.coefficients = { 80, -381, 568, 598, -3813, 5518, 27630, 5518, -3813, 598, 568, -381, 80 }
		} },
		.pfirRxMagCompNb = {
		{
			.numCoeff = 13,
			.coefficients = { 80, -381, 568, 598, -3813, 5518, 27630, 5518, -3813, 598, 568, -381, 80 }
		},
		{
			.numCoeff = 13,
			.coefficients = { 80, -381, 568, 598, -3813, 5518, 27630, 5518, -3813, 598, 568, -381, 80 }
		} }
	}
};

adi_adrv9001_RadioConfig_t adrv9001RadioConfig_0 =
{
	.radioCtrlInit =
	{
		.adrv9001DeviceClockDivisor = ADI_ADRV9001_DEVICECLOCKDIVISOR_2,
		.armGpioSigMapSize = 1,
		.armGpioSigMap = {
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_TX1_ON,
			.gpioPinSel = ADI_ADRV9001_AGPIO_00,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		},
		{
			.gpioSignalSel = ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,
			.gpioPinSel = ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN,
			.polarity = ADI_ADRV9001_GPIO_POLARITY_NORMAL,
			.control = ADI_ADRV9001_GPIO_CONTROL_BBIC
		} },
		.slewRateLimiterCfg =
		{
			.srlEnable = false,
			.srlStatisticsEnable = false,
			.srlTableSelect = ADI_ADRV9001_SRL_TABLE0,
			.srlSlewOffset = 0,
			.srlStatisticsMode = ADI_ADRV9001_SRL_STATISTICS_MIN_SLEW_FACTOR_OBSERVED
		},
		.mbSetState = ADI_ADRV9001_MB_NOT_ALLOWED,
		.rxCarrierFreq_Hz = { 2400000000UL, 2400000000UL },
		.txCarrierFreq_Hz = { 2400000000UL, 2400000000UL },
		.gainIndex = { 0, 0 },
		.txAttenConfig =
		{
			.disableTxOnPllUnlock = false,
			.dacFullScaleBoostEnable = false,
			.txAttenStepSize = ADI_ADRV9001_TXATTEN_0P05_DB,
			.attenMode = ADI_ADRV9001_TXATTEN_SPI_ATTEN_MODE
		}
	},
	.initCals =
	{
		.sysInitCalMask = 0, /*ADI_ADRV9001_SYSTEM_ALL,*/
		.chanInitCalMask = { 705751 /* ADI_ADRV9001_RX_TX_ALL */, 705751 /*ADI_ADRV9001_RX_TX_ALL*/ },
		.calMode = ADI_ADRV9001_INITCAL_RUN_ALL_ALGO_MODE
	}
};

adi_adrv9001_PlatformFiles_t adrv9001PlatformFiles =
{
	.armImageFile = { "Navassa_EvaluationFw_0.4.4.4.bin" },
	.streamImageFile = { "fixme" },
	.rxGainTableFile = { "RxGainTable_0.2.0.csv" },
	.txAttenTableFile = { "TxAttenTable_0.1.0.csv" }
};

int adrv9002_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;
	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

	dev_err(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, buf[2], ret);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	return buf[2];
}

int adrv9002_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

	return 0;
}

static int adrv9002_phy_reg_access(struct iio_dev *indio_dev,
				   u32 reg, u32 writeval,
				   u32 *readval)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		ret = adrv9002_spi_write(phy->spi, reg, writeval);
	else {
		*readval = adrv9002_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

#define ADRV9002_MAX_CLK_NAME 79

static char *adrv9002_clk_set_dev_name(struct adrv9002_rf_phy *phy,
				       char *dest, const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strlcpy(dest, dev_name(&phy->spi->dev),
			      ADRV9002_MAX_CLK_NAME);
		else
			*dest = '\0';

		return strncat(dest, name, ADRV9002_MAX_CLK_NAME - len);
}

static unsigned long adrv9002_bb_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);
	return clk_priv->rate;
}

static int adrv9002_bb_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);
	clk_priv->rate = rate;

	return 0;
}

static long adrv9002_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);
	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = adrv9002_bb_round_rate,
	.set_rate = adrv9002_bb_set_rate,
	.recalc_rate = adrv9002_bb_recalc_rate,
};

static int adrv9002_clk_register(struct adrv9002_rf_phy *phy,
				 const char *name, const char *parent_name,
				 const char *parent_name2, unsigned long flags,
				 u32 source)
{
	struct adrv9002_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV9002_MAX_CLK_NAME + 1], p_name[2][ADRV9002_MAX_CLK_NAME + 1];
	const char *_parent_name[2];

	/* struct adrv9002_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] = adrv9002_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] = adrv9002_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = adrv9002_clk_set_dev_name(phy, c_name, name);;
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
		case RX1_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = phy->curr_profile->rx.rxChannelCfg[0].profile.rxOutputRate_Hz; /* TODO: check indexing */
			break;
		case RX2_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = phy->curr_profile->rx.rxChannelCfg[1].profile.rxOutputRate_Hz;
			break;
		case TX1_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = phy->curr_profile->tx.txProfile[0].txInputRate_Hz;
			break;
		case TX2_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = phy->curr_profile->tx.txProfile[1].txInputRate_Hz;
			break;
		default:
			return -EINVAL;
	}

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

int adrv9002_dev_err(struct adrv9002_rf_phy *phy, const char* function)
{
	int ret;
	dev_err(&phy->spi->dev, "%s failed with %s (%d)\n", function,
		phy->adrv9001->common.error.errormessage ?
		phy->adrv9001->common.error.errormessage : "",
		phy->adrv9001->common.error.errCode);


	switch (phy->adrv9001->common.error.errCode) {
		case ADI_COMMON_ERR_INV_PARAM:
		case ADI_COMMON_ERR_NULL_PARAM:
			ret = -EINVAL;
		case ADI_COMMON_ERR_API_FAIL:
			ret = -EFAULT;
		case ADI_COMMON_ERR_SPI_FAIL:
			ret = -EIO;
		case ADI_COMMON_ERR_MEM_ALLOC_FAIL:
			ret = -ENOMEM;
		case ADI_COMMON_ERR_OK:
			ret = 0;
		default:
			ret = -EFAULT;
	}

	adi_common_ErrorClear(&phy->adrv9001->common);

	return ret;
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static ssize_t adrv9002_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RadioState_t radioState;
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel;
	u64 readin;
	int ret = 0;

	port = ADRV_ADDRESS_PORT(chan->address);
	channel = ADRV_ADDRESS_CHAN(chan->address);

	switch (private) {
	case LOEXT_FREQ:

		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		mutex_lock(&indio_dev->mlock);

		ret = adi_adrv9001_Radio_State_Get(phy->adrv9001, &radioState);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
			break;
		}

		ret = adi_adrv9001_Radio_Channel_ToCalibrated(phy->adrv9001,
					port, channel);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
			break;
		}

		ret = adi_adrv9001_Radio_CarrierFrequency_Set(phy->adrv9001,
					port, channel,
					ADI_ADRV9001_PLL_CAL_MODE_NORM,
					ADI_ADRV9001_MB_NOT_ALLOWED,
					readin);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
			break;
		}
		ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001,
					port, channel,
					radioState.channelStates[port][channel]);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
		}
		break;
	default:
		ret = -EINVAL;
		break;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t adrv9002_phy_lo_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel;
	u64 val;
	int ret;

	port = ADRV_ADDRESS_PORT(chan->address);
	channel = ADRV_ADDRESS_CHAN(chan->address);

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		ret = adi_adrv9001_Radio_CarrierFrequency_Get(phy->adrv9001,
					port,channel, &val);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
		}

		break;
	default:
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : sprintf(buf, "%llu\n", val);
}

#define _ADRV9002_EXT_LO_INFO(_name, _ident) { \
.name = _name, \
.read = adrv9002_phy_lo_read, \
.write = adrv9002_phy_lo_write, \
.private = _ident, \
}

static const struct iio_chan_spec_ext_info adrv9002_phy_ext_lo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV9002_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{ },
};

static int adrv9002_set_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RxGainCtrlMode_e val;
	int ret;

	switch (mode) {
	case 0:
		val = ADI_ADRV9001_MGC;
		break;
	case 1:
		val = ADI_ADRV9001_AGC;
		break;
	default:
		return -EINVAL;
	}

	ret = adi_adrv9001_Rx_GainCtrlMode_Set(phy->adrv9001,
					       ADRV_ADDRESS_CHAN(chan->address),
					       val);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
	}

	return ret;
}

static int adrv9002_get_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RxGainCtrlMode_e gainCtrlMode;

	adi_adrv9001_Rx_GainCtrlMode_Get(phy->adrv9001,
		       ADRV_ADDRESS_CHAN(chan->address), &gainCtrlMode);

	return (gainCtrlMode == ADI_ADRV9001_MGC) ? 0 : 1;
}

static const char * const adrv9002_agc_modes[] =
	{"manual", "automatic"};

static const struct iio_enum adrv9002_agc_modes_available = {
	.items = adrv9002_agc_modes,
	.num_items = ARRAY_SIZE(adrv9002_agc_modes),
	.get = adrv9002_get_agc_mode,
	.set = adrv9002_set_agc_mode,

};


static int adrv9002_set_ensm_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001,
						 ADRV_ADDRESS_PORT(chan->address),
						 ADRV_ADDRESS_CHAN(chan->address),
						 mode + 1);
	if (ret)
		ret = adrv9002_dev_err(phy, __func__);

	return ret;
}

static int adrv9002_get_ensm_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RadioState_t radioState;
	int ret;

	ret = adi_adrv9001_Radio_State_Get(phy->adrv9001, &radioState);
	if (ret)
		return adrv9002_dev_err(phy, __func__);

	return radioState.channelStates[ADRV_ADDRESS_PORT(chan->address)][ADRV_ADDRESS_CHAN(chan->address)] - 1;
}

static const char * const adrv9002_ensm_modes[] =
	{"calibrated", "primed", "rf_enabled"};

static const struct iio_enum adrv9002_ensm_modes_available = {
	.items = adrv9002_ensm_modes,
	.num_items = ARRAY_SIZE(adrv9002_ensm_modes),
	.get = adrv9002_get_ensm_mode,
	.set = adrv9002_set_ensm_mode,
};

static ssize_t adrv9002_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
//	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel;
	bool enable;
	int ret = 0;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	port = ADRV_ADDRESS_PORT(chan->address);
	channel = ADRV_ADDRESS_CHAN(chan->address);

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case RSSI:
		break;
	case RX_GAIN_CTRL_PIN_MODE:

		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}



static ssize_t adrv9002_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u16 dec_pwr_mdb;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case RSSI:
		ret = adi_adrv9001_Rx_DecimatedPower_Get(phy->adrv9001,
							 ADRV_ADDRESS_CHAN(chan->address),
					   &dec_pwr_mdb);
		if (ret)
			ret = adrv9002_dev_err(phy, __func__);

		if (ret == 0)
			ret = sprintf(buf, "%u.%02u dB\n",
				      dec_pwr_mdb / 1000, dec_pwr_mdb % 1000);
		break;

	case RX_RF_BANDWIDTH:
		ret = phy->curr_profile->rx.rxChannelCfg[chan->channel].profile.primarySigBandwidth_Hz;
		if (ret > 0)
			ret = sprintf(buf, "%u\n", ret);
		break;
	case RX_GAIN_CTRL_PIN_MODE:
		break;
	default:
		ret = -EINVAL;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

#define _ADRV9002_EXT_RX_INFO(_name, _ident) { \
.name = _name, \
.read = adrv9002_phy_rx_read, \
.write = adrv9002_phy_rx_write, \
.private = _ident, \
}


static ssize_t adrv9002_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel;
	adi_adrv9001_TxAttenMode_e mode;
	int val = 0, ret = 0;

	port = ADRV_ADDRESS_PORT(chan->address);
	channel = ADRV_ADDRESS_CHAN(chan->address);

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case TX_RF_BANDWIDTH:
		val = phy->curr_profile->tx.txProfile[chan->channel].primarySigBandwidth_Hz;
		break;
	case TX_ATTN_CTRL_PIN_MODE:

		ret = adi_adrv9001_Tx_AttenuationMode_Get(phy->adrv9001,
							    channel,
							    &mode);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
			break;
		}
		val = (mode == ADI_ADRV9001_TXATTEN_GPIO_MODE);
		break;
	case TX_PA_PROTECTION:
		break;

	default:
		ret = -EINVAL;

	}

	if (ret == 0)
		ret = sprintf(buf, "%d\n", val);

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t adrv9002_phy_tx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel;
	port = ADRV_ADDRESS_PORT(chan->address);
	channel = ADRV_ADDRESS_CHAN(chan->address);
	bool enable;
	int ret = 0;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case TX_ATTN_CTRL_PIN_MODE:
		ret =  adi_adrv9001_Tx_AttenuationMode_Set(phy->adrv9001,
				channel,
				enable ? ADI_ADRV9001_TXATTEN_GPIO_MODE : ADI_ADRV9001_TXATTEN_SPI_ATTEN_MODE);
		if (ret)
			ret = adrv9002_dev_err(phy, __func__);

		break;
	case TX_PA_PROTECTION:
		break;

	default:
		ret = -EINVAL;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

#define _ADRV9002_EXT_TX_INFO(_name, _ident) { \
.name = _name, \
.read = adrv9002_phy_tx_read, \
.write = adrv9002_phy_tx_write, \
.private = _ident, \
}

static const struct iio_chan_spec_ext_info adrv9002_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE_SHARED("ensm_mode", 0,  &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", false, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("gain_control_mode", 0,  &adrv9002_agc_modes_available),
	IIO_ENUM("gain_control_mode", false, &adrv9002_agc_modes_available),
	_ADRV9002_EXT_RX_INFO("rssi", RSSI),
	_ADRV9002_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV9002_EXT_RX_INFO("gain_control_pin_mode_en", RX_GAIN_CTRL_PIN_MODE),
	{ },
};

static struct iio_chan_spec_ext_info adrv9002_phy_tx_ext_info[] = {
	IIO_ENUM_AVAILABLE_SHARED("ensm_mode", 0,  &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", false, &adrv9002_ensm_modes_available),
	_ADRV9002_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	_ADRV9002_EXT_TX_INFO("atten_control_pin_mode_en", TX_ATTN_CTRL_PIN_MODE),
//	_ADRV9002_EXT_TX_INFO("pa_protection_en", TX_PA_PROTECTION),
	{ },
};

static int adrv9002_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val,
				 int *val2,
				 long m)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_ChannelNumber_e channel;
	u16 temp;
	int ret;


	channel = ADRV_ADDRESS_CHAN(chan->address);

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			u16 atten_mdb;

			ret = adi_adrv9001_Tx_Attenuation_Get(phy->adrv9001,
							      channel,
							      &atten_mdb);
			if (ret) {
				ret = adrv9002_dev_err(phy, __func__);
				break;
			}

			*val = -1 * (atten_mdb / 1000);
			*val2 = (atten_mdb % 1000) * 1000;
			if (!*val)
				*val2 *= -1;
			ret = IIO_VAL_INT_PLUS_MICRO_DB;

		} else {
			adi_adrv9001_RxGainCtrlMode_e gainCtrlMode;
			u8 index;

			ret = adi_adrv9001_Rx_GainCtrlMode_Get(phy->adrv9001,
							       channel, &gainCtrlMode);
			if (ret) {
				ret = adrv9002_dev_err(phy, __func__);
				break;
			}

			switch (gainCtrlMode) {
			case ADI_ADRV9001_MGC:
				ret = adi_adrv9001_Rx_MgcGain_Get(phy->adrv9001, channel, &index);
				break;
			case ADI_ADRV9001_AGC:
				ret = adi_adrv9001_Rx_Gain_Get(phy->adrv9001, channel, &index);
				break;
			default:
				ret = -EINVAL;
			}

			if (ret) {
				ret = adrv9002_dev_err(phy, __func__);
				break;
			}

// 			ret = adrv9002_gainindex_to_gain(phy, chan->channel,
// 							 index, val, val2);

			*val = index;
			ret = IIO_VAL_INT;
		}

		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output) {
			switch (channel) {
			case ADI_CHANNEL_1:
				*val = clk_get_rate(phy->clks[TX1_SAMPL_CLK]);
				break;
			case ADI_CHANNEL_2:
				*val = clk_get_rate(phy->clks[TX2_SAMPL_CLK]);
				break;
			}
		} else {
			switch (channel) {
			case ADI_CHANNEL_1:
				*val = clk_get_rate(phy->clks[RX1_SAMPL_CLK]);
				break;
			case ADI_CHANNEL_2:
				*val = clk_get_rate(phy->clks[RX2_SAMPL_CLK]);
				break;
			}
		}

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		ret = adi_adrv9001_Temperature_Get(phy->adrv9001, &temp);
		if (ret) {
			ret = adrv9002_dev_err(phy, __func__);
			break;
		}
		*val = temp * 1000;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int adrv9002_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int val,
				  int val2,
				  long mask)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_ChannelNumber_e channel;
	u32 code;
	int ret = 0;

	channel = ADRV_ADDRESS_CHAN(chan->address);

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			ret = adi_adrv9001_Tx_Attenuation_Set(phy->adrv9001, channel, code);
		} else {
// 			ret = adrv9002_gain_to_gainindex(phy, chan->channel,
// 							 val, val2, &code);
// 			if (ret < 0)
// 				break;

			ret = adi_adrv9001_Rx_Gain_Set(phy->adrv9001, channel, val);
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		break;

	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&indio_dev->mlock);

	if (ret)
		ret = adrv9002_dev_err(phy, __func__);

	return ret;
}

static const struct iio_chan_spec adrv9002_phy_chan[] = {
	{	/* RX1 LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "RX1_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
		.address = ADRV_ADDRESS(ADI_RX, ADI_CHANNEL_1),
	}, {	/* RX2_LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "RX2_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
		.address = ADRV_ADDRESS(ADI_RX, ADI_CHANNEL_2),
	},{	/* TX1 LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 2,
		.extend_name = "TX1_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
		.address = ADRV_ADDRESS(ADI_TX, ADI_CHANNEL_1),
	}, {	/* TX2_LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 3,
		.extend_name = "TX2_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
		.address = ADRV_ADDRESS(ADI_TX, ADI_CHANNEL_2),
	}, {	/* TX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
		//.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9002_phy_tx_ext_info,
		.address = ADRV_ADDRESS(ADI_TX, ADI_CHANNEL_1),
	}, {	/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9002_phy_rx_ext_info,
		.address = ADRV_ADDRESS(ADI_RX, ADI_CHANNEL_1),
	}, {	/* TX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
		//.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9002_phy_tx_ext_info,
		.address = ADRV_ADDRESS(ADI_TX, ADI_CHANNEL_2),

	}, {	/* RX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9002_phy_rx_ext_info,
		.address = ADRV_ADDRESS(ADI_RX, ADI_CHANNEL_2),
	}, {
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info adrv9002_phy_info = {
	.read_raw = &adrv9002_phy_read_raw,
	.write_raw = &adrv9002_phy_write_raw,
	.debugfs_reg_access = &adrv9002_phy_reg_access,
// 	.attrs = &adrv9002_phy_attribute_group,
	.driver_module = THIS_MODULE,
};

adi_adrv9001_Device_t *adrv9001Device_glob;

int count = 0;

#if 0
int32_t adi_adrv9001Ee01_Mcs_Requested(void)
{
	adi_hal_Cfg_t *hal = adrv9001Device_glob->common.devHalInfo;
	struct spi_device *spi = hal->spi;

	pr_err("%s: GPIO %d\n", __func__, gpiod_get_value(hal->int_gpio));

	//ADI_EXPECT(adi_adrv9001_Mcs_Digital_Reset, adrv9001Device_glob);
	//ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device_glob, 2);

	return ADI_COMMON_ACT_NO_ACTION	;
}
#endif

static void ssiConfigSettingsInitDefault(adi_adrv9001_SsiConfigSettings_t *ssiConfig)
{
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxStrobeDelay[0] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxStrobeDelay[1] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxIDataDelay[0] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxIDataDelay[1] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxQDataDelay[0] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsRxQDataDelay[1] = 0x0;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsTxClkDelay[0] = 0x3;
	ssiConfig->ssiCalibrationCfg.lvdsSsiCalibrationCfg.lvdsTxClkDelay[1] = 0x3;
}

#define ADRV9001_BF_EQUAL(value, mask) ((mask) == ((value) & (mask)))


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

static int adrv9002_setup(struct adrv9002_rf_phy *phy, adi_adrv9001_Init_t *adrv9001Init,  adi_adrv9001_RadioConfig_t *adrv9001RadioConfig)
{
	adi_adrv9001_Device_t *adrv9001Device = phy->adrv9001;
	adi_adrv9001_SsiConfigSettings_t ssiConfig = { 0 };

	adi_adrv9001_ResourceCfg_t adrv9001ResourceCfg = { adrv9001Init, adrv9001RadioConfig, &adrv9001PlatformFiles };
	uint8_t initCalsError = 0;
	uint8_t channelMask = 0;
	static const uint8_t ALL_RX_CHANNEL_MASK = 0xF;
	/* TX_CHANNEL_MASK_OFFSET is '4' as channel mask info for RX1/2 and ORX1/2 occupy the least 4 bits */
	static const uint8_t TX_CHANNEL_MASK_OFFSET = 4;
	int ret;

	adrv9001_RxInterfaceGainCtrl_t rxInterfaceGainConfig_11 =
	{
		.updateInstance = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NOW,
		.gainControlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL,
		.gain = ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB,
		.signalPar_dB = 0
	};

	phy->curr_profile = adrv9001Init;

	channelMask = (adrv9001Init->tx.txInitChannelMask << TX_CHANNEL_MASK_OFFSET) | (adrv9001Init->rx.rxInitChannelMask & ALL_RX_CHANNEL_MASK);

	adi_common_ErrorClear(&phy->adrv9001->common);
	adi_adrv9001_HwOpen(adrv9001Device, &spiSettings);
	adi_common_LogLevelSet(&adrv9001Device->common, ADI_HAL_LOG_ERR);

	ret = adi_adrv9001_InitAnalog(adrv9001Device, adrv9001Init, adrv9001RadioConfig->radioCtrlInit.adrv9001DeviceClockDivisor);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_InitDigitalLoad(adrv9001Device, &adrv9001ResourceCfg);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ssiConfigSettingsInitDefault(&ssiConfig);
	ssiConfig.ssiMode = ADI_ADRV9001_LVDS_2LANE_16I16Q;

	ret = adi_adrv9001_InitRadioLoad(adrv9001Device, NULL, &adrv9001ResourceCfg, &ssiConfig, channelMask);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_InitCalsRun(adrv9001Device, NULL, &adrv9001RadioConfig->initCals, 60000, &initCalsError);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_InitRxGainParameters(adrv9001Device, adrv9001Init, &adrv9001ResourceCfg);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_RX, ADI_CHANNEL_1, ADI_ADRV9001_SPI_MODE);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_TX, ADI_CHANNEL_1, ADI_ADRV9001_SPI_MODE);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_RX, ADI_CHANNEL_1);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_TX, ADI_CHANNEL_1);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_RX, ADI_CHANNEL_2, ADI_ADRV9001_SPI_MODE);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_TX, ADI_CHANNEL_2, ADI_ADRV9001_SPI_MODE);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_RX, ADI_CHANNEL_2);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_TX, ADI_CHANNEL_2);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Rx_GainCtrlMode_Set(adrv9001Device, ADI_CHANNEL_2, ADI_ADRV9001_MGC);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Rx_MinMaxGainIndex_Set(adrv9001Device, 1, 183, 255);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Configure(adrv9001Device, ADI_CHANNEL_1, &rxInterfaceGainConfig_11);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Configure(adrv9001Device, ADI_CHANNEL_2, &rxInterfaceGainConfig_11);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_RX, ADI_CHANNEL_1);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_RX, ADI_CHANNEL_2);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_TX, ADI_CHANNEL_1);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	ret = adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_TX, ADI_CHANNEL_2);
	if (ret) {
		ret = adrv9002_dev_err(phy, __func__);
		goto out;
	}

	clk_set_rate(phy->clks[RX1_SAMPL_CLK], phy->curr_profile->rx.rxChannelCfg[0].profile.rxOutputRate_Hz);
	clk_set_rate(phy->clks[RX2_SAMPL_CLK], phy->curr_profile->rx.rxChannelCfg[1].profile.rxOutputRate_Hz);
	clk_set_rate(phy->clks[TX1_SAMPL_CLK], phy->curr_profile->tx.txProfile[0].txInputRate_Hz);
	clk_set_rate(phy->clks[TX2_SAMPL_CLK], phy->curr_profile->tx.txProfile[1].txInputRate_Hz);

out:
	return ret;

}

static ssize_t
adrv9002_profile_bin_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	if (off == 0) {
		if (phy->bin_attr_buf == NULL) {
			phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev,
							 bin_attr->size, GFP_KERNEL);
			if (!phy->bin_attr_buf)
				return -ENOMEM;
		} else
			memset(phy->bin_attr_buf, 0, bin_attr->size);
	}

	memcpy(phy->bin_attr_buf + off, buf, count);

	if (strnstr(phy->bin_attr_buf, "\n}", off + count) == NULL)
		return count;

	dev_dbg(&phy->spi->dev, "%s:%d: size %lld\n", __func__, __LINE__, off + count);

	mutex_lock(&phy->indio_dev->mlock);

	ret = adi_adrv9001_Utilities_DeviceProfile_Parse(phy->adrv9001,
							 &phy->profile,
							 phy->bin_attr_buf,
							 off + count);
	if (ret)
		goto out;

	adi_adrv9001_Shutdown(phy->adrv9001);

	ret = adrv9002_setup(phy, &phy->profile, &adrv9001RadioConfig_0);

out:
	mutex_unlock(&phy->indio_dev->mlock);

	return (ret < 0) ? ret : count;
}

static ssize_t
adrv9002_profile_bin_read(struct file *filp, struct kobject *kobj,
			  struct bin_attribute *bin_attr,
			  char *buf, loff_t off, size_t count)
{
	if (off)
		return 0;

	return sprintf(buf, "TBD");
}

static int adrv9002_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9002_rf_phy *phy;
	struct clk *clk = NULL;
	adi_common_ApiVersion_t apiVersion;
	adi_adrv9001_ArmVersion_t armVersion;
	int ret;
	u8 rev;

	int id  = spi_get_device_id(spi)->driver_data;

	dev_info(&spi->dev, "%s : enter", __func__);

	clk = devm_clk_get(&spi->dev, "adrv9002_ext_refclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->spi_device_id = id;

	phy->adrv9001 = &phy->adrv9001_device;
	phy->hal.spi = spi;

	phy->hal.logCfg.logLevel = ADI_HAL_LOG_ALL;
	phy->adrv9001->common.devHalInfo = &phy->hal;

	phy->hal.reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	phy->hal.int_gpio = devm_gpiod_get(&spi->dev, "int", GPIOD_IN);

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	ret = adrv9002_setup(phy, &adrv9001Init_0, &adrv9001RadioConfig_0);
	if (ret < 0)
		return ret;

	adrv9002_clk_register(phy, "-rx1_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		       RX1_SAMPL_CLK);
	adrv9002_clk_register(phy, "-rx2_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		       RX2_SAMPL_CLK);

	adrv9002_clk_register(phy, "-tx1_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		       TX1_SAMPL_CLK);

	adrv9002_clk_register(phy, "-tx2_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		       TX2_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_ADRV9002_CLKS;

	ret = of_clk_add_provider(spi->dev.of_node,
				  of_clk_src_onecell_get, &phy->clk_data);
	if (ret)
		goto out_disable_clocks;

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = "adrv9002-phy";

	indio_dev->modes = INDIO_DIRECT_MODE;

	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "profile_config";
	phy->bin.attr.mode = S_IWUSR | S_IRUGO;
	phy->bin.write = adrv9002_profile_bin_write;
	phy->bin.read = adrv9002_profile_bin_read;
	phy->bin.size = 73728;

	switch (id) {
	case ID_ADRV9002:
		indio_dev->info = &adrv9002_phy_info;
		indio_dev->channels = adrv9002_phy_chan;
		indio_dev->num_channels = ARRAY_SIZE(adrv9002_phy_chan);
		break;
	default:
		ret = -EINVAL;
		goto out_clk_del_provider;
	}
	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_clk_del_provider;

	ret = adrv9002_register_axi_converter(phy);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		goto out_iio_device_unregister;

// 	if (spi->irq) {
// 		ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
// 						adrv9002_irq_handler,
// 						IRQF_TRIGGER_RISING  | IRQF_ONESHOT,
// 						indio_dev->name, indio_dev);
//
// 		if (ret) {
// 			dev_err(&spi->dev,
// 				"request_irq() failed with %d\n", ret);
// 			goto out_remove_sysfs_bin;
// 		}
// 	}
//

	adi_adrv9001_ApiVersionGet(phy->adrv9001, &apiVersion);
	adi_adrv9001_arm_Version(phy->adrv9001, &armVersion);
	adi_adrv9001_DeviceRevGet(phy->adrv9001, &rev);

	dev_info(&spi->dev,
		 "%s: %s Rev %d, Firmware %u.%u.%u.%u API version: %u.%u.%u.%u successfully initialized",
	  	__func__, spi_get_device_id(spi)->name, rev, armVersion.majorVer, armVersion.minorVer,
		 armVersion.maintVer,armVersion.rcVer, apiVersion.major,
	  	 apiVersion.minor, apiVersion.patch, apiVersion.build);

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);
out_clk_del_provider:
	of_clk_del_provider(spi->dev.of_node);
out_disable_clocks:
	clk_disable_unprepare(phy->dev_clk);

	return ret;
}

static int adrv9002_remove(struct spi_device *spi)
{
	struct adrv9002_rf_phy *phy = adrv9002_spi_to_phy(spi);

	iio_device_unregister(phy->indio_dev);

	return 0;
}

static const struct spi_device_id adrv9002_id[] = {
	{"adrv9002", ID_ADRV9002},
	{}
};
MODULE_DEVICE_TABLE(spi, adrv9002_id);

static struct spi_driver adrv9002_driver = {
	.driver = {
		.name	= "adrv9002",
		.owner	= THIS_MODULE,
	},
	.probe		= adrv9002_probe,
	.remove		= adrv9002_remove,
	.id_table	= adrv9002_id,
};
module_spi_driver(adrv9002_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9002 ADC");
MODULE_LICENSE("GPL v2");
