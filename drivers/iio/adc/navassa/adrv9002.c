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
			clk_priv->rate = 15360000;//phy->talInit.rx.rxProfile.rxOutputRate_kHz;
			break;
		case RX2_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = 15360000;//phy->talInit.rx.rxProfile.rxOutputRate_kHz;
			break;
		case TX1_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = 15360000;//phy->talInit.tx.txProfile.txInputRate_kHz;
			break;
		case TX2_SAMPL_CLK:
			init.ops = &bb_clk_ops;
			clk_priv->rate = 15360000;//phy->talInit.tx.txProfile.txInputRate_kHz;
			break;
		default:
			return -EINVAL;
	}

	//clk_priv->rate *= 1000;

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

enum lo_ext_info {
	LOEXT_FREQ,
	FHM_ENABLE,
	FHM_HOP,
};

static ssize_t adrv9002_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	u64 readin;
	u8 status;
	u16 loop_bw;
	bool enable;
	int ret = 0;

	switch (private) {
		case LOEXT_FREQ:

			ret = kstrtoull(buf, 10, &readin);
			if (ret)
				return ret;

// 			mutex_lock(&indio_dev->mlock);
//
// 			ret = adi_adrv9001_Radio_CarrierFrequency_Set(adi_adrv9001_Device_t *device,
// 									adi_common_Port_e port,
// 						   adi_common_ChannelNumber_e channel,
// 						 ADI_ADRV9001_PLL_CAL_MODE_NORM,
// 						 ADI_ADRV9001_MB_NOT_ALLOWED,
// 						   readin);
//
// 			adrv9002_set_radio_state(phy, RADIO_FORCE_OFF);
//
// 			if (readin >= 3000000000ULL)
// 				loop_bw = 300;
// 			else
// 				loop_bw = 50;
//
// 			if (loop_bw != phy->current_loopBandwidth_kHz[chan->channel]) {
// 				TALISE_setPllLoopFilter(phy->talDevice, TAL_RF_PLL + chan->channel, loop_bw,
// 							phy->loopFilter_stability);
// 				phy->current_loopBandwidth_kHz[chan->channel] = loop_bw;
// 			}
//
// 			ret = TALISE_setRfPllFrequency(phy->talDevice, TAL_RF_PLL + chan->channel,
// 						       readin);
// 			if (ret != TALACT_NO_ACTION) {
// 				adrv9002_set_radio_state(phy, RADIO_RESTORE_STATE);
// 				break;
// 			}
//
// 			ret = TALISE_getPllsLockStatus(phy->talDevice, &status);
// 			if (!((status & BIT(chan->channel + 1) || (ret != TALACT_NO_ACTION))))
// 				ret = -EFAULT;
//
// 			phy->trx_lo_frequency = readin;
// 			adrv9002_set_radio_state(phy, RADIO_RESTORE_STATE);
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

	u64 val;
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
		case LOEXT_FREQ:
// 			ret = TALISE_getRfPllFrequency(phy->talDevice, TAL_RF_PLL + chan->channel,
// 						       &val);
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


static const struct iio_chan_spec adrv9002_phy_chan[] = {
	{	/* RX LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "RX_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
	}, {	/* TX_LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "TX_LO",
		.ext_info = adrv9002_phy_ext_lo_info,
	}, /*{*/	/* TX1 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = 0,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
// 		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_tx_ext_info,
// 	}, {	/* RX1 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_RX1,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_rx_ext_info,
// 	}, {	/* TX2 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = 1,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
// 		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_tx_ext_info,
// 	}, {	/* RX2 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_RX2,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_rx_ext_info,
// 	}, {	/* RX Sniffer/Observation */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_OBS_RX1,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_obs_rx_ext_info,
// 	}, {	/* RX Sniffer/Observation */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_OBS_RX2,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
// 		.ext_info = adrv9002_phy_obs_rx_ext_info,
// 	}, {	/* AUXADC0 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_AUXADC0,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXADC1 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_AUXADC1,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXADC2 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_AUXADC2,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXADC3 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.channel = CHAN_AUXADC3,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC0 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC0,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC1 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC1,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC2 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC2,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC3 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC3,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC4 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC4,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC5 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC5,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC6 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC6,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC7 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC7,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC8 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC8,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC9 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC9,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
// 	}, {	/* AUXDAC10 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC10,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE),
// 	}, {	/* AUXDAC11 */
// 		.type = IIO_VOLTAGE,
// 		.indexed = 1,
// 		.output = 1,
// 		.channel = CHAN_AUXDAC11,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
// 		BIT(IIO_CHAN_INFO_SCALE),
// 	}, {
// 		.type = IIO_TEMP,
// 		.indexed = 1,
// 		.channel = 0,
// 		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
// 	},
};



static const struct iio_info adrv9002_phy_info = {
// 	.read_raw = &adrv9002_phy_read_raw,
// 	.write_raw = &adrv9002_phy_write_raw,
// 	.debugfs_reg_access = &adrv9002_phy_reg_access,
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
	ssiConfigSettingsInitDefault(&ssiConfig);
	adi_adrv9001_ResourceCfg_t adrv9001ResourceCfg = { adrv9001Init, adrv9001RadioConfig, &adrv9001PlatformFiles };
	uint8_t initCalsError = 0;
	uint8_t channelMask = 0;
	static const uint8_t ALL_RX_CHANNEL_MASK = 0xF;
	/* TX_CHANNEL_MASK_OFFSET is '4' as channel mask info for RX1/2 and ORX1/2 occupy the least 4 bits */
	static const uint8_t TX_CHANNEL_MASK_OFFSET = 4;
	int ret;

	adi_adrv9001_SpiSettings_t spiSettings = {
		.msbFirst = 1,
		.enSpiStreaming = 0,
		.autoIncAddrUp = 1,
		.fourWireMode = 1,
		.cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
	};

	channelMask = (adrv9001Init->tx.txInitChannelMask << TX_CHANNEL_MASK_OFFSET) | (adrv9001Init->rx.rxInitChannelMask & ALL_RX_CHANNEL_MASK);


	adi_adrv9001_HwOpen(adrv9001Device, &spiSettings);

	adrv9001Device_glob = adrv9001Device;

//	adi_common_LogLevelSet(&adrv9001Device->common, 0x7F);


	ADI_EXPECT(adi_adrv9001_InitAnalog, adrv9001Device, adrv9001Init, adrv9001RadioConfig->radioCtrlInit.adrv9001DeviceClockDivisor);
	ADI_EXPECT(adi_adrv9001_InitDigitalLoad, adrv9001Device, &adrv9001ResourceCfg);

	ssiConfig.ssiMode = ADI_ADRV9001_LVDS_2LANE_16I16Q;

	ADI_EXPECT(adi_adrv9001_InitRadioLoad, adrv9001Device, NULL, &adrv9001ResourceCfg, &ssiConfig, channelMask);
	ADI_EXPECT(adi_adrv9001_InitCalsRun, adrv9001Device, NULL, &adrv9001RadioConfig->initCals, 60000, &initCalsError);

	ADI_EXPECT(adi_adrv9001_InitRxGainParameters, adrv9001Device, adrv9001Init, &adrv9001ResourceCfg);

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_RX, ADI_CHANNEL_1, ADI_ADRV9001_SPI_MODE);

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_TX, ADI_CHANNEL_1, ADI_ADRV9001_SPI_MODE);

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_RX, ADI_CHANNEL_1);

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_TX, ADI_CHANNEL_1);

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_RX, ADI_CHANNEL_2, ADI_ADRV9001_SPI_MODE);

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(adrv9001Device, ADI_TX, ADI_CHANNEL_2, ADI_ADRV9001_SPI_MODE);

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_RX, ADI_CHANNEL_2);

	ret = adi_adrv9001_Radio_Channel_Prime(adrv9001Device, ADI_TX, ADI_CHANNEL_2);

	ret = adi_adrv9001_Rx_GainCtrlMode_Set(adrv9001Device, ADI_CHANNEL_2, ADI_ADRV9001_MGC);

	ret = adi_adrv9001_Rx_MinMaxGainIndex_Set(adrv9001Device, 1, 183, 255);

	adrv9001_RxInterfaceGainCtrl_t rxInterfaceGainConfig_11 =
	{
		.updateInstance = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NOW,
		.gainControlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL,
		.gain = ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB,
		.signalPar_dB = 0
	};

	ret = adi_adrv9001_Rx_InterfaceGain_Configure(adrv9001Device, ADI_CHANNEL_1, &rxInterfaceGainConfig_11);
	ret = adi_adrv9001_Rx_InterfaceGain_Configure(adrv9001Device, ADI_CHANNEL_2, &rxInterfaceGainConfig_11);

	adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_RX, ADI_CHANNEL_1);
	adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_RX, ADI_CHANNEL_2);
	adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_TX, ADI_CHANNEL_1);
	adi_adrv9001_Radio_Channel_EnableRf(adrv9001Device, ADI_TX, ADI_CHANNEL_2);

	return ret;

}

static int adrv9002_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9002_rf_phy *phy;
	struct clk *clk = NULL;
	const char *name;
	adi_common_ApiVersion_t apiVersion;
	adi_adrv9001_ArmVersion_t armVersion;
	int ret, i;
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
out_unregister_notifier:

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
