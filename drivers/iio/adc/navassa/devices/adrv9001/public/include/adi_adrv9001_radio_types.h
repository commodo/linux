/**
 * \file
 * \brief Contains ADRV9001 API Radio Control data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RADIO_TYPES_H_
#define _ADI_ADRV9001_RADIO_TYPES_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FIXME: Vivek - Need to add structures and enum related to radio control for Navassa ADRV9001
                  Also need to confirm about 'ADRV9001_ORXSTREAMSIZEMAX' and 'ADRV9001_ORXNUM' */


//#define ADRV9001_IMAGEINFOSIZE		4
#define ADRV9001_MAINSTREAMSIZEMAX	   16*1024	/* Memory size for Main stream processor */
#define ADRV9001_TXSTREAMSIZEMAX		4*1024	/* Memory size for Tx stream processor */
#define ADRV9001_RXSTREAMSIZEMAX		4*1024	/* Memory size for Rx stream processor */
//#define ADRV9001_ORXSTREAMSIZEMAX		4*1024	/* Memory size for ORx stream processor */
#define ADRV9001_MAINNUM				1		/* Maximum number of main stream processors */
#define ADRV9001_TXNUM					2		/* Maximum number of Tx channels */
#define ADRV9001_RXNUM					2		/* Maximum number of Rx channels */
//#define ADRV9001_ORXNUM				2		/* Maximum number of ORx channels */

/**
 *  \brief Enum of PLL selections
 */
typedef enum adi_adrv9001_PllName
{
    ADI_ADRV9001_LO1_PLL = 0, /*!< Selects PLL LO1 for Rx and Tx */
    ADI_ADRV9001_LO2_PLL,     /*!< Selects PLL LO2 for Rx and Tx */
    ADI_ADRV9001_AUX_PLL      /*!< Selects AUX PLL for Rx and tx*/
} adi_adrv9001_PllName_e;

/**
 *  \brief Enum of PLL calibration mode
 */
typedef enum adrv9001_PllCalMode 
{
    ADI_ADRV9001_PLL_CAL_MODE_NORM = 0x00, /*!< PLL calibration setting in Normal mode */
    ADI_ADRV9001_PLL_CAL_MODE_FAST = 0x01, /*!< PLL calibration setting in Fast mode  */
    ADI_ADRV9001_PLL_RESERVED      = 0x02  /*!< RESERVED */
} adrv9001_PllCalMode_e;

/**
 * TODO: Remove
 * \brief Enum of all ADRV9001 channels
 */
typedef enum adi_adrv9001_Channel
{
    ADI_ADRV9001_CHANNELS_NONE  =   0x00,
    ADI_ADRV9001_CHANNEL_RX1    =   0x01,
    ADI_ADRV9001_CHANNEL_RX2    =   0x02,
    ADI_ADRV9001_CHANNEL_ORX1   =   0x04,
    ADI_ADRV9001_CHANNEL_ORX2   =   0x08,
    ADI_ADRV9001_CHANNEL_TX1    =   0x10,
    ADI_ADRV9001_CHANNEL_TX2    =   0x20,
    ADI_ADRV9001_CHANNELS_ALL   =   0x3F
} adi_adrv9001_Channel_e;
    
/**
 * \brief Modes controlling how a channel is enabled
 */    
typedef enum adi_adrv9001_ChannelEnableMode
{
    ADI_ADRV9001_SPI_MODE = 0,      // Channel is enabled via SPI
    ADI_ADRV9001_PIN_MODE           // Channel is enabled via GPIO pin
} adi_adrv9001_ChannelEnableMode_e;

typedef enum adi_adrv9001_ChannelState
{
    ADI_ADRV9001_CHANNEL_STANDBY,       /*!< Initial state for all channels once ARM completes boot sequences */
    ADI_ADRV9001_CHANNEL_CALIBRATED,    /*!< Minimum set of initial calibration done without errors; CLK PLL is on; data paths are off */
    ADI_ADRV9001_CHANNEL_PRIMED,        /*!< Data path and interface are on; tracking algorithms NOT scheduled, TRx NOT transmitting or receiving data */
    ADI_ADRV9001_CHANNEL_RF_ENABLED     /*!< Data path and interface are on; tracking algorithms are scheduled, TRx is transmitting or receiving data */
} adi_adrv9001_ChannelState_e;

/**
 *  \brief Data structure to hold radio state information
 */
typedef struct adi_adrv9001_RadioState
{
    adi_adrv9001_ArmSystemStates_e systemState;             /*!< ARM System State */
    adi_adrv9001_ArmMonitorModeStates_e monitorModeState;   /*!< ARM Monitor Mode State (only valid when systemState is in Monitor Mode) */
    adi_adrv9001_ArmBootStates_e bootState;                 /*!< ARM Boot State (only valid when systemState is in POWERUP state) */
    adi_adrv9001_ChannelState_e channelStates[ADI_ADRV9001_NUM_PORTS][ADI_ADRV9001_NUM_CHANNELS + 1];   /*!< State of each channel; Wastes some memory for convenient indexing
                                                                                                         *   e.g., RX1 is channelStates[ADI_RX][ADI_CHANNEL_1] */
} adi_adrv9001_RadioState_t;

/**
 * \brief Data structure to hold Synthesizer PLL Loop filter settings
 */
typedef struct adi_adrv9001_PllLoopFilterCfg
{
    uint16_t effectiveLoopBandwidth_kHz; /*!< Synthesizer PLL effective Loop filter bandwidth. Range TBD. For read-only */
    uint16_t loopBandwidth_kHz; /*!< Synthesizer PLL Loop filter bandwidth. Range 50-1500 */
    uint8_t  phaseMargin_degrees; /*< Synthesizer PLL Loop filter phase margin in degrees. Range 40-85 */
    uint8_t  powerScale; /*!< Synthesizer PLL Loop filter power scale. Range 0 - 10. Default is 10 */
} adi_adrv9001_PllLoopFilterCfg_t;

/**
 *  \brief Enumeration of GPIO pins 
 */
typedef enum adi_adrv9001_ArmGpioPin
{
    ADI_ADRV9001_DGPIO_00_DO_NOT_ASSIGN = 0, /*!< DGPIO Pin 0 - Used to trigger ARM interrupt, do not assign */
    ADI_ADRV9001_DGPIO_01 = 1,               /*!< DGPIO Pin 1     */
    ADI_ADRV9001_DGPIO_02,                   /*!< DGPIO Pin 2     */
    ADI_ADRV9001_DGPIO_03,                   /*!< DGPIO Pin 3     */
    ADI_ADRV9001_DGPIO_04,                   /*!< DGPIO Pin 4     */
    ADI_ADRV9001_DGPIO_05,                   /*!< DGPIO Pin 5     */
    ADI_ADRV9001_DGPIO_06,                   /*!< DGPIO Pin 6     */
    ADI_ADRV9001_DGPIO_07,                   /*!< DGPIO Pin 7     */
    ADI_ADRV9001_DGPIO_08,                   /*!< DGPIO Pin 8     */
    ADI_ADRV9001_DGPIO_09,                   /*!< DGPIO Pin 9     */
    ADI_ADRV9001_DGPIO_10,                   /*!< DGPIO Pin 10    */
    ADI_ADRV9001_DGPIO_11,                   /*!< DGPIO Pin 11    */
    ADI_ADRV9001_DGPIO_12,                   /*!< DGPIO Pin 12    */
    ADI_ADRV9001_DGPIO_13,                   /*!< DGPIO Pin 13    */
    ADI_ADRV9001_DGPIO_14,                   /*!< DGPIO Pin 14    */
    ADI_ADRV9001_DGPIO_15,                   /*!< DGPIO Pin 15    */
    ADI_ADRV9001_AGPIO_00,                   /*!< AGPIO Pin 0     */
    ADI_ADRV9001_AGPIO_01,                   /*!< AGPIO Pin 1     */
    ADI_ADRV9001_AGPIO_02,                   /*!< AGPIO Pin 2     */
    ADI_ADRV9001_AGPIO_03,                   /*!< AGPIO Pin 3     */
    ADI_ADRV9001_AGPIO_04,                   /*!< AGPIO Pin 4     */
    ADI_ADRV9001_AGPIO_05,                   /*!< AGPIO Pin 5     */
    ADI_ADRV9001_AGPIO_06,                   /*!< AGPIO Pin 6     */
    ADI_ADRV9001_AGPIO_07,                   /*!< AGPIO Pin 7     */
    ADI_ADRV9001_AGPIO_08,                   /*!< AGPIO Pin 8     */
    ADI_ADRV9001_AGPIO_09,                   /*!< AGPIO Pin 9     */
    ADI_ADRV9001_AGPIO_10,                   /*!< AGPIO Pin 10    */
    ADI_ADRV9001_AGPIO_11,                   /*!< AGPIO Pin 11    */
    ADI_ADRV9001_GPIO_UNASSIGNED             /*!< Invalid GPIO Pin */
} adi_adrv9001_ArmGpioPin_e;
 
/**
 * \brief Enumeration of signals from the BBIC to ARM.
 *   These signals could be mapped to various GPIO pins. The GPIO_CTRL
 *    mailbox command is used to configure the GPIO pin to signal mapping.
 */
typedef enum adi_adrv9001_ArmGpioSignal
{
    /* Digital GPIO Functions */
    ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,  /*!< ORx Enable signal channel 1            */
    ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_2,  /*!< ORx Enable signal channel 2            */
    ADI_ADRV9001_GPIO_SIGNAL_MON_ENABLE,    /*!< Monitor mode enable request signal     */
    ADI_ADRV9001_GPIO_SIGNAL_MON_WAKEUP,    /*!< Monitor mode wake up DSP signal        */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP,        /*!< Frequency hopping hop request signal   */
    ADI_ADRV9001_GPIO_SIGNAL_FH_GAIN_SEL_0, /*!< Frequency hopping gain select bit 0    */
    ADI_ADRV9001_GPIO_SIGNAL_FH_GAIN_SEL_1, /*!< Frequency hopping gain select bit 1    */
    ADI_ADRV9001_GPIO_SIGNAL_FH_GAIN_SEL_2, /*!< Frequency hopping gain select bit 2    */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_0, /*!< Frequency hopping frequency index select bit 0 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_1, /*!< Frequency hopping frequency index select bit 1 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_2, /*!< Frequency hopping frequency index select bit 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_3, /*!< Frequency hopping frequency index select bit 3 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_4, /*!< Frequency hopping frequency index select bit 4 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_FREQ_SEL_5, /*!< Frequency hopping frequency index select bit 5 */
    
    ADI_ADRV9001_GPIO_SIGNAL_LNA_CONTROL_1, /*!< RX1 LNA attenuation control 1          */
    ADI_ADRV9001_GPIO_SIGNAL_LNA_CONTROL_2, /*!< RX1 LNA attenuation control 2          */
    ADI_ADRV9001_GPIO_SIGNAL_LNA_CONTROL_3, /*!< RX1 LNA attenuation control 3          */
    ADI_ADRV9001_GPIO_SIGNAL_LNA_CONTROL_4, /*!< RX1 LNA attenuation control 4          */
    ADI_ADRV9001_GPIO_SIGNAL_RX1_GAIN_UP,   /*!< RX1 Gain Up request signal             */
    ADI_ADRV9001_GPIO_SIGNAL_RX1_GAIN_DOWN, /*!< RX1 Gain Down request signal           */
    ADI_ADRV9001_GPIO_SIGNAL_RX2_GAIN_UP,   /*!< RX2 Gain Up request signal             */
    ADI_ADRV9001_GPIO_SIGNAL_RX2_GAIN_DOWN, /*!< RX2 Gain Down request signal           */
    ADI_ADRV9001_GPIO_SIGNAL_TX1_ATTN_UP,   /*!< TX1 Attenuation Up request signal      */
    ADI_ADRV9001_GPIO_SIGNAL_TX1_ATTN_DOWN, /*!< TX1 Attenuation Down request signal    */
    ADI_ADRV9001_GPIO_SIGNAL_TX2_ATTN_UP,   /*!< TX2 Attenuation Up request signal      */
    ADI_ADRV9001_GPIO_SIGNAL_TX2_ATTN_DOWN, /*!< TX2 Attenuation Down request signal    */
    
    ADI_ADRV9001_GPIO_SIGNAL_PA_RAMP_CTRL_1, /*!< Tx1 Aux DAC ramp control request signal*/
    ADI_ADRV9001_GPIO_SIGNAL_PA_RAMP_CTRL_2, /*!< Tx2 Aux DAC ramp control request signal*/
    
    /* Analog GPIO Functions */
    ADI_ADRV9001_GPIO_SIGNAL_TX1_ON,         /*!< Tx1 On signal                          */
    ADI_ADRV9001_GPIO_SIGNAL_TX2_ON,         /*!< Tx2 On signal                          */
    ADI_ADRV9001_GPIO_SIGNAL_RX1_ON,         /*!< Rx1 On signal                          */
    ADI_ADRV9001_GPIO_SIGNAL_RX2_ON,         /*!< Rx2 On signal                          */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_LOCK_1, /*!< External PLL 1 lock signal             */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_LOCK_2, /*!< External PLL 2 lock signal             */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_CE_1,   /*!< External PLL 1 CE signal               */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_CE_2,   /*!< External PLL 2 CE signal               */
    ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_CE_1,    /*!< Rx VCO 1 CE signal                     */
    ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_CE_2,    /*!< Rx VCO 2 CE signal                     */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_1,      /*!< Aux DAC control 1 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_2,      /*!< Aux DAC control 2 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_3,      /*!< Aux DAC control 3 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_4,      /*!< Aux DAC control 4 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_1,      /*!< Aux ADC control 1 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_2,      /*!< Aux ADC control 2 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_3,      /*!< Aux ADC control 3 signal               */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_4,      /*!< Aux ADC control 4 signal               */
    
    /* Future GPIO Functions */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_2,       /*!< Frequency hopping hop request signal   */
    ADI_ADRV9001_GPIO_SIGNAL_TX_CAL_EN,      /*!< Tx channel 1 and 2  calibration enable signal */
    ADI_ADRV9001_GPIO_SIGNAL_CAL_UPDATE,     /*!< Calibration update selection signal  */
    
    ADI_ADRV9001_GPIO_NUM_SIGNALS            /*!< Total Number of signals from BBIC    */
} adi_adrv9001_ArmGpioSignal_e;

/**
 * \brief Enumeration of GPIO signal polarity 
 **/
typedef enum adi_adrv9001_ArmGpioPolarity
{
    ADI_ADRV9001_GPIO_POLARITY_NORMAL,   /*!< Normal polarity */
    ADI_ADRV9001_GPIO_POLARITY_INVERTED, /*!< Inverted polarity */
} adi_adrv9001_ArmGpioPolarity_e;


/**
 * \brief Enumeration of GPIO feature control 
 */
typedef enum adi_adrv9001_ArmGpioControl
{
    ADI_ADRV9001_GPIO_CONTROL_BBIC, /*!< BBIC Control the Pin */
    ADI_ADRV9001_GPIO_CONTROL_ARM   /*!< ARM Control the Pin  */
} adi_adrv9001_ArmGpioControl_e;

/**
 * \brief Enum used to determine if a Mail Box command is allowed inPrimed state 
 */
typedef enum adi_adrv9001_MBSetInPrimedState
{
    ADI_ADRV9001_MB_NOT_ALLOWED, /*!< Mail Box command not allowed in primed state  */
    ADI_ADRV9001_MB_RESERVED     /*!< Reserved */
} adi_adrv9001_MBSetInPrimedState_e;

/**
 * \brief Structure which holds the GPIO to signal mapping 
 */
typedef struct adi_adrv9001_ArmGpioSignalMap
{
    adi_adrv9001_ArmGpioSignal_e    gpioSignalSel; /*!< GPIO signal of the GPIO pin */
    adi_adrv9001_ArmGpioPin_e       gpioPinSel;    /*!< GPIO pin mapped to the given signal */
    adi_adrv9001_ArmGpioPolarity_e  polarity;      /*!< Polarity of the GPIO pin (normal or inverted)*/
    adi_adrv9001_ArmGpioControl_e   control;       /*!< Whether BBIC or ARM controls this feature */
} adi_adrv9001_ArmGpioSignalMap_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RADIO_TYPES_H_ */
