/**
* \file
* \brief Contains ADRV9001 API Rx datapath data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_RX_TYPES_H_
#define _ADI_ADRV9001_RX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* ADI specific header files */
//#include "adi_adrv9001_radioctrl_types.h"


/* Header files related to libraries */


/* System header files */
#include <stdint.h>
    
/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

#define ADI_ADRV9001_NUM_GAIN_COMP9						6U
#define ADI_ADRV9001_MAG_COMP_NB_PFIR_COEFS_MAX_SIZE	13
#define ADI_ADRV9001_MAG_COMP_PFIR_COEFS_MAX_SIZE		21
#define ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE			128

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/**
*  \brief Enum of possible Rx gain table SRAM base addresses
*/

/* FIXME: Vivek - Need to confirm all these values with ARM firmware team */
typedef enum adrv9001_RxChanGainTableBaseAddr_e
{
    ADI_ADRV9001_RX1_GAIN_TABLE_BASEADDR = 0x73300000,              /*!< Select gain table SRAM base address for Rx1 Channel */
    ADI_ADRV9001_RX2_GAIN_TABLE_BASEADDR = 0x73400000,              /*!< Select gain table SRAM base address for Rx2 Channel */
} adrv9001_RxChanGainTableBaseAddr_e;

// TODO: Refactor into separate enums
/**
*  \brief Enum of possible Rx channel enables
*/
typedef enum adi_adrv9001_RxChannels
{
    ADI_ADRV9001_RXOFF = 0x00,  /*!< No Rx/ORx channels are enabled */
    ADI_ADRV9001_RX1   = 0x01,  /*!< Rx1 channel enabled */
    ADI_ADRV9001_RX2   = 0x02,  /*!< Rx2 channel enabled */
    ADI_ADRV9001_ORX1  = 0x04,  /*!< ORx1 channel enabled */
    ADI_ADRV9001_ORX2  = 0x08,  /*!< ORx2 channel enabled */
    ADI_ADRV9001_ILB1  = 0x10,  /*!< Tx1 internal loopback into ORx1 channel enabled */
    ADI_ADRV9001_ILB2  = 0x20,  /*!< Tx2 internal loopback into ORx2 channel enabled */
    ADI_ADRV9001_ELB1  = 0x40,  /*!< Tx1 external loopback into ORx1 channel enabled */
    ADI_ADRV9001_ELB2  = 0x80   /*!< Tx2 external loopback into ORx2 channel enabled */
} adi_adrv9001_RxChannels_e;	

/**
 * \brief Enumerations of the SINC_MUX5 options
 */
typedef enum adi_adrv9001_RxSincMux5Output 
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO = 0u,  /*!< Output zero   */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC3 = 1u, /*!< Select SINC3 for the output     */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC4 = 2u, /*!< Select SINC4 for the output     */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6 = 4u, /*!< Select SINC6 for the toutput    */
} adi_adrv9001_RxSincMux5Output_e;

/**
 * \brief Enumerations of the SINC gain settings
 */
typedef enum adi_adrv9001_RxSincGainMuxOutput 
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_0_DB      = 0u,      /*!< 0 dB   */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB      = 1u,      /*!< 6 dB   */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_12_DB     = 2u,     /*!< 12 dB   */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_6_DB  = 3u,  /*!< -6 dB   */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_12_DB = 4u  /*!< -12 dB   */
} adi_adrv9001_RxSincGainMuxOutput_e;

/**
 * \brief Enumerations of the HB_MUX options 
 */
typedef enum adi_adrv9001_RxHBMuxOutput 
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_ZERO = 0u, /*!< Output zero   */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB0 = 1u,  /*!< Select HB0 for the output     */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2 = 2u,  /*!< Select HB2 for the output     */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1 = 4u   /*!< Select HB1 for the toutput    */
} adi_adrv9001_RxHBMuxOutput_e;

/**
 *  \brief Enumerations of PFIR input selection
 */
typedef enum adi_adrv9001_RxWbNbPfirInSel 
{
    ADI_ADRV9001_RX_WB_NB_PFIR_SEL_INT_IN = 0u, /*!< PFIR input from RXNB NCO */
    ADI_ADRV9001_RX_WB_NB_PFIR_SEL_EXT_IN = 1u, /*!< PFIR input from RP point */
} adi_adrv9001_RxWbNbPfirInSel_e;

/**
 *  \brief Enumerations of PFIR bank selection
 */
typedef enum adi_adrv9001_PfirBank 
{
    ADI_ADRV9001_PFIR_BANK_A = 0u, /*!< PFIR bank A */
    ADI_ADRV9001_PFIR_BANK_B = 1u, /*!< PFIR bank B */
    ADI_ADRV9001_PFIR_BANK_C = 2u, /*!< PFIR bank C */
    ADI_ADRV9001_PFIR_BANK_D = 3u, /*!< PFIR bank D */
} adi_adrv9001_PfirBank_e;

/**
 *  \brief Enumerations to determine symmetricity of PFIR coefficients
 */
typedef enum adi_adrv9001_PfirSymmetric 
{
    ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC = 0u, /*!< Coefficients are expected to be non-symmetric */
    ADI_ADRV9001_PFIR_COEF_SYMMETRIC     = 1u, /*!< Coefficients are expected to be symmetric */
} adi_adrv9001_PfirSymmetric_e;

/**
 *  \brief Enumerations to determine number of PFIR filter taps
 */
typedef enum adi_adrv9001_PfirNumTaps 
{
    ADI_ADRV9001_PFIR_32_TAPS     = 0u, /*!<  32 taps PFIR */
    ADI_ADRV9001_PFIR_64_TAPS     = 1u, /*!<  64 taps PFIR */
    ADI_ADRV9001_PFIR_96_TAPS     = 2u, /*!<  96 taps PFIR */
    ADI_ADRV9001_PFIR_128_TAPS    = 3u, /*!< 128 taps PFIR */
    ADI_ADRV9001_PFIR_TAPS_MAX_ID = 3u, /*!< PFIR taps max ID */
} adi_adrv9001_PfirNumTaps_e;

/**
 *  \brief Enumerations for PFIR gain
 */
typedef enum adi_adrv9001_PfirGain 
{
    ADI_ADRV9001_PFIR_GAIN_NEG_12_DB   = 0u, /*!< -12dB */
    ADI_ADRV9001_PFIR_GAIN_NEG_6_DB    = 1u, /*!< - 6dB */
    ADI_ADRV9001_PFIR_GAIN_ZERO_DB     = 2u, /*!<   0dB */
    ADI_ADRV9001_PFIR_GAIN_POS_6_DB    = 3u, /*!< + 6dB */
    ADI_ADRV9001_PFIR_GAIN_POS_9_54_DB = 4u, /*!< + 9.54dB */
    ADI_ADRV9001_PFIR_GAIN_POS_12_DB   = 5u, /*!< +12dB */
    ADI_ADRV9001_PFIR_GAIN_POS_14_DB   = 6u, /*!< +14dB */
    ADI_ADRV9001_PFIR_GAIN_POS_20_DB   = 7u, /*!< +20dB */
    ADI_ADRV9001_RX_MAX                = 7u,

    ADI_ADRV9001_PFIR_GAIN_PLUS_24DB   = 8u, /*!< +24dB */
    ADI_ADRV9001_PFIR_GAIN_PLUS_26DB   = 9u, /*!< +26dB */
    ADI_ADRV9001_PFIR_GAIN_MAX         = 9u,
} adi_adrv9001_PfirGain_e;

/**
 *  \brief Enum to set the Rx Gain control mode
 */
typedef enum 
{
    ADI_ADRV9001_MGC = 0,     /*!< Manual Gain Control */
    ADI_ADRV9001_AGC = 2      /*!< Automatic Gain Control (Slow Loop) */
} adi_adrv9001_RxGainCtrlMode_e;

/*! Enum for RX dpinfifo mode settings */
typedef enum adi_adrv9001_RxDpInFifoMode
{
    ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING = 0u, /*!< In detecting mode */
    ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTED  = 1u, /*!< In detected mode  */
} adi_adrv9001_RxDpInFifoMode_e;

/*! Enum for RX dpinfifo input data select */
typedef enum adi_adrv9001_RxDpInFifoInputSel
{
    ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL  = 0u, /*!< select data from datapath */
    ADI_ADRV9001_DP_IN_FIFO_INPUT_SPI_SEL = 1u, /*!< select test pattern from SPI  */
} adi_adrv9001_RxDpInFifoInputSel_e;

/*! Enum for RX WbNb Compensation PFIR input select */
typedef enum adi_adrv9001_RxPfirInMuxSel
{
    ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN = 0u,
    ADI_ADRV9001_RP_FIR_IN_MUX_EXT_IN = 1u,
} adi_adrv9001_RxPfirInMuxSel_e;

typedef enum adi_adrv9001_RxGsOutMuxSel
{
    ADI_ADRV9001_GS_OUT_MUX_GS_OUT = 0u,
    ADI_ADRV9001_GS_OUT_MUX_BYPASS = 1u,
} adi_adrv9001_RxGsOutMuxSel_e;

/*! Enum for Rx out select settings */
typedef enum adi_adrv9001_RxOutSel
{
    ADI_ADRV9001_RX_OUT_IQ_SEL    = 0u, /*!< IQ out */
    ADI_ADRV9001_RX_OUT_FMDM_SEL  = 1u, /*!< FM demodulation out */
    ADI_ADRV9001_RX_OUT_SEL_PFIR  = 2u, /*!< PFIR out */
} adi_adrv9001_RxOutSel_e;

/*! Enum for RX round output mode */
typedef enum adi_adrv9001_RxRoundMode
{
    ADI_ADRV9001_RX_ROUNDMODE_IQ = 0u, /*!< IQ output */
    ADI_ADRV9001_RX_ROUNDMODE_I  = 1u, /*!< I only output */
} adi_adrv9001_RxRoundMode_e;

/*! Enum for RX output select */
typedef enum adi_adrv9001_RxDpArmSel
{
    ADI_ADRV9001_DP_SEL  = 0u, /*!< select datapath output */
    ADI_ADRV9001_ARM_SEL = 1u, /*!< select arm output */
} adi_adrv9001_RxDpArmSel_e;

/**
 * \brief Enumeration of the Rx signal type
 */
typedef enum adi_adrv9001_RxSignalType
{
    ADI_ADRV9001_RX_IQ    = 0,
    ADI_ADRV9001_RX_FREQUENCY_DEVIATION = 1,
    ADI_ADRV9001_RX_FM_SYMBOLS = 2
} adi_adrv9001_RxSignalType_e;

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/

/**
* \brief Data structure to hold ADRV9001 Rx RSSI status
*/
typedef struct adi_adrv9001_RxRssiStatus
{
    double linearPower;	/* Linear Power */
    float  dBPower;     /* dB Power */
} adi_adrv9001_RxRssiStatus_t;

/**
 * \brief Data structure to hold PFI coefficients
 */
typedef struct adi_adrv9001_PfirCoeff
{
    uint32_t numCoeff; /*!< Number of Coefficients */
} adi_adrv9001_PfirCoeff_t;

/* TODO: With new PFIR structure change, this structure is not used anywhere now. 
         But according to Jim Bush, this structure will be used in dynamic profile switching */
/**
 * \brief Data structure to hold PfirWbNbConfig
 */
typedef struct adi_adrv9001_PfirWbNbConfig
{
    adi_adrv9001_PfirBank_e      bankSel;		/*!< bank: PFIR_BANK_A = 0u (bank A), PFIR_BANK_B = 1u (bank B),
                                                           PFIR_BANK_C = 2u (bank C), PFIR_BANK_D = 3u (bank D) */
    adi_adrv9001_PfirSymmetric_e symmetricSel;	/*!< PFIR coefficient symmetricity */
    adi_adrv9001_PfirNumTaps_e   tapsSel;		/*!< Number of taps */
    adi_adrv9001_PfirGain_e      gainSel;		/*!< gain in dB */
    adi_adrv9001_PfirCoeff_t     coeff;         /*!< PFIR coefficients */
} adi_adrv9001_PfirWbNbConfig_t;

/**
 * \brief Data structure to hold ADRV9001 RX Narrowband decimation top block
 */
typedef struct adi_adrv9001_RxNbDecTop 
{
    uint8_t scicBlk23En;			/*!< Setting for block #23. RSCIC */
    uint8_t scicBlk23DivFactor;		/*!< STATIC or DYNAMIC, BBIC to determine based on user case */
    uint8_t scicBlk23LowRippleEn;	/*!< Enable low ripple mode */
    uint8_t decBy2Blk35En;			/*!< Setting for block #35. DEC2_1 */
    uint8_t decBy2Blk37En;			/*!< Setting for block #37. DEC2_2 */
    uint8_t decBy2Blk39En;			/*!< Setting for block #39. DEC2_3 */
    uint8_t decBy2Blk41En;			/*!< Setting for block #41. DEC2_4 */
    uint8_t decBy2Blk43En;			/*!< Setting for block #43. DEC2_5 */
    uint8_t decBy3Blk45En;			/*!< Setting for block #45. DEC3   */
    uint8_t decBy2Blk47En;			/*!< Setting for block #47. DEC2_6 */
} adi_adrv9001_RxNbDecTop_t;

/**
 * \brief Data structure to hold ADRV9001 RX Wideband decimation top block 
 */
typedef struct adi_adrv9001_RxWbDecTop 
{
    uint8_t decBy2Blk25En;	/*!< Setting for block #25. DEC2_1 */
    uint8_t decBy2Blk27En;	/*!< Setting for block #27. DEC2_2 */
    uint8_t decBy2Blk29En;	/*!< Setting for block #29. DEC2_3 */
    uint8_t decBy2Blk31En;	/*!< Setting for block #31. DEC2_4 */
    uint8_t decBy2Blk33En;	/*!< Setting for block #33. DEC2_5 */
    uint8_t wbLpfBlk33p1En; /*!< Setting for block #33.1. WBLPF */
} adi_adrv9001_RxWbDecTop_t;

/**
* \brief Data structure to hold ADRV9001 RX decimation top block 
*/
typedef struct adi_adrv9001_RxDecTop 
{
    uint8_t decBy3Blk15En;		/*!< Setting for block #15.   DEC2 */
    uint8_t decBy2Hb3Blk17p1En; /*!< Setting for block #17.1. HB3 */
    uint8_t decBy2Hb4Blk17p2En; /*!< Setting for block #17.2. HB4 */
    uint8_t decBy2Hb5Blk19p1En; /*!< Setting for block #19.1. HB5 */
    uint8_t decBy2Hb6Blk19p2En; /*!< Setting for block #19.2. HB6 */
} adi_adrv9001_RxDecTop_t;

/**
 * \brief Data structure to hold ADRV9001 RX Sinc Halfband top block
 */
typedef struct adi_adrv9001_RxSincHbTop 
{
    adi_adrv9001_RxSincGainMuxOutput_e	sincGainMux;			/*!< Sinc gain setting */
    adi_adrv9001_RxSincMux5Output_e		sincMux;				/*!< Sinc MUX selection */
    adi_adrv9001_RxHBMuxOutput_e		hbMux;					/*!< HB MUX selection */

    uint8_t								isGainCompEnabled;      /*!< Gain compensation */
    int16_t								gainComp9GainI[ADI_ADRV9001_NUM_GAIN_COMP9];
    int16_t								gainComp9GainQ[ADI_ADRV9001_NUM_GAIN_COMP9];
} adi_adrv9001_RxSincHbTop_t;

/*! dpinfifo configuration structure */
typedef struct adi_adrv9001_RxDpInFifoConfig
{
    uint8_t								dpInFifoEn;                  /*!< dpinfifo enable */
    adi_adrv9001_RxDpInFifoMode_e		dpInFifoMode;
    adi_adrv9001_RxDpInFifoInputSel_e	dpInFifoTestDataSel;
} adi_adrv9001_RxDpInFifoConfig_t;

/**
 * \brief Parameters for IQDM NCO
 */
typedef struct adi_adrv9001_NcoDpConfig
{
    int32_t  freq;        /*!< NCO output frequency */
    uint32_t sampleFreq;  /*!< Sampling frequency at NCO connects to */
    uint16_t phase;       /*!< Phase offset */
    uint8_t  realOut;     /*!< Real out enable */
} adi_adrv9001_NcoDpConfig_t;

/**
 * \brief Data structure to hold ADRV9001 Rx NB NCO structure
 */
typedef struct adi_adrv9001_RxNbNcoConfig
{
    uint8_t                      rxNbNcoEn;		/*!< rxnb nco enable */
    adi_adrv9001_NcoDpConfig_t rxNbNcoConfig;
} adi_adrv9001_RxNbNcoConfig_t;

/**
 * \brief Data structure to hold ADRV9001 Rx WbNb compensation PFir structure
 */
typedef struct adi_adrv9001_RxWbNbCompPFir
{
    adi_adrv9001_PfirBank_e      bankSel;		/*!< bank: PFIR_BANK_A = 0u (bank A), PFIR_BANK_B = 1u (bank B),
                                                           PFIR_BANK_C = 2u (bank C), PFIR_BANK_D = 3u (bank D) */
    adi_adrv9001_RxPfirInMuxSel_e   rxWbNbCompPFirInMuxSel;
    uint8_t							rxWbNbCompPFirEn;            /*!< rx WbNb compensation PFir enable */
} adi_adrv9001_RxWbNbCompPFir_t;

/**
 * \brief Data structure to hold ADRV9001 Rx resampler structure
 */
typedef struct adi_adrv9001_RxResampConfig
{
    uint8_t            rxResampEn;                 /*!< resampler enable */
    uint32_t           resampPhaseI;               /*!< I channel resampler phase */
    uint32_t           resampPhaseQ;               /*!< Q channel resampler phase */
} adi_adrv9001_RxResampConfig_t;

/**
 * \brief Data structure to hold ADRV9001 RX Dem block
 */
typedef struct adi_adrv9001_RxNbDemConfig
{
    adi_adrv9001_RxDpInFifoConfig_t	dpInFifo;
    adi_adrv9001_RxNbNcoConfig_t    rxNbNco;
    adi_adrv9001_RxWbNbCompPFir_t   rxWbNbCompPFir;
    adi_adrv9001_RxResampConfig_t   resamp;
    adi_adrv9001_RxGsOutMuxSel_e    gsOutMuxSel; /*!< fic algorithm enable */
    adi_adrv9001_RxOutSel_e         rxOutSel;
    adi_adrv9001_RxRoundMode_e      rxRoundMode;
    adi_adrv9001_RxDpArmSel_e       dpArmSel;
} adi_adrv9001_RxNbDemConfig_t;

/**
 *  \brief Data structure to hold ADRV9001 Rx Data Path profile settings
 */
typedef struct adi_adrv9001_RxDpProfile
{
    adi_adrv9001_RxNbDecTop_t		rxNbDecTop;		/*!< RX narrowband decimation top */
    adi_adrv9001_RxWbDecTop_t		rxWbDecTop;		/*!< RX wideband decimation top */
    adi_adrv9001_RxDecTop_t			rxDecTop;		/*!< RX Common decimation top */
    adi_adrv9001_RxSincHbTop_t		rxSincHBTop;	/*!< RX Sinc HB top */
    adi_adrv9001_RxNbDemConfig_t	rxNbDem;		/*!< RX NB Dem block */
} adi_adrv9001_RxDpProfile_t;

/**
 * \brief Data structure to hold ADRV9001 Rx gain table row entry
 */
typedef struct adi_adrv9001_RxGainTableRow
{
    uint8_t rxFeGain;     /*!< Rx Front End gain for a given gain index */
    uint8_t extControl;   /*!< External LNA control word */
    uint8_t adcTiaGain;   /*!< ADC and TIA control for a given gain index */
    int16_t digGain;      /*!< Digital gain ranging from -18dB to 50dB (68dB total range) */
    uint16_t phaseOffset; /*!< 16 bit phase offset from 0 - 2pi in resolution of 0.005 degrees */
} adi_adrv9001_RxGainTableRow_t;

/**
*  \brief Enum of Rx interface gain control update instance
*/
typedef enum adrv9001_RxInterfaceGainCtrl_Update
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NEXT_FRAME = 0x00,
    /*!< Update Rx interface gain control at start of next frame */
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_UPDATE_NOW = 0x01,
    /*!< Update Rx interface gain control immediately  */
} adrv9001_RxInterfaceGainCtrl_Update_e;

/**
*  \brief Enum to determine Rx interface gain control type
*/
typedef enum adrv9001_RxInterfaceGainCtrl_Type
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC = 0x00,
    /*!< Use internal Rx interface gain value. */
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL = 0x01,
    /*!< Use external Rx interface gain value. Gain value has to be provided in this case. */
} adrv9001_RxInterfaceGainCtrl_Type_e;

/**
*  \brief Enum to determine Rx interface gain value
*/
typedef enum adrv9001_RxInterfaceGain
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB = 0x00,    /*!< 18 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_12_DB = 0x01,    /*!< 12 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_6_DB = 0x02,     /*!<  6 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB = 0x03,     /*!<  0 dB */
} adrv9001_RxInterfaceGain_e;

/**
* \brief Data structure to set/get the Rx interface gain control parameters for the given Rx channel.
*/
typedef struct adrv9001_RxInterfaceGainCtrl
{
    adrv9001_RxInterfaceGainCtrl_Update_e updateInstance;  /*!< Time at which Rx interface gain control has to be updated.
                                                                0: To be updated at start of next frame
                                                                1: To be updated immediately */
    adrv9001_RxInterfaceGainCtrl_Type_e   gainControlMode; /*!< 0: Uses internal Rx interface gain value.
                                                                1: Uses external Rx interface gain value. Gain value has to be provided in this case. */
    adrv9001_RxInterfaceGain_e            gain;            /*!< Should be initialized to '-1' by default. Used only when configType = 1, ignored otherwise. 
                                                                Can have a value between 0 and 3 (0x0 = 0dB, 0x1 = 6dB, 0x2 = 12dB, 0x3 = 18dB). */
    uint8_t signalPar_dB;                                  /*!< expected ratio of peak value on any channel to average "I^2 + Q^2" value in dB */
} adrv9001_RxInterfaceGainCtrl_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_TYPES_H_ */
