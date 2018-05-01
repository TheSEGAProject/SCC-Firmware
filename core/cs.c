/**************************************************************************//**
 * \file cs.c
 * \brief File for configuring the clock system.
 * \addtogroup HAL
 * @{
 *****************************************************************************/

#include <msp432p401r.h>
#include "std.h"
#include "cs.h"

//! \def CS_PWD
//! \brief Password for clock system register access
#define CS_PWD	0x0000695AL


//! \struct s_CS_TLV_values
//! \brief Calibration values for the clock system from the tag length vector (TLV) table in memory
struct {
		unsigned long * pulIR_Cal;
		unsigned long * pulDCO_IR_RSEL04_MPT; ///< Max positive tune for DCORSEL 0 to 4 using internal resistor
		unsigned long * pulDCO_IR_RSEL04_MNT;	///< Max negative tune for DCORSEL 0 to 4 using internal resistor
		unsigned long * pulDCO_IR_RSEL5_MPT; 	///< Max positive tune for DCORSEL 5 using internal resistor
		unsigned long * pulDCO_IR_RSEL5_MNT;	///< Max negative tune for DCORSEL 5 using internal resistor
		float * pulDCO_IR_RSEL04_K;		///< DCO constant for DCORSEL 0 to 4 using internal resistor
		float * pulDCO_IR_RSEL5_K;		///< DCO constant for DCORSEL 5 using internal resistor

		unsigned long * pulER_Cal;
		unsigned long * pulDCO_ER_RSEL04_MPT; ///< Max positive tune for DCORSEL 0 to 4 using external resistor
		unsigned long * pulDCO_ER_RSEL04_MNT;	///< Max negative tune for DCORSEL 0 to 4 using external resistor
		unsigned long * pulDCO_ER_RSEL5_MPT; 	///< Max positive tune for DCORSEL 5 using external resistor
		unsigned long * pulDCO_ER_RSEL5_MNT;	///< Max negative tune for DCORSEL 5 using external resistor
		float * pulDCO_ER_RSEL04_K;		///< DCO constant for DCORSEL 0 to 4 using external resistor
		float * pulDCO_ER_RSEL5_K;		///< DCO constant for DCORSEL 5 using external resistor

}s_CS_TLV_values;

//! \struct s_CS_TLV_values
//! \brief Calibration values for the clock system from the tag length vector (TLV) table in memory
static struct {
		unsigned char ucDCO;	///< Digitilly controlled oscillator frequency
		unsigned char ucMCLK; ///< Master clock frequency
		unsigned char ucHSMLCK;	///< High speed sub-master clock frequency
		unsigned char SMLCK; 	///< Sub-master clock frequency

}s_CSFreq;

//! \var ucaDCORSELNominalFreq
//! \brief Array holds the nominal frequencies for each DCO range setting.
const float faDCORSELNominalFreq[6] = {
		1.5,
		3,
		6,
		12,
		24,
		48
};

/************************************************************************//**
 * \fn vCS_GetTLVdata
 *
 * \brief Reads calibration data for the clock system from the TLV table
 ***************************************************************************/
static void vCS_GetTLVdata(void){
	s_CS_TLV_values.pulIR_Cal = (unsigned long *)0x0020104CL;
	s_CS_TLV_values.pulDCO_IR_RSEL04_MPT = (unsigned long *)0x00201054L;
	s_CS_TLV_values.pulDCO_IR_RSEL04_MNT = (unsigned long *)0x00201058L;
	s_CS_TLV_values.pulDCO_IR_RSEL5_MPT = (unsigned long *)0x0020105CL;
	s_CS_TLV_values.pulDCO_IR_RSEL5_MNT = (unsigned long *)0x00201060L;
	s_CS_TLV_values.pulDCO_IR_RSEL04_K = (float *)0x00201064L;
	s_CS_TLV_values.pulDCO_IR_RSEL5_K = (float *)0x00201068L;

	s_CS_TLV_values.pulER_Cal = (unsigned long *)0x0020106CL;
	s_CS_TLV_values.pulDCO_ER_RSEL04_MPT = (unsigned long *)0x00201074L;
	s_CS_TLV_values.pulDCO_ER_RSEL04_MNT = (unsigned long *)0x00201078L;
	s_CS_TLV_values.pulDCO_ER_RSEL5_MPT = (unsigned long *)0x0020107CL;
	s_CS_TLV_values.pulDCO_ER_RSEL5_MNT = (unsigned long *)0x00201080L;
	s_CS_TLV_values.pulDCO_ER_RSEL04_K = (float*)0x00201084L;
	s_CS_TLV_values.pulDCO_ER_RSEL5_K = (float *)0x00201088L;
}

/***************************************************************************
 * \fn ucCS_GetDCORange
 * \brief Gets the values required to configure the DCO
 * \param *ulDCORange, Pointer to the DCO range variable
 * \param *pfDCOConstant, Pointer to the DCO constant variable
 * \param *pulDCOCal, Pointer to the DCO calibration variable
 * \param *pucNominalFreq, Pointer to the Nominal frequency variable
 * \return ucRetCode, Error(1) or Success(0)
 ***************************************************************************/
static unsigned char ucCS_GetDCOValues(unsigned long *pulDCORange, float *pfDCOConstant, unsigned long *pulDCOCal, float *pucNominalFreq){
	unsigned char ucRetCode;

	// Optimism is good
	ucRetCode = SUCCESS;

	// Determine DCO range select
	if (s_CSFreq.ucDCO >= 1 && s_CSFreq.ucDCO <= 2){
		*pulDCORange = DCORSEL_0;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL04_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[0];
	}
	else if (s_CSFreq.ucDCO > 2 && s_CSFreq.ucDCO <= 4){
		*pulDCORange = DCORSEL_1;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL04_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[1];
	}
	else if (s_CSFreq.ucDCO > 4 && s_CSFreq.ucDCO <= 8){
		*pulDCORange = DCORSEL_2;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL04_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[2];
	}
	else if (s_CSFreq.ucDCO > 8 && s_CSFreq.ucDCO <= 16){
		*pulDCORange = DCORSEL_3;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL04_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[3];
	}
	else if (s_CSFreq.ucDCO > 16 && s_CSFreq.ucDCO <= 32){
		*pulDCORange = DCORSEL_4;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL04_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[4];
	}
	else if (s_CSFreq.ucDCO > 32 && s_CSFreq.ucDCO <= 64){
		*pulDCORange = DCORSEL_5;
		*pfDCOConstant = *s_CS_TLV_values.pulDCO_ER_RSEL5_K;
		*pulDCOCal = *s_CS_TLV_values.pulER_Cal;
		*pucNominalFreq = faDCORSELNominalFreq[5];
	}
	else
		ucRetCode =  ERROR;

	return ucRetCode;
}

/***************************************************************************
 * \fn ucCS_SetFreqVars
 * \brief Sets the fields in the clock frequency structure
 * \param ulConfig, Desired clock clock configuration
 * \return Error(1) or Success(0)
 ***************************************************************************/
static unsigned char ucCS_SetFreqVars(unsigned char ucConfig){

	switch (ucConfig) {
		case CLOCK_CONFIG_1:
			s_CSFreq.ucDCO = 48;
			s_CSFreq.ucMCLK = 48;
			s_CSFreq.ucHSMLCK = 24;
			s_CSFreq.SMLCK = 12;
		break;

		case CLOCK_CONFIG_2:
			s_CSFreq.ucDCO = 16;
			s_CSFreq.ucMCLK = 16;
			s_CSFreq.ucHSMLCK = 4;
			s_CSFreq.SMLCK = 4;
		break;

		case CLOCK_CONFIG_3:
			s_CSFreq.ucDCO = 1;
			s_CSFreq.ucMCLK = 1;
			s_CSFreq.ucHSMLCK = 1;
			s_CSFreq.SMLCK = 1;
		break;

		default:
			s_CSFreq.ucDCO = 1;
			s_CSFreq.ucMCLK = 1;
			s_CSFreq.ucHSMLCK = 1;
			s_CSFreq.SMLCK = 1;
			return ERROR;
	}

	return SUCCESS;
}

/***************************************************************************
 * \fn ucCS_Init
 * \brief Initializes the clock system
 * \param ucConfig, Desired clock clock configuration
 * \return ucRetCode, Error(1) or Success(0)
 ***************************************************************************/
unsigned char ucCS_Init(unsigned char ucConfig){
	float fDCOConstant;
	float fTempNum;
	float fTempDen;
	float fNominalFreq;
	unsigned char ucRetCode;
	unsigned long ulDCORange;
	unsigned long ulDCOCal;
	signed long lDCOTuneBits;
	signed int iDCOTune;

	// The cup is half full
	ucRetCode = SUCCESS;

	// Make sure that the calibration values are read from TLV before continuing
	vCS_GetTLVdata();

	// Set the frequency variables based on the configuration
	if(ucCS_SetFreqVars(ucConfig) == ERROR)
		ucRetCode = ERROR;

	// Determine DCO range select
	if(ucCS_GetDCOValues(&ulDCORange, &fDCOConstant, &ulDCOCal, &fNominalFreq) == ERROR)
		ucRetCode = ERROR;

	// Compute the numerator and denominator of the DCO tune bits calculation
	fTempNum = (s_CSFreq.ucDCO - fNominalFreq)*(((fDCOConstant * (768 - ulDCOCal)) + 1)*8);
	fTempDen = (fDCOConstant * s_CSFreq.ucDCO);

	// Compute the tune bits
	lDCOTuneBits = fTempNum/fTempDen;

	// The DCO tune bits field is 13-bits wide and the value is two's complement,
	// therefore, we must do some bit manipulation.
	if(lDCOTuneBits < 0){
		iDCOTune = (signed int)(lDCOTuneBits & 0x1FFF) *-1;
		iDCOTune = (signed int)(iDCOTune ^ 0x1FFF)+1;
		iDCOTune &= 0x1FFF;
	}
	else{
	iDCOTune =	lDCOTuneBits & 0x00001FFF;
	}
	// Set up Port J bits to receive clock signal from 32 kHz crystal
	PJSEL0 |= BIT0 | BIT1;

	// Unlock the CS registers
  CSKEY = CS_PWD;

  CSCTL0 = DCOEN | DCORES | ulDCORange | iDCOTune;
  // Enable the low frequency oscillator at the maximum drive strength (reduce it later)
  CSCTL2 = LFXT_EN | LFXTDRIVE_7;

  CSCLKEN = SMCLK_EN | HSMCLK_EN | MCLK_EN | ACLK_EN;

  // Loop until XT1, XT2 & DCO fault flag is cleared
  do
  {
      // Clear XT2,XT1,DCO fault flags
     CSCLRIFG |= CLR_DCORIFG | CLR_HFXTIFG | CLR_LFXTIFG;
     SYSCTL_NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
  } while (SYSCTL_NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);// Test oscillator fault flag


  // Set the clock dividers based on the configuration
  switch (ucConfig)
	{
		case CLOCK_CONFIG_1:
		  // MCLK = DCO, SMCLK = DCO/4, HSMCLK = DCO/2, ACLK = BCLK = LFXTCLK
		  CSCTL1 = DIVS_2 | DIVA_0 | DIVHS_1 | DIVM_0 | SELA_0 | SELS_3 | SELM_3;
		break;
		case CLOCK_CONFIG_2:
		  // MCLK = DCO, SMCLK = DCO/8, ACLK = BCLK = LFXTCLK
		  CSCTL1 = DIVS_2 | DIVA_0 | DIVM_0 | SELA_0 | SELS_3 | SELM_3;
		break;
		case CLOCK_CONFIG_3:
		  // MCLK = DCO, HSMCLK = DCO, SMCLK = DCO, ACLK = BCLK = LFXTCLK
		  CSCTL1 = DIVS_0 | DIVA_0 | DIVHS_0 | DIVM_0 | SELA_0 | SELS_3 | SELM_3;
		break;
	}

  // Reduce drive strength to crystal
  CSCTL2 &= ~LFXTDRIVE_6;

  // Lock CS module
  CSKEY = 0;

#if 1
  // Output ACLK HSMCLK MCLK clock signals for testing
  P4DIR |= BIT2 | BIT3 | BIT4;
  P4SEL0 |= BIT2 | BIT3 | BIT4;
  P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

  return ucRetCode;
}
//! @}
