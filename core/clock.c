/*
 * clock.c
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "clock.h"



void vMCLK_24MHz(void){
	P4DIR |= BIT2 | BIT3;
	P4SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4SEL1 &= ~(BIT2 | BIT3);

	CSKEY = 0x695A;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters
	CSCTL0 = DCORSEL_4;                   // Set DCO to 24MHz (nominal, center of 8-16MHz range)

	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CSCTL1 = SELA_2 | SELS_3 | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses
#if 1
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}

void vMCLK_12MHz(void){
	P4DIR |= BIT2 | BIT3;
	P4SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4SEL1 &= ~(BIT2 | BIT3);

	CSKEY = 0x695A;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters
	CSCTL0 = DCOEN | DCORSEL_3;                   // Set DCO to 12MHz (nominal, center of 8-16MHz range)

	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CSCTL1 = SELA_2 | SELS_3 | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}
void vMCLK_3MHz(void){
	P4DIR |= BIT2 | BIT3;
	P4SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4SEL1 &= ~(BIT2 | BIT3);


	CSKEY = CSKEY_VAL;                        // Unlock CS module for register access
	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CSCTL1 = SELA_2 | SELS_3 | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}
void vMCLK_16MHzCAL(void){

	int32_t nomFreq, calVal, dcoSigned;
	int16_t dcoTune;
	float dcoConst;

	dcoSigned = (int32_t) 16000000;
	nomFreq = (int32_t) 24000000;

	dcoConst = *((float *) &TLV->rDCOER_CONSTK_RSEL04);
	calVal = TLV->rDCOER_FCAL_RSEL04;


	dcoTune = (int16_t) (((dcoSigned - nomFreq)
			* (1.0 + dcoConst * (768.0 - calVal)) * 8.0)
			/ (dcoSigned * dcoConst));


	if (dcoTune < 0)
	{
		dcoTune = (dcoTune & DCOTUNE_M) | 0x1000;
	}
	else
	{
		dcoTune &= DCOTUNE_M;
	}

//	dcoTune = 5875;//TODO: This is hardcoded to actually get 16MHz on SCC with REV C MSP432

	PJSEL0 |= BIT0 | BIT1;                    // set LFXT pin as second function


	CSKEY = 0x695A;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters

	CSCTL0 = DCOEN | DCORES | DCORSEL_4 | dcoTune;                   // Set DCO to 24MHz (nominal, center of 16-32MHz range, tune to 16MHz)

	CSCTL2 |= LFXT_EN;                         // LFXT on
	CSCLKEN = SMCLK_EN | MCLK_EN | HSMCLK_EN | ACLK_EN;


	do
	{
		// Clear XT2,XT1,DCO fault flags
		CSCLRIFG |= CLR_DCORIFG | CLR_HFXTIFG | CLR_LFXTIFG;
		SYSCTL_NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
	} while (SYSCTL_NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);// Test oscillator fault flag

	// Select ACLK = LFXT, SMCLK = MCLK = DCO/3
	CSCTL1 = DIVS__4 | DIVHS__4 | DIVA_0 | DIVM_0 | DIVHS_0 | SELA__LFXTCLK | SELS_3 | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses

#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}
void vMCLK_16MHz(void){

	unsigned char ucDCO = 16;
	unsigned char ucNominalFreq = 24;
	float * pulDCO_ER_RSEL04_K = (float*)0x00201084L;
	unsigned long * pulER_Cal = (unsigned long *)0x0020106CL;

	float fDCOConstant = *pulDCO_ER_RSEL04_K;
	unsigned long ulDCOCal = *pulER_Cal;


	float fTempNum = (ucDCO - ucNominalFreq)*(((fDCOConstant * (768 - ulDCOCal)) + 1)*8);
	float fTempDen = (fDCOConstant * ucDCO);


	signed long lDCOTuneBits = fTempNum/fTempDen;
	signed int iDCOTune;
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

	PJSEL0 |= BIT0 | BIT1;                    // set LFXT pin as second function


	CSKEY = 0x695A;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters

	/**
	 * NOTE: TODO: The DCORES external selection is unstable in the current release,
	 */
	//	CSCTL0 = DCOEN | DCORES | DCORSEL_3 | iDCOTune;                   // Set DCO to 12MHz (nominal, center of 8-16MHz range)
	CSCTL0 = DCOEN  | DCORSEL_4 | iDCOTune;
	CSCTL2 |= LFXT_EN;                         // LFXT on
	CSCLKEN = SMCLK_EN | MCLK_EN | HSMCLK_EN | ACLK_EN;
	do
	{
		// Clear XT2,XT1,DCO fault flags
		CSCLRIFG |= CLR_DCORIFG | CLR_HFXTIFG | CLR_LFXTIFG;
		SYSCTL_NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
	} while (SYSCTL_NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);// Test oscillator fault flag

	// Select ACLK = LFXT, SMCLK = MCLK = DCO/3
	CSCTL1 = DIVS__4 | DIVHS__4 | DIVA_0 | DIVM_0 | DIVHS_0 | SELA__LFXTCLK | SELS_3 | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses

#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}

void vMCLK_48MHz(void){
	PJSEL0 |= BIT2 | BIT3;                  // Configure PJ.2/3 for HFXT function
	PJSEL1 &= ~(BIT2 | BIT3);

	CSKEY = CSKEY_VAL;                      // Unlock CS module for register access
	CSCTL2 |= HFXT_EN | HFXTFREQ_6 | HFXTDRIVE; //Enable HFXT, set freq range to 40-48MHz, drive when HFXTFREQ > 4MHz

	//Wait for both clock and HFXT interrupt flags to be set
	while(CSIFG & HFXTIFG)
		CSCLRIFG |= CLR_HFXTIFG; //Clear the HFXT interrupt

	// Select MCLK & HSMCLK = HFXT, no divider
	//CSCTL1 = CSCTL1 & ~(SELM_M | DIVM_M | SELS_M | DIVHS_M) | SELM__HFXTCLK | SELS__HFXTCLK;
	//Set MCLK = HFXT, SMCLK=MCLK/4
	CSCTL1 = DIVS_2 | SELM_5 | SELS_5;
	CSKEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4DIR |= BIT2 | BIT3 | BIT4;
	P4SEL0 |= BIT2 | BIT3 | BIT4;
	P4SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}

