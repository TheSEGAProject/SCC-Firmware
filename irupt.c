/*
 * irupt.c
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */




#include "msp.h"
#include "core/core.h"
#include "helpers/adc.h"
#include "helpers/led.h"

extern volatile uint8 g_ucaRXBuffer[MAXMSGLEN];
extern volatile uint8 g_ucRXBufferIndex;
extern volatile uint8 ucADC_COMPLETE;

uint8 ucSecondCounter = 1;

// Timer A0 interrupt service routine
void TimerA0_0IsrHandler(void)
{

	TA0CCTL0 &= ~CCIFG;
	if(ucSecondCounter == 10){
		//Pet the dog
		WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS_3;
		P2OUT ^= BIT4;
		ucSecondCounter = 1;
	}
	else{
		ucSecondCounter++;
	}



}

// UART interrupt service routine
void eUSCIA0IsrHandler(void)
{
	if (UCA0IFG & UCRXIFG)
	{
		g_ucaRXBuffer[g_ucRXBufferIndex] = UCA0RXBUF;
		g_ucRXBufferIndex++; // Increment index for next byte
		if(g_ucRXBufferIndex >= 64){
			g_ucRXBufferIndex = 0;
		}

	}
}



// ADC14 interrupt service routine
void ADC14IsrHandler(void)
{

	if (ADC14IFGR0 & ADC14IFG5)
	{
		Battery_ADC[index] = ADC14MEM1 * 0.0002670288;
		Solar_V_ADC[index] = ADC14MEM2 * 0.0001525879f;
		Solar_I_ADC[index] = ADC14MEM5 * 0.0001525879f;

		if(index == (Num_of_Results - 1))
		{
			index = 0;
			//Stop the conversion
			ADC14CTL0 &= ~ADC14ENC & ~ADC14SC;
			//Shutdown the ADC
			vADC_Shutdown();
			//Alert that ADC readings are fully complete
			ucADC_COMPLETE = 0x01;

		}
		else
		{
			index++;
		}


		//Clear ADC flag
		ADC14CLRIFGR0 = CLRADC14IFG5;
	}

}
