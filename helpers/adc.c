/*
 * adc.c
 *
 *  Created on: Feb 15, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "adc.h"

volatile float Battery_ADC[Num_of_Results];
volatile float Solar_V_ADC[Num_of_Results];
volatile float Solar_I_ADC[Num_of_Results];

volatile float fBatteryVoltage;
volatile float fSolarVoltage;

volatile float fCurrentCurrent;
volatile float fCurrentVoltage;
volatile float fAverageCurrent;
volatile float fAveragePower;
volatile float fAverageVoltage;
volatile float fVsense;


volatile uint8 index;
volatile uint8 ucADC_COMPLETE;
volatile uint8 ucFaultDetected = 0x00;

void vADC_Init(void){

	//Configure reference voltage
	while(REFCTL0 & REFGENBUSY); //Wait for ref to be ready
	REFCTL0 |= REFVSEL_3 | REFON; //2.5V and turn on reference

	__delay_cycles(300); //Let ref settle (~75uS)


	// Configure GPIO
	P5SEL1 |= BIT4 | BIT3 | BIT0;       // Enable A/D channel A1,A2,A5
	P5SEL0 |= BIT4 | BIT3 | BIT0;




	P5DIR |= BIT5 | BIT1; //Set P5.1 and P5.5 to output
	P5OUT |= BIT5 | BIT1; //Turn on MAX9934 and Battery ADC

	__enable_interrupt();
	NVIC_ISER0 = 1 << ((INT_ADC14 - 16) & 31);// Enable ADC interrupt in NVIC module




	ADC14CTL0 = ADC14ON |  ADC14MSC | ADC14SHT0__192 | ADC14SHP | ADC14CONSEQ_3; // Turn on ADC14, extend sampling time to avoid overflow of results
	ADC14CTL1 = ADC14RES_3; //14 bit resolution

	ADC14MCTL1 = ADC14VRSEL_1 | ADC14INCH_1;				// ref+=VREF, channel = A1, ADC = Battery_ADC
	ADC14MCTL2 = ADC14VRSEL_1 | ADC14INCH_2;				// ref+=VREF, channel = A2, ADC = SolarVin_ADC
	ADC14MCTL5 = ADC14VRSEL_1 | ADC14INCH_5 | ADC14EOS;		// ref+=VREF, channel = A5, end seq, ADC = SolarIin_ADC
	ADC14IER0 = ADC14IE5;                     // Enable ADC14IFG.5
	__delay_cycles(600);//Give MAX9934 time to settle (~100uS)


}

void vADC_Shutdown(void){

	P5OUT &= ~(BIT5 | BIT1); //Turn off MAX9934 and Battery ADC

	// Configure GPIO
	P5SEL1 &= ~(BIT4 | BIT3 | BIT0);       // Disable A/D channel A5
	P5SEL0 &= ~(BIT4 | BIT3 | BIT0);

	//Configure reference voltage
	REFCTL0 &= ~REFON; //Turn off reference

	ADC14CTL0 &= ~(ADC14ON | ADC14ENC | ADC14SC); // Turn OFF ADC14
	ADC14IER0 &= ~ADC14IE5;  // Disable ADC14IFG.5
}

uint8 uiADC_Read(uint8 *ucParam){
	ADC14CTL0 |= ADC14ENC | ADC14SC;        // Start conversion-software trigger

	while(!ucADC_COMPLETE)

	vADC_Calculate();

	if(fSolarVoltage > 0.25 && fAverageCurrent < 10.0 && fAverageVoltage < 0.25 ){
		ucFaultDetected = 0x01;


		__delay_cycles(4800000);
		vADC_Init();
		ucADC_COMPLETE = 0x00;//Set adc complete flag to 0
		ADC14CTL0 |= ADC14ENC | ADC14SC;        // Start conversion-software trigger

		while(!ucADC_COMPLETE)

		vADC_Calculate();

	}

	return 0;







}



void vADC_Calculate(){
	unsigned char j;
	//Reset current and average values
	fBatteryVoltage = 0;
	fSolarVoltage = 0;
	fAverageVoltage = 0;
	fAverageCurrent = 0;
	fAveragePower = 0;


	//Convert the battery ADC
	for(j = 0; j < Num_of_Results; j++)
	{
		fBatteryVoltage += Battery_ADC[j];
		fSolarVoltage += Solar_V_ADC[j];
		fCurrentVoltage = Solar_I_ADC[j];
		fCurrentCurrent = fCurrentVoltage/11.25;
		fAverageVoltage += fCurrentVoltage;
		fAverageCurrent += fCurrentCurrent;
		fAveragePower += fCurrentVoltage*fCurrentCurrent;

	}


	fBatteryVoltage = fBatteryVoltage/Num_of_Results;
	fSolarVoltage = fSolarVoltage/Num_of_Results;
	fAverageVoltage = fAverageVoltage/Num_of_Results;
	fAverageCurrent = fAverageCurrent/Num_of_Results;
	fAveragePower = fAveragePower/Num_of_Results;

}























