/*
 * led.c
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */
#include "msp.h"
#include "led.h"

unsigned char ucIndex;


void vLED_Init(void){
	P8DIR |= BIT7;                          // P8.7 set as output
}

void vLED_Shutdown(void){
	P8DIR &= ~BIT7;                          // Shutdown P8.7
}

void vLED_Blink(void){
	P8OUT &= ~BIT7; //Turn off LED

	//Blink the LED twice
	for(ucIndex = 0; ucIndex < 4; ucIndex++){
		P8OUT ^= BIT7;
		__delay_cycles(0x1FFF);
	}
}

void vLED_Quick_Blink(void){
	P8OUT &= ~BIT7; //Turn off LED

	//Blink the LED twice
	for(ucIndex = 0; ucIndex < 4; ucIndex++){
		P8OUT ^= BIT7;
		__delay_cycles(0x0FFF);
	}

}

void vLED_Blink_Delay(unsigned char blinks, unsigned char delay){
	P8OUT &= ~BIT7; //Turn off LED

	//Blink the LED twice
	for(ucIndex = 0; ucIndex < blinks; ucIndex++){
		P8OUT ^= BIT7;
		if(delay == SHORT_DELAY){
			__delay_cycles(0x1FFF);
		}
		else if(delay == LONG_DELAY){
			__delay_cycles(0xFFFF);
		}
		else{
			__delay_cycles(0x7FFF);
		}
	}

	P8OUT &= ~BIT7; //Turn off LED
}


