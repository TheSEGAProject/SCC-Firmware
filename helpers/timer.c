/*
 * timer.c
 *
 *  Created on: Feb 25, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "timer.h"


void vINIT_TIMER_A0(void){

	 TA0CCTL0 = OUTMOD_4 + CCIE;                // TA0CCR0 toggle, interrupt enabled
	 TA0CTL = TASSEL_1 + MC_2 + TAIE;           // ACLK, cont mode, interrupt enabled


	 __enable_interrupt();
	 NVIC_ISER0 |= 1 << ((INT_TA0_0 - 16) & 31);
}


