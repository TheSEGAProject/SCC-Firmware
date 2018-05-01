/*
 * adc.h
 *
 *  Created on: Feb 15, 2016
 *      Author: jdk85
 */

#ifndef ADC_H_
#define ADC_H_
#include "../core/core.h"



#define Num_of_Results   10


extern volatile uint8 ucADC_COMPLETE;
extern volatile float Battery_ADC[Num_of_Results];
extern volatile float Solar_V_ADC[Num_of_Results];
extern volatile float Solar_I_ADC[Num_of_Results];

extern volatile unsigned char index;


void vADC_Init(void);
void vADC_Shutdown(void);
void vADC_Calculate(void);

uint8 uiADC_Read(uint8 *ucParam);


#endif /* ADC_H_ */
