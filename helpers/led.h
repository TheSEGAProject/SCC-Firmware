/*
 * led.h
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */

#ifndef LED_H_INCLUDED
#define LED_H_INCLUDED

	#define SHORT_DELAY	0x00
	#define LONG_DELAY	0x01

	void vLED_Init(void);
	void vLED_Shutdown(void);
	void vLED_Blink(void);
	void vLED_Quick_Blink(void);
	void vLED_Blink_Delay(unsigned char blinks, unsigned char delay);



#endif /* LED_H_ */
