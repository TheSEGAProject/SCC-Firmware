/*
 * uart.h
 *
 *  Created on: Feb 15, 2016
 *      Author: jdk85
 */

#ifndef UART_H_
#define UART_H_

	void vUART_Init(void);
	void vUARTCOM_TXString( char*, int);
	void vUARTCOM_TXStringNoLen(char*);
	void vUART_PrintADC(void);


#endif /* UART_H_ */
