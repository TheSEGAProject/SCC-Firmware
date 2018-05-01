///////////////////////////////////////////////////////////////////////////////
//! \file comm.h
//! \brief Header file for the software UART module
//!
//! This file provides all of the defines and function prototypes for the
//! \ref comm Module.
//!
//! @addtogroup core
//! @{
//!
//! @addtogroup comm Software UART
//! The software UART module allows the user to define a UART interface on
//! any two GPIO pins, provided that the RX pin is interrupt capable. The
//! module requires the use of TimerA.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wireless Networks Research Lab
//     Dept. of Electrical Engineering, CEFNS
//     Northern Arizona University
//
//*****************************************************************************

#ifndef COMM_H_
#define COMM_H_

//******************  Pin Configurations  ***********************************//

// Status Flags
//! \name Status Flags
//! These are bit defines that are used to set and clear the
//! g_ucCOMM_Flags register.
//! @{
//! \def COMM_RUNNING
//! \brief Bit define - Indicates UART is running
#define COMM_RUNNING 0x01
//! \def COMM_TX_BUSY
//! \brief Bit define - Indicates TX in progress
#define COMM_TX_BUSY 0x02
//! \def COMM_RX_BUSY
//! \brief Bit define - Indicates RX in progress
#define COMM_RX_BUSY 0x04
//! \def COMM_PARITY_ERR
//! \brief Bit define - Indicates a parity bit failure
#define COMM_PARITY_ERR 0x08
//! @}


// Return codes
//! \name Return Codes
//! Possible return codes from the \ref comm functions
//! @{
//! \def COMM_OK
//! \brief Function return code
#define COMM_OK               0x00
//! \def COMM_BUFFER_UNDERFLOW
//! \brief user has tried to pull more data that is available from g_ucaRXBuffer
#define COMM_BUFFER_UNDERFLOW 0x01
//! \def COMM_BUFFER_OVERFLOW
//! \brief The RX buffer contains too many bytes
#define COMM_BUFFER_OVERFLOW 	0x02
//! \def COMM_ERROR
//! \brief Indicates a general communication error
#define COMM_ERROR						0x04
//! \def COMM_ACK_ERR
//! \brief Indicates that the ack bit was not received
#define COMM_ACK_ERR					0x10
//! @}

// Comm.c function prototypes
//! @name Control Functions
//! These functions handle controlling the \ref comm Module.
//! @{
void vCOMM_Init(void);
void vCOMM_Shutdown(void);
uint8 ucCOMM_WaitForMessage(void);
//! @}

//! @name Transmit Functions
//! These functions transmit information on the \ref comm Module.
//! @{
uint8 ucCOMM_SendByte(uint8 ucChar);
void vCOMM_SendMessage(volatile uint8 * pBuff, uint8 ucLength);
//! @}

//! @name Receive Functions
//! These functions take information from the \ref comm Module and format
//! it appropriately.
//! @{
uint8 ucCOMM_WaitForStartCondition(void);
uint8 ucCOMM_ReceiveByte(void);
uint8 ucCOMM_GrabMessageFromBuffer(volatile uint8 * pucBuff);
//! @}

//! @name Interrupt Handlers
//! These are the interrupt handlers used by the \ref comm Module.
//! @{
__interrupt void PORT2_ISR(void);
__interrupt void TIMERA0_ISR(void);
//! @}

#endif /*COMM_H_*/
//! @}
//! @}
