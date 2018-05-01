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

  //! \def RX_BUFFER_SIZE
  //! \brief The number of bytes to allocate for the UART RX buffer
  #define RX_BUFFER_SIZE 0x20

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

  //! @}

  // Baud rate defines
  //! @name Baud Rate Defines
  //! These values are computed for a 4MHz SMCLK to produce the corresponding
  //! baud rates
  //! @{
  //! \def BAUD_460800
  //! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_460800 0x0008
  //! \def BAUD_345600
  //! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_345600 0x000B
  //! \def BAUD_230400
  //! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_230400 0x0011
  //! \def BAUD_115200
  //! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_115200 0x0023
  //! \def BAUD_57600
  //! Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_57600  0x0045
  //! \def BAUD_19200
  //! Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_19200  0x00D0
  //! \def BAUD_9600
  //! Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_9600   0x01A0
  //! \def BAUD_1200
  //! Timer count for specific UART data rate, computed for 4Mhz SMCLK
  #define BAUD_1200   0x0D05 //3333
  //! @}

//******************  Baud Rate Delays  *************************************//
//! @name Baud Rate Start Delays
//! These values are used to delay from the start bit to the middle of the
//! first data bit. Computed for a 4MHz SMCLK.
//! @{
//! \def BAUD_460800_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_460800_DELAY  0x0004
//! \def BAUD_345600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_345600_DELAY  0x0006
//! \def BAUD_230400_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_230400_DELAY  0x0007//Slightly low to account for the set-up cycles that we delayed. At these times that's necessary...
//! \def BAUD_115200_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_115200_DELAY  0x0010//Slightly low to account for the set-up cycles that we delayed. At these times that's necessary...
//! \def BAUD_57600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_57600_DELAY   0x0022
//! \def BAUD_19200_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_19200_DELAY   0x0068
//! \def BAUD_9600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_9600_DELAY    0x00D0
//! \def BAUD_9600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_1200_DELAY    0x0682 //1666
//! @}


  // Return codes
  //! \name Return Codes
  //! Possible return codes from the \ref comm functions
  //! @{
  //! \def COMM_BUFFER_UNDERFLOW
  //! \brief Function return code
  //!
  //! Getting this return code means that the user has tried to pull more
  //! data that is avalibe from g_ucaRXBuffer
  #define COMM_BUFFER_UNDERFLOW 0x01
  //! \def COMM_OK
  //! \brief Function return code
  #define COMM_OK               0x00
  //! @}


  // Comm.c function prototypes
  //! @name Control Functions
  //! These functions handle controlling the \ref comm Module.
  //! @{
  void vCOMM_Init(uint16 ucBaud);
  void vCOMM_Shutdown(void);
  void vCOMM_WaitFor32BitDataMessage(void);
  void vCOMM_WaitFor128BitDataMessage(void);
  void vCOMM_WaitForLabelMessage(void);
  //! @}

  //! @name Transmit Functions
  //! These functions transmit information on the \ref comm Module.
  //! @{
  void vCOMM_SendByte(uint8 ucChar);
  void vCOMM_Send32BitDataMessage(union SP_32BitDataMessage * p_DataMessage);
  void vCOMM_Send128BitDataMessage(union SP_128BitDataMessage * p_DataRetMessage);
  void vCOMM_SendLabelMessage(union SP_LabelMessage * p_LabelMessage);
  //! @}

  //! @name Receive Functions
  //! These functions take information from the \ref comm Module and format
  //! it appropriately.
  //! @{
  uint8 ucCOMM_Grab32BitDataMessageFromBuffer(union SP_32BitDataMessage * message);
  uint8 ucCOMM_Grab128BitDataMessageFromBuffer(union SP_128BitDataMessage * message);
  uint8 ucCOMM_GrabLabelMessageFromBuffer(union SP_LabelMessage * message);
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
