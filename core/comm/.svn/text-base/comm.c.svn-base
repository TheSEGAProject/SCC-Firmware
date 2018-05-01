///////////////////////////////////////////////////////////////////////////////
//! \file comm.c
//! \brief This modules implements a software UART on any two Digital I/O pins
//!
//! This module uses TimerA to implement a software UART on any two defined
//! digital I/O pin. Note that the RX pin must also be interrupt capable.
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
//     Wirless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//*****************************************************************************

#include <msp430x23x.h>
#include "../core.h"

//******************  Pin Configurations  ***********************************//
// Defines for the TX pin (currently 1.1)
//! @name Pin Defines
//! These defines are used so that the software has the right hardware
//! registers for the TX and RX pins.
//! @{
//! \def TX_PIN
//! \brief The pin number of the TX pin (BIT0 to BIT7)
#define TX_PIN          BIT1
//! \def P_TX_OUT
//! \brief The PxOUT register that the TX pin is on
#define P_TX_OUT        P1OUT
//! \def P_TX_DIR
//! \brief The PxDIR register that the TX pin is on
#define P_TX_DIR        P1DIR

// Defines for the RX pin (currently 2.2)
//! \def RX_PIN
//! \brief The pin number of the RX pin (BIT0 to BIT7)
#define RX_PIN          BIT2
//! \def P_RX_DIR
//! \brief The PxDIR register of the RX pin
#define P_RX_DIR        P2DIR
//! \def P_RX_IN
//! \brief The PxIN register of the RX pin
#define P_RX_IN         P2IN
//! \def P_RX_IES
//! \brief The PxIES register of the RX pin
#define P_RX_IES        P2IES
//! \def P_RX_IFG
//! \brief The PxIFG register of the RX pin
#define P_RX_IFG        P2IFG
//! \def P_RX_IE
//! \brief The PxIE register of the RX pin
#define P_RX_IE         P2IE

// Define for the Interrupt pin (currently 2.0)
//! \def INT_PIN
//! \brief The pin number of the INT pin (BIT0 to BIT7)
#define INT_PIN          BIT0
//! @}



//******************  Control and Indication Variables  *********************//
//! @name Control and Indication Variables
//! These variables are used to indicate to the system the current status
//! of the \ref comm Module and to store the baud rate timer information.
//! @{
//! \var volatile uint8 g_ucCOMM_Flags
//! \brief This 8-bit field indicates the status of the COMM module.
volatile uint8 g_ucCOMM_Flags;

//! \var uint16 g_unCOMM_BaudRateControl
//! \brief This is the value used to control the baud rate.
//!
//! This value is the number of timer ticks corresponding to one bit period
//! for the baud rate. It should be set from the \ref comm_baud
//! "Baud Rate Defines".
uint16 g_unCOMM_BaudRateControl;

//! \var uint16 g_unCOMM_BaudRateDelayControl
//! \brief This is the value used to delay from the start bit
//!
//! This value is the number of timer ticks to wait from the beginning of the
//! start bit to the middle of the first data bit. It should be set from the
//! \ref comm_baud_delay "Baud Rate Start Delays".
uint16 g_unCOMM_BaudRateDelayControl;
//! @}

//******************  RX Variables  *****************************************//
//! @name Receive Variables
//! These variables are used in the receiving of data on the \ref comm Module.
//! @{
//! \var volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE]
//! \brief The software UART RX Buffer
volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE];

//! \var volatile uint8 g_ucRXBufferIndex
//! \brief This index into g_ucaRXBuffer showing the current write position.
volatile uint8 g_ucRXBufferIndex;

//! \var uint8 g_ucRXBitsLeft
//! \brief The number of bits left to be received for the current byte.
uint8 g_ucRXBitsLeft;
//! @}

//******************  TX Variables  *****************************************//
//! @name Transmit Variables
//! There variables are used in the transmitting of data on the \ref comm
//! Module.
//! @{
//! \var uint8 g_ucTXBuffer
//! \brief The current byte being transmitted by the software.
uint8 g_ucTXBuffer;

//! \var uint8 g_ucTXBitsLeft
//! \brief The number of bits left to be transmitted for the current byte.
uint8 g_ucTXBitsLeft;
//! @}



//******************  Functions  ********************************************//
///////////////////////////////////////////////////////////////////////////////
//! \brief This sets up the hardware resources for doing software UART
//!
//! Since we are doing UART without the USCI, we use TimerA and it's interrupt
//! to control the baud rate. The TX and RX pins are completely controllable
//! at compile time. The software UART expects 1 start bit, 8 data bits and
//! 1 stop bit.
//!
//! To ensure correct operation of the software UART, the \ref comm_pins
//! "Comm Pin Defines" must be set correctly.
//!   \param ucBaud The baud rate define to use
//!   \return None
//!   \sa vCOMM_SendByte(), TIMERA0_ISR()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Init(uint16 ucBaud)
{
  // We set the directionality of the TX and RX pins based on the defines
  P_TX_DIR |= TX_PIN;
  P_RX_DIR &= ~RX_PIN;

  // Idle state for UART is high, so take the TX high
  P_TX_OUT |= TX_PIN;

  // Clear the RX buffer and reset index
  for (g_ucRXBufferIndex = 0x00;
       g_ucRXBufferIndex < RX_BUFFER_SIZE;
       g_ucRXBufferIndex++)
  {
    g_ucaRXBuffer[g_ucRXBufferIndex] = 0xFF;
  }
  g_ucRXBufferIndex = 0x00;

  // Enable the falling edge interrupt on RX to see start bits
  P_RX_IES |= RX_PIN;
  P_RX_IFG &= ~RX_PIN;
  P_RX_IE  |= RX_PIN;

  // BUG FIX: Clear TACTL in case someone was using it before us
  TACTL = 0x0000;

  // Hold TimerA in reset
  TACTL &= ~(MC0 | MC1);

  // Use the SMCLK, enable CCR0 interrupt
  TACTL |= TASSEL_2;
  TACCTL0 |= CCIE;

  // The timer interrupt controls the baud rate, currently configured for a
  // 4 MHz SMCLK
  g_unCOMM_BaudRateControl = ucBaud;
  switch(g_unCOMM_BaudRateControl)
  {
  	case BAUD_1200:
      g_unCOMM_BaudRateDelayControl = BAUD_1200_DELAY;
      break;

    case BAUD_9600:
      g_unCOMM_BaudRateDelayControl = BAUD_9600_DELAY;
      break;

    case BAUD_19200:
      g_unCOMM_BaudRateDelayControl = BAUD_19200_DELAY;
      break;

    case BAUD_57600:
      g_unCOMM_BaudRateDelayControl = BAUD_57600_DELAY;
      break;

    case BAUD_115200:
      g_unCOMM_BaudRateDelayControl = BAUD_115200_DELAY;
      break;

	case BAUD_230400:
      g_unCOMM_BaudRateDelayControl = BAUD_230400_DELAY;
      break;

  	case BAUD_345600:
      g_unCOMM_BaudRateDelayControl = BAUD_345600_DELAY;
      break;

  	case BAUD_460800:
      g_unCOMM_BaudRateDelayControl = BAUD_460800_DELAY;
      break;

    default:
      g_unCOMM_BaudRateControl = BAUD_9600;
      g_unCOMM_BaudRateDelayControl = BAUD_9600_DELAY;
      break;
  }

  TACCR0 = g_unCOMM_BaudRateControl;

  g_ucCOMM_Flags = COMM_RUNNING;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a byte via the software UART
//!
//! This function pushes \e ucChar into the global TX buffer, where the
//! TimerA ISR can access it. The system drops into LPM0, keeping the SMCLK
//! alive for TimerA. The ISR handles the start and stop bits as well as the
//! baud rate. This is a blocking call and will not return until the software
//! has sent the entire message.
//!   \param ucChar The 8-bit value to send
//!   \return None
//!   \sa TIMERA0_ISR(), vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_SendByte(uint8 ucChar)
{
  // If we are already busy, return so as not to screw it up
  if(g_ucCOMM_Flags & COMM_TX_BUSY)
    return;

  // Indicate in the status register that we are now busy
  g_ucCOMM_Flags |= COMM_TX_BUSY;

  // Push the value to the global TX buffer
  g_ucTXBuffer = ucChar;

  // Reset the bit count so the ISR knows how many bits left to send
  g_ucTXBitsLeft = 0x0A;

  TA0CCR0 = g_unCOMM_BaudRateControl;
  // Starts the counter in 'Up-Mode'
  TACTL |= TACLR | MC_1;

  // Shutoff the MCLK, execution returns here after TX is done
  __bis_SR_register(GIE + LPM0_bits);

  // Stop the timer and show we are done
  TACTL &= ~(MC0 | MC1 | TAIFG);
  g_ucCOMM_Flags &= ~COMM_TX_BUSY;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Shuts off the software modules
//!
//! This shuts down TimerA and disables all of the interrupts used
//!   \param None
//!   \return None
//!   \sa vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Shutdown(void)
{
  // Halt timer and clear interrupt enables
  TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);
  TACCTL0 &= ~CCIE;

  // Disable RX interrupt
  P_RX_IE &= ~RX_PIN;
  g_ucCOMM_Flags &= ~COMM_RUNNING;

  //Let TX drop
  P_TX_OUT &= ~TX_PIN;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a data message
//!
//! This function waits for a data message to be recieved on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a data message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_WaitFor32BitDataMessage(void)
{
  //__bis_SR_register(LPM3_bits); //CPU asleep and all clocks except ACLK asleep.
  //ACLK must stay on because valves may need it. If either UART has a byte or
  //we get a start bit from the CP-board, we leave this and check if we have
  // a complete message from CP. Then it waits for complete message (needs LPM0
  //so we can receive message from CP-board). I wanted the UART to be checked
  //inside this while loop in case a start bit comes while UART is being handled.
  //(Danger of re-entering LMP3 while SMCLK timer is needed)
  //A better code could be written with flags etc, but UART is only used in test
  //environment so this is not OP critical.

  while (g_ucRXBufferIndex != SP_32BITDATAMESSAGE_SIZE){
	  //UARTDELETE
	  if(ucUARTCOM_getBufferFill() &&ucUARTCOM_LastIsReturn()){//Can comment this out when final code published.
		  vUARTCOM_HandleUART();
	  }
	  __bis_SR_register(LPM0_bits); //CPU asleep.
  }
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a data message
//!
//! This function waits for a data message to be recieved on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a data message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_WaitFor128BitDataMessage(void)
{


  while (g_ucRXBufferIndex != SP_128BITDATAMESSAGE_SIZE){
	  __bis_SR_register(LPM0_bits); //CPU asleep.
  }
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a label message
//!
//! This function waits for a label message to be received on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a label message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_WaitForLabelMessage(void)
{
  while (g_ucRXBufferIndex != SP_LABELMESSAGE_SIZE){



  }
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data message on the serial port
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////.
void vCOMM_Send32BitDataMessage(union SP_32BitDataMessage * p_32BitDataMessage)
{
  uint8 ucLoopCount;

  for (ucLoopCount = 0x00;
       ucLoopCount < SP_32BITDATAMESSAGE_SIZE;
       ucLoopCount++)
    vCOMM_SendByte(p_32BitDataMessage->ucByteStream[ucLoopCount]);
}

#if SP_PACKET_SIZE_128
///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data return message on the serial port
//! Written by -scb
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////

void vCOMM_Send128BitDataMessage(union SP_128BitDataMessage * p_128BitDataMessage)
{
  uint8 ucLoopCount;
  for (ucLoopCount = 0x00;
       ucLoopCount < SP_128BITDATAMESSAGE_SIZE;//size is 160 bit (128 + 32)
       ucLoopCount++)
    vCOMM_SendByte(p_128BitDataMessage->ucByteStream[ucLoopCount]);
}
#endif
///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a label message on the serial port
//!
//! This function sends the label message pointed to by \e p_LabelMessage on
//! the software UART line
//!   \param p_LabelMessage Pointer to the message to send
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_SendLabelMessage(union SP_LabelMessage * p_LabelMessage)
{
  uint8 ucLoopCount;

  for (ucLoopCount = 0x00;
       ucLoopCount < SP_LABELMESSAGE_SIZE;
       ucLoopCount++)
    vCOMM_SendByte(p_LabelMessage->ucByteStream[ucLoopCount]);
}


///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a data message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the data message pointed to by \e message.
//!   \param message Pointer to the message
//!   \return The error code indicating the status after call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_Grab32BitDataMessageFromBuffer(union SP_32BitDataMessage * message)
{
  uint8 ucLoopCount;

  if (g_ucRXBufferIndex < SP_32BITDATAMESSAGE_SIZE)
    return COMM_BUFFER_UNDERFLOW;

  for (ucLoopCount = 0x00;
       ucLoopCount < SP_32BITDATAMESSAGE_SIZE;
       ucLoopCount++)
    message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

  g_ucRXBufferIndex = 0x00;

  return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a data message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the data message pointed to by \e message.
//!   \param message Pointer to the message
//!   \return The error code indicating the status after call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_Grab128BitDataMessageFromBuffer(union SP_128BitDataMessage * message)
{
  uint8 ucLoopCount;

  if (g_ucRXBufferIndex < SP_128BITDATAMESSAGE_SIZE)
    return COMM_BUFFER_UNDERFLOW;

  for (ucLoopCount = 0x00;
       ucLoopCount < SP_128BITDATAMESSAGE_SIZE;
       ucLoopCount++)
    message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

  g_ucRXBufferIndex = 0x00;

  return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a label message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the label message pointed to by \e message.
//!   \param message Pointer to the label message
//!   \return The error code indicatin the status of the call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_GrabLabelMessageFromBuffer(union SP_LabelMessage * message)
{
  uint8 ucLoopCount;

  if (g_ucRXBufferIndex < SP_LABELMESSAGE_SIZE)
    return COMM_BUFFER_UNDERFLOW;

  for (ucLoopCount = 0x00;
       ucLoopCount < SP_LABELMESSAGE_SIZE;
       ucLoopCount++)
    message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

  g_ucRXBufferIndex = 0x00;

  return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief TimerA0 ISR, sends and receives on the software UART lines
//!
//! TimerA has been configured by \e vCOMM_Init() to generate an interrupt
//! for the bit timing on a specific baud rate. Each time it is called, the
//! ISR checks to see which bit it is sending, control or data. Then the
//! appropriate bits are calculated and sent. If there are no more bits to
//! send, then the ISR returns the system to active mode (AM).
//!
//! This ISR handles the timing for both TX and RX. Therefore, it is best to
//! use this module in half-duplex mode only. Other wise you risk the
//! possiblity of having a very long ISR, which will kill the delicate UART
//! timing.
//!
//! NOTE: Traditionally an ISR is only used to changed state and not for
//! for making decisions and changing pins. However, because we are doing
//! software UART and time was imortant, I wanted to avoid the latency
//! associated with moving into AND out of an ISR. For future improvements,
//! see the TODO below.
//!
//! TODO: Move the bulk of this code and decision making into the SendByte
//! routine, only have the ISR perform MCU wake up and state switching for
//! TX. The RX code has to remain as this is an asynchronous protocol and
//! packets can come in at any time. These changes may require tweaking of
//! the UART delay values to ensure that the space and mark timing is
//! correct.
//!
//! NOTE: This interrupt is ONLY for handling CCR0, allowing the user to use
//! the TIMERA1 interrupt for other uses of TA CCR1, CCR2 and TAIFG
//!   \param None
//!   \return None
//!   \sa vCOMM_Init(), vCOMM_SendByte()
///////////////////////////////////////////////////////////////////////////////
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA0_ISR(void)
{
  if (g_ucCOMM_Flags & COMM_TX_BUSY)
  {
    switch(g_ucTXBitsLeft)
    {
      case 0x00:
        // If there are no bits left, then return to function call
        __bic_SR_register_on_exit(LPM0_bits);
        break;

      case 0x01:
        // Last bit is stop bit, return to idle state
        P_TX_OUT |= TX_PIN;
        __bic_SR_register_on_exit(LPM0_bits);
        break;

      case 0x0A:
        // First bit is start bit
        P_TX_OUT &= ~TX_PIN;
        break;

      default:
        // For data bits, mask to get correct value and the shift for next time
        if (g_ucTXBuffer & 0x01)
          P_TX_OUT |= TX_PIN;
        else
          P_TX_OUT &= ~TX_PIN;

        g_ucTXBuffer >>= 1;
        break;
    }

    // Decrement the total bit count
    g_ucTXBitsLeft--;
  }

  if (g_ucCOMM_Flags & COMM_RX_BUSY)
  {
    switch(g_ucRXBitsLeft)
    {
      case 0x00:
        // There are no bits left, so lets reset all the values and stop timer
        TACTL &= ~(MC0 | MC1);
        P_RX_IE |= RX_PIN;
        P_RX_IFG &= ~RX_PIN;
        g_ucRXBufferIndex++;
        g_ucCOMM_Flags &= ~COMM_RX_BUSY;
        //Set All Clocks and CPU etc awake now that we received a byte. (check if it's last later)
        //If it is not the last Byte, the core will put us back into LPM0, which won't stop the clocks, just the CPU
        //This does not have to be done here, since we unset the RX flag, the later function will do this for us.
        break;

      case 0x01:
        if (P_RX_IN & RX_PIN)
          g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
        else
          g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;

        break;

      default:
        if (P_RX_IN & RX_PIN)
          g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
        else
          g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;

        g_ucaRXBuffer[g_ucRXBufferIndex] >>= 1;
        break;
    }
    g_ucRXBitsLeft--;
  }

  if(!(g_ucCOMM_Flags & COMM_RX_BUSY) && !(g_ucCOMM_Flags & COMM_TX_BUSY))
  {
	  __bic_SR_register_on_exit(LPM4_bits); //Exit LPM if it was neither RX nor TX
  }

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Port2 ISR, handles the start of the UART RX
//!
//! The idle state for UART is line high, thus when we get the falling edge
//! indicating the start bit, we can start the timer to handle the UART
//! sampling.
//!   \param None
//!   \return None
//!   \sa vCOMM_Init(), TIMERA0_ISR
///////////////////////////////////////////////////////////////////////////////
/*
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{

    // If we get an interrupt from the RX pin, then we have RXed a start bit

	if(P_RX_IFG & RX_PIN)//P2IFG & BIT2
	{
      // Delay for half bit, this ensures we start sampling at the middle of each bit

      TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);//Halt timer, Disable interrupts
      TACTL |= TACLR;//Clear Timer
      //TACCTL0 &= ~CCIE;//Disable Compare interrupt
      TACCTL0 |= CCIE;//Enable Compare interrupt
      TACCR0 = g_unCOMM_BaudRateDelayControl; //Input delay time in compare register
      TACTL |= MC_1;//Start timer in UP Mode (Counts to TACCR0 then starts at 0 again)
      g_ucCOMM_Flags |= COMM_RX_SETUP;


      __bic_SR_register_on_exit(LPM4_bits); //All Clocks and CPU etc awake now that we received a start bit.
      //It will reset to LPM3 but in that LMP the timer can still run.


      //while(!(TACTL & TAIFG)); //Wait until Compare register is met

     //TACTL &= ~TAIFG;


     // Enable timer interrupt, configure for baud rate
     //TACTL &= ~(MC0 | MC1);
     //TACTL |= TACLR;
     //TACCTL0 |= CCIE;
     //TACCR0 = g_unCOMM_BaudRateControl;


     // Disable interrupt on RX
     P_RX_IE &= ~RX_PIN;
     //g_ucCOMM_Flags |= COMM_RX_BUSY;
     //g_ucRXBitsLeft = 0x08;
     //TACTL |= MC_1;

     //Clear Interrupt Flag
     P_RX_IFG &= ~RX_PIN;

  }
}*/

// Original for BACKUP

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{

   // Determine the source of the interrupt
   // If we get an interrupt from the RX pin, then we have RXed a start bit
   //case RX_PIN:
   if(P_RX_IFG & RX_PIN)//P2IFG & BIT2
   {
	  if(!(RX_PIN & P_RX_IN))
	  {
      // Delay for half bit, this ensures we start sampling at the middle of
      // each bit
      TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);//Halt timer, Disable interrupts
      TACTL |= TACLR;//Clear Timer
      TACCTL0 &= ~CCIE;//Disable Compare interrupt
      TACCR0 = g_unCOMM_BaudRateDelayControl; //Input delay time in compare register
      TACTL |= MC_1;//Start timer in UP Mode (Counts to TACCR0 then starts at 0 again)

      while(!(TACTL & TAIFG)); //Wait until Compare register is met
      TACTL &= ~TAIFG;

      // Enable timer interrupt, configure for baud rate
      TACTL &= ~(MC0 | MC1);
      TACTL |= TACLR;
      TACCTL0 |= CCIE;
      TACCR0 = g_unCOMM_BaudRateControl;

      // Disable interrupt on RX
      P_RX_IE &= ~RX_PIN;
      g_ucCOMM_Flags |= COMM_RX_BUSY;
      g_ucRXBitsLeft = 0x08;
      TACTL |= MC_1;
	  }

      //Clear Interrupt Flag
      P_RX_IFG &= ~RX_PIN;

      __bic_SR_register_on_exit(LPM4_bits); //All Clocks and CPU etc awake now that we received a start bit.

   }//END if(P_RX_IFG & RX_PIN)//P2IFG & BIT2


   if(P_RX_IFG & INT_PIN)//P2IFG & BIT0
   {
	   __bic_SR_register_on_exit(LPM4_bits); //All Clocks and CPU etc awake if interrupt called.
   }//END if(P_RX_IFG & INT_PIN)

}//END __interrupt void PORT2_ISR(void)

//! @}
//! @}
















