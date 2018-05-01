///////////////////////////////////////////////////////////////////////////////
//! \file core.c
//! \brief This is the primary file for the SP Board core
//!
//! This file contains the heart of the SP Board core. The function pointer
//! and label tables are kept here and maintained by the core.
//!
//! @addtogroup core Core
//! The Core Module handles all of the communication to the CP board as well
//! as acts as the supervisor to all activities on the SP Board. The user
//! built wrapper should interface with the Core Module as documented.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wireless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//
//	Modified by: Chris Porter
//
//*****************************************************************************

#include "msp432.h"
#include "core.h"
#include "clock.h"
#include "comm/crc.h"
#include "comm/msg.h"


//******************  Software version variables  ***************************//
//! @name Software Version Variables
//! These variables contain ID and version information.
//! @{
//! \var static const uint8 g_ucaCoreVersion[VERSION_LABEL_LEN]
//! \brief The name and version of the core
#define VERSION_LABEL "SCC-Core v1.00   "
//! @}

//! \var uiHID
//! \brief Variable holds the unique SCC ID as a byte array
uint16 uiHID[4];

extern volatile uint8 g_ucaRXBuffer[MAXMSGLEN];
extern volatile uint8 g_ucRXBufferIndex;
extern volatile float fBatteryVoltage;
extern volatile float fSolarVoltage;
extern volatile float fAverageCurrent;
extern volatile float fAverageVoltage;
extern volatile uint8 ucADC_COMPLETE;
extern volatile uint8 ucFaultDetected;


//******************  Functions  ********************************************//
///////////////////////////////////////////////////////////////////////////////
//! \brief This function starts up the Core and configures hardware & RAM
//!
//! All of the clock setup and initial port configuration is handled in this
//! function. At the end of the function, before the return, any additional
//! core initilization functions are called.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCORE_Initialize(void)
{
	// First, stop the watchdog
	WDTCTL = WDTPW + WDTHOLD;



	// Terminate all pins on the device
	P1DIR |= 0xFF; P1OUT = 0x00;
	P2DIR |= 0xFF; P2OUT = 0x00;
	P3DIR |= 0xFF; P3OUT = 0x00;
	P4DIR |= 0xFF; P4OUT = 0x00;
	P5DIR |= 0xFF; P5OUT = 0x00;
	P6DIR |= 0xFF; P6OUT = 0x00;
	P7DIR |= 0xFF; P7OUT = 0x00;
	P8DIR |= 0xFF; P8OUT = 0x00;
	P9DIR |= 0xFF; P9OUT = 0x00;
	P10DIR |= 0xFF; P10OUT = 0x00;


	//Configure the m clock for 4MHz
	vMCLK_16MHzCAL();

	// Get the SPs serial number from flash
	//vFlash_GetHID(uiHID);
	//Hard-code uiHID for now
	uiHID[0] = 0x0A0A;
	uiHID[1] = 0x0B0B;
	uiHID[2] = 0x0C0C;
	uiHID[3] = 0x0D0D;

	//Init UART for SCC-WiSARD communication
	vCOMM_Init();

	SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;           //Resume operation after returning from interrupt


	__enable_interrupt();
	//Enable WDT interrupt
	//NVIC_ISER0 = 1 << ((INT_WDT_A - 16) & 31);
	//Enable TA0 Interrupt
	NVIC_ISER0 = 1 << ((INT_TA0_0 - 16) & 31);
	TA0CCTL0 &= ~CCIFG;
	TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
	TA0CCR0 = 32768;
	TA0CTL |= TASSEL__ACLK | MC__UP | TACLR;   // ACLK, UP mode
	/* Configure watch dog
	 * - ACLK as clock source
	 * - Watchdog mode
	 * - Clear WDT counter (initial value = 0)
	 * - Timer interval = 16s @ 32kHz ACLK
	 */
	WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS_3;




	//    /*Go to LPM3 by setting the sleep deep bit */
	//    SCB_SCR |= (SCB_SCR_SLEEPDEEP);
	//
	//    PCMCTL0 = PCM_CTL_KEY_VAL | LPMR__LPM45;


}

///////////////////////////////////////////////////////////////////////////////
//! \brief Measure the MSP430 supply voltage
//!
//! Uses the ADC12 to measure the input voltage. Uses the MEM15 register.
//!
//!   \param none
//!
//!   \return unsigned int Input voltage * 100
///////////////////////////////////////////////////////////////////////////////
/*
unsigned int unCORE_GetVoltage(void)
{
	int rt_volts;

	ADC12CTL0 &= ~(SHT10 + SHT12 + SHT13 + MSC + ADC12OVIE + ADC12TOVIE + ENC + ADC12SC); //ADC12CTL0 &= ~0xD08F = ~1101 0000 1000 1111 //Have to turn ENC Off first
	ADC12CTL0 |= (SHT11 + REF2_5V + REFON + ADC12ON); //ADC12CTL0 |= 0x2070 = 0010 xxxx 0111 00(11)* - 16-Cycle Hold time + Single Conversion + 2.5V Ref + RefON + ADC ON + Interrupts off + (Enable + Start)
	ADC12CTL1 &= ~(SHS1 + SHS0 + ISSH + ADC12DIV2 + ADC12DIV1 + ADC12DIV0 + ADC12SSEL1 + ADC12SSEL0 + CONSEQ1 + CONSEQ0); //ADC12CTL1 &= ~0x0FDE = ~0000 1101 1111 1110
	ADC12MEM15 = 0;
	ADC12MCTL15 |= (SREF0 + INCH3 + INCH1 + INCH0); //ADC12MCTL15 |= 0x1B = x001 1011 - Reference Select + Input Select
	ADC12MCTL15 &= ~(SREF2 + SREF1 + INCH2); // ADC12MCTL15 &= ~0x64 = 0110 0100
	ADC12IE &= ~0x8000; //Turn off IE and clear IFG
	ADC12IFG &= ~0x8000;

	__delay_cycles(1000);

	ADC12CTL1 |= (CSTARTADD3 + CSTARTADD2 + CSTARTADD1 + CSTARTADD0 + SHP); //ADC12CTL1 |= 0xF200 = 1111 0010 0000 000x - MEM15 + Internal OSC CLK + Single-Channel, Single-conversion
	ADC12CTL0 |= ENC + ADC12SC; // Sampling and conversion start

	while (!(ADC12IFG & 0x8000)); //End when something is written in. Can't sleep because we wanted to keep interrupts for users (not in core)

	rt_volts = ADC12MEM15; //(0.5*Vin)/2.5V * 4095
	ADC12IFG &= ~0x8000; //Unset IFG Flag
	ADC12CTL0 &= ~ENC;
	ADC12CTL0 &= ~(REFON + ADC12ON); // turn off A/D to save power

	rt_volts = (rt_volts * 5) / 41;
	return (rt_volts);
}
 */

///////////////////////////////////////////////////////////////////////////////
//! \brief Send the confirm packet
//!
//!  Confirm packet includes all the data received so CP Board can confirm it
//!  is correct.
//!
//!   \param none
//!   \sa core.h
///////////////////////////////////////////////////////////////////////////////
vCORE_Send_ConfirmPKT()
{
	uint8 ucaMsg_Buff[MAXMSGLEN];

	// Send confirm packet that we received message
	ucaMsg_Buff[MSG_TYP_IDX] = CONFIRM_COMMAND;
	ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
	ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;

	// Send the message
	vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Send the confirm packet
//!
//!  Confirm packet includes all the data received so CP Board can confirm it
//!  is correct.
//!
//!   \param none
//!   \sa core.h
///////////////////////////////////////////////////////////////////////////////
void vCORE_Send_ErrorMsg(uint8 ucErrMsg)
{
	uint8 ucaMsg_Buff[MAXMSGLEN];

	// Send confirm packet that we received message
	ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
	ucaMsg_Buff[MSG_LEN_IDX] = 4;
	ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
	ucaMsg_Buff[MSG_PAYLD_IDX] = ucErrMsg;

	// Send the message
	vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
}

///////////////////////////////////////////////////////////////////////////////
//! \brief This functions runs the core
//!
//! This function runs the core. This function does not return, so all of the
//! core setup and init must be done before the call to this function. The
//! function waits for a data packet from the CP Board, then parses and handles
//! it appropriately. A response packet is then sent and the core waits for
//! the next data packet.
//!   \param None.
//!   \return NEVER. This function never returns
//!   \sa msg.h
///////////////////////////////////////////////////////////////////////////////
void vCORE_Run(void)
{
	uint16 unTransducerReturn; //The return parameter from the transducer function
	uint8 ucaMsg_Buff[MAXMSGLEN];
	uint8 ucMsgBuffIdx;
	uint8 ucCmdTransNum;
	uint8 ucCmdParamLen;
	uint8 ucParamCount;
	uint8 ucParam[20];
	uint8 ucCommState;
	uint8 ucRXMessageSize = SP_HEADERSIZE;


	__no_operation();


	// The primary execution loop
	while(1)
	{
		//Enter LMP0 and wait for UART RX interrupt
		__sleep();
		//UART RX interrupt woke us up
		//Check if full message was received
		if(g_ucRXBufferIndex == SP_HEADERSIZE){
			ucRXMessageSize = g_ucaRXBuffer[MSG_LEN_IDX] + CRC_SZ;
			//TODO CHECK MESSAGE SIZE
			__no_operation();

		}
		else if(ucRXMessageSize != SP_HEADERSIZE && g_ucRXBufferIndex == ucRXMessageSize){
			//We RX'd the full message, get the state
			ucCommState = ucCOMM_GrabMessageFromBuffer(ucaMsg_Buff);
			// Reset the RX index
			g_ucRXBufferIndex = 0x00;
			// Pull the message from the RX buffer and load it into a local buffer
			if (ucCommState == COMM_OK)
			{
				//Switch based on the message type
				switch (ucaMsg_Buff[MSG_TYP_IDX])
				{
				//////////////////////////////////////////////////////////////////
				//
				//		COMMAND_PKT
				//
				//////////////////////////////////////////////////////////////////
				case COMMAND_PKT:

					// Send a confirmation packet
					vCORE_Send_ConfirmPKT();
					unTransducerReturn = 0; //default return value to 0

					// Read through the length of the message and execute commands as they are read
					for (ucMsgBuffIdx = MSG_PAYLD_IDX; ucMsgBuffIdx < ucaMsg_Buff[MSG_LEN_IDX];)
					{
						// Get the transducer number and the parameter length
						ucCmdTransNum = ucaMsg_Buff[ucMsgBuffIdx++];
						ucCmdParamLen = ucaMsg_Buff[ucMsgBuffIdx++];

						for (ucParamCount = 0; ucParamCount < ucCmdParamLen; ucParamCount++)
						{
							ucParam[ucParamCount] = ucaMsg_Buff[ucMsgBuffIdx++];
						}

						// Dispatch to perform the task, pass all values needed to populate the data
						unTransducerReturn |= uiMainDispatch(ucCmdTransNum, ucCmdParamLen, ucParam);
					}

					break; //END COMMAND_PKT
					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_DATA
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_DATA:
					// Stuff the header
					ucaMsg_Buff[MSG_TYP_IDX] = REPORT_DATA;
					if (ucADC_COMPLETE == 1){
						ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
						// Load the message buffer with data.  The fetch function returns length
						ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE + ucMain_FetchData(&ucaMsg_Buff[MSG_PAYLD_IDX]);

					}
					else{
						ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
						ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
					}
					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
//					if(ucFaultDetected){
//						printf("\r\n===== FAULT DETECTED =====\r\n");
//					}
					ucFaultDetected = 0x00;
//					printf("%f,",fBatteryVoltage);\
//					printf("%f,",fSolarVoltage);
//					printf("%f,",fAverageVoltage);
//					printf("%f\r\n",fAverageCurrent*1000);




					break; //END REQUEST_DATA
					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_LABEL
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_LABEL:
					//					// Format first part of return message
					//					ucaMsg_Buff[MSG_TYP_IDX] = REPORT_LABEL;
					//					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;// + TRANSDUCER_LABEL_LEN;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_LABELMESSAGE_VERSION;
					//
					//					// Make call to main for the trans. labels.  This way the core is not constrained to a fixed number of transducers
					//					vMain_FetchLabel(ucaMsg_Buff[3], &ucaMsg_Buff[3]);
					//
					//					// Send the label message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					break; //END REQUEST_LABEL
					/*
								//Report the BSL password to the CP
							case REQUEST_BSL_PW:

								// Stuff the header
								ucaMsg_Buff[MSG_TYP_IDX] = REQUEST_BSL_PW;
								ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE + BSLPWDLEN; // BSL password is 32 bytes long
								ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;

								//go to the flash.c file to read the value in the 0xFFE0 to 0xFFFF
								vFlash_GetBSLPW(&ucaMsg_Buff[3]);

								//once the password is obtained send it to the CP
								vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
							break;
					 */
					// The CP requests sensor and board information from the SP
					//////////////////////////////////////////////////////////////////
					//
					//		INTERROGATE
					//
					//////////////////////////////////////////////////////////////////
				case INTERROGATE:
					ucaMsg_Buff[MSG_TYP_IDX] = INTERROGATE;
					ucaMsg_Buff[MSG_LEN_IDX] = 16;  //header and ID packet length
					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					ucaMsg_Buff[3] = 0x02; 	//Two "transducers"
					ucaMsg_Buff[4] = 0x53; 	//Sensor - ADC readings
					ucaMsg_Buff[5] = 0x01; 	//Less than a second duration
					ucaMsg_Buff[6] = 0x41;	//Actuator -forecast data
					ucaMsg_Buff[7] = 0x00;	//Less than a second duration
					/*
					ucaMsg_Buff[3] = ucMain_getNumTransducers(); 	// Number of transducers attached

					// Loop through the number of sensors and fetch the sensor type and sample duration
					ucMsgBuffIdx = 4;
					for(ucTransIdx = 1; ucTransIdx <= ucMain_getNumTransducers(); ucTransIdx++)
					{
						ucaMsg_Buff[ucMsgBuffIdx++] = ucMain_getTransducerType(ucTransIdx);
						ucaMsg_Buff[ucMsgBuffIdx++] = ucMain_getSampleDuration(ucTransIdx);
					}
					 */
					ucMsgBuffIdx = 8;
					// Load the board name into the message buffer
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE1;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE1;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE2;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE2;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE3;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE3;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE4;
					ucaMsg_Buff[ucMsgBuffIdx] = ID_PKT_LO_BYTE4;

					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					break;
					//////////////////////////////////////////////////////////////////
					//
					//		SET_SERIALNUM
					//
					//////////////////////////////////////////////////////////////////
				case SET_SERIALNUM:
					//
					//					ucMsgBuffIdx = MSG_PAYLD_IDX;
					//					uiHID[0] = (uint16) ucaMsg_Buff[ucMsgBuffIdx++];
					//					uiHID[0] = uiHID[0] | (uint16) (ucaMsg_Buff[ucMsgBuffIdx++] << 8);
					//
					//					uiHID[1] = (uint16) ucaMsg_Buff[ucMsgBuffIdx++];
					//					uiHID[1] = uiHID[1] | (uint16) (ucaMsg_Buff[ucMsgBuffIdx++] << 8);
					//
					//					uiHID[2] = (uint16) ucaMsg_Buff[ucMsgBuffIdx++];
					//					uiHID[2] = uiHID[2] | (uint16) (ucaMsg_Buff[ucMsgBuffIdx++] << 8);
					//
					//					uiHID[3] = (uint16) ucaMsg_Buff[ucMsgBuffIdx++];
					//					uiHID[3] = uiHID[3] | (uint16) (ucaMsg_Buff[ucMsgBuffIdx] << 8);
					//
					//					// Write the message header assuming success
					//					ucaMsg_Buff[MSG_TYP_IDX] = SET_SERIALNUM;
					//					ucaMsg_Buff[MSG_LEN_IDX] = 11;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					//
					//					// Write the new HID to flash
					//					if (ucFlash_SetHID(uiHID))
					//					{
					//						// Report an error if the write was unsuccessful
					//						ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
					//						ucaMsg_Buff[MSG_LEN_IDX] = 3;
					//					}
					//					else
					//					{
					//						// Get the SPs serial number from flash
					//						vFlash_GetHID(uiHID);
					//
					//						// Write the new HID to the message buffer
					//						ucaMsg_Buff[3] = (uint8) uiHID[0];
					//						ucaMsg_Buff[4] = (uint8) (uiHID[0] >> 8);
					//						ucaMsg_Buff[5] = (uint8) uiHID[1];
					//						ucaMsg_Buff[6] = (uint8) (uiHID[1] >> 8);
					//						ucaMsg_Buff[7] = (uint8) uiHID[2];
					//						ucaMsg_Buff[8] = (uint8) (uiHID[2] >> 8);
					//						ucaMsg_Buff[9] = (uint8) uiHID[3];
					//						ucaMsg_Buff[10] = (uint8) (uiHID[3] >> 8);
					//					}
					//
					//					// Send the message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);

					break;

					// The CP commands the sensor types to be retrieved
					//////////////////////////////////////////////////////////////////
					//
					//		COMMAND_SENSOR_TYPE
					//
					//////////////////////////////////////////////////////////////////
				case COMMAND_SENSOR_TYPE:
					//				{
					//					uint8 ucChannel;
					//
					//					// loop through each channel, get sample, and assign type
					//					for(ucChannel = 1; ucChannel < 5; ucChannel++)
					//					{
					//						// command the retrieval of sensor type
					//						vMAIN_RequestSensorType(ucChannel);
					//					}
					//				}
					break;

					// The CP requests the sensor types from the SPs
					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_SENSOR_TYPE
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_SENSOR_TYPE:
					//				{
					//					uint8 retVal = 0;
					//					uint8 ucSensorTypes[4];
					//					uint8 ucSensorCount = 0;
					//
					//					// Format first part of return message
					//					ucaMsg_Buff[MSG_TYP_IDX] = 0x0D;
					//					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE + 2;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					//
					//					// Loop through sensors and place types on msg buffer
					//					for(ucSensorCount = 1; ucSensorCount < 5; ucSensorCount++)
					//					{
					//						// get sensor type for a specific channel
					//						retVal = ucMAIN_ReturnSensorType(ucSensorCount);
					//
					//						// otherwise, register type
					//						ucSensorTypes[ucSensorCount - 1] = retVal;
					//					}
					//
					//					// put sensor types on buffer
					//					for(ucSensorCount = 0; ucSensorCount < 4; ucSensorCount++)
					//					{
					//						ucaMsg_Buff[ucSensorCount + 3] = ucSensorTypes[ucSensorCount];
					//					}
					//
					//					// Send the sensor types message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					//				}
					break;

				default:
					//					ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
					//					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION; //-scb
					//
					//					// Send the message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);

					break; //END default
				} // END: switch(g_32DataMsg.fields.ucMsgType)
			}
			else
			{
				vCORE_Send_ErrorMsg(ucCommState);
			} // END: if(ucCOMM_GrabMessageFromBuffer)


			ucRXMessageSize = SP_HEADERSIZE; //Reset ucRXMessageSize

		}



	} // END: while(TRUE)
}

//! @}
