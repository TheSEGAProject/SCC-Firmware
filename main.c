


#include "msp.h"
#include "core/core.h"
#include "helpers/led.h"
#include "helpers/adc.h"
#include "helpers/uart.h"
#include "helpers/timer.h"
#include <stdio.h>
#include <cstring>

uint8 debug = 0x00;

extern volatile uint8 ucADC_COMPLETE;
extern volatile uint8 ucFaultDetected;
extern volatile float Battery_ADC[Num_of_Results];
extern volatile float Solar_V_ADC[Num_of_Results];
extern volatile float Solar_I_ADC[Num_of_Results];

extern volatile float fBatteryVoltage;
extern volatile float fSolarVoltage;

extern volatile float fCurrentCurrent;
extern volatile float fCurrentVoltage;
extern volatile float fAverageCurrent;
extern volatile float fAveragePower;
extern volatile float fAverageVoltage;
extern volatile float fVsense;


//volatile unsigned char j;


///////////////////////////////////////////////////////////////////////////////
//!
//! \brief Loads the passed buffer with data stored in the S_Report structure
//!
//! \param *pBuff
//! \return ucLength, the amount of bytes added to the passed buffer
///////////////////////////////////////////////////////////////////////////////
uint8 ucMain_FetchData(uint8 * pucBuff)
{
	uint32 ulConvertedValue;

	*pucBuff++ = 0x01;//Data generator 1 (Battery ADC)
	*pucBuff++ = 0x04;//Length of value

	ulConvertedValue = (long)(fBatteryVoltage*1000);
	*pucBuff++ = (uint8) (ulConvertedValue >> 24);
	*pucBuff++ = (uint8) (ulConvertedValue >> 16);
	*pucBuff++ = (uint8) (ulConvertedValue >> 8);
	*pucBuff++ = (uint8) ulConvertedValue;

	*pucBuff++ = 0x02;//Data generator 2 (Solar Vin ADC)
	*pucBuff++ = 0x04;//Length of value

	ulConvertedValue = (long)(fSolarVoltage*1000); //Convert to mV
	*pucBuff++ = (uint8) (ulConvertedValue >> 24);
	*pucBuff++ = (uint8) (ulConvertedValue >> 16);
	*pucBuff++ = (uint8) (ulConvertedValue >> 8);
	*pucBuff++ = (uint8) ulConvertedValue;


	*pucBuff++ = 0x03;//Data generator 3 (Solar Vin MAX9934 ADC)
	*pucBuff++ = 0x04;//Length of value

	ulConvertedValue = (long)(fAverageVoltage*1000); //Convert to mV
	*pucBuff++ = (uint8) (ulConvertedValue >> 24);
	*pucBuff++ = (uint8) (ulConvertedValue >> 16);
	*pucBuff++ = (uint8) (ulConvertedValue >> 8);
	*pucBuff++ = (uint8) ulConvertedValue;

	*pucBuff++ = 0x04;//Data generator 4 (Solar Iload MAX9934 ADC)
	*pucBuff++ = 0x04;//Length of value

	ulConvertedValue = (long)(fAverageCurrent*1000); //Convert to mA
	*pucBuff++ = (uint8) (ulConvertedValue >> 24);
	*pucBuff++ = (uint8) (ulConvertedValue >> 16);
	*pucBuff++ = (uint8) (ulConvertedValue >> 8);
	*pucBuff++ = (uint8) ulConvertedValue;


	*pucBuff++ = 0x05;//Data generator 5 (Solar Pload MAX9934 ADC)
	*pucBuff++ = 0x04;//Length of value

	ulConvertedValue = (long)(fAveragePower*1000); //Convert to mW
	*pucBuff++ = (uint8) (ulConvertedValue >> 24);
	*pucBuff++ = (uint8) (ulConvertedValue >> 16);
	*pucBuff++ = (uint8) (ulConvertedValue >> 8);
	*pucBuff++ = (uint8) ulConvertedValue;

	if(ucFaultDetected){
		*pucBuff++ = 0x06;//Data generator 6 (Potential Fault)
		*pucBuff++ = 0x01;//Length of value
		*pucBuff++ = (uint8) 0x00;
		return 33; //length of report with a fault detection
	}
	else{
		return 30; //length of 5 6-byte data elements
	}


}
///////////////////////////////////////////////////////////////////////////////
//!
//! \brief Dispatches to the sensor functions
//!
//! This is called from the core.  Calling a function in the application layer
//! to dispatch to the sensor functions decouples the core since it does not
//! need to "know" anything about the number of transducers
//!
//! \param ucCmdTransNum, the transducer number; ucCmdParamLen, length of parameters
//! *ucParam, pointer to parameter array
//! \return
//!
///////////////////////////////////////////////////////////////////////////////
uint16 uiMainDispatch(uint8 ucCmdTransNum, uint8 ucCmdParamLen, uint8 *ucParam)
{

	uint8 ucRetVal;

	switch (ucCmdTransNum)
	{
	case 0:
		//diagnostic stuff
		break;
	case 1:
		vADC_Init();
		ucADC_COMPLETE = 0x00;//Set adc complete flag to 0
		ucRetVal = uiADC_Read(ucParam);
		break;
	case 2:
		//forecast data
		break;
	default:
		ucRetVal = 1;
		break;

	}

	return ucRetVal;

}


int main(void)
{

	//Init the core
	vCORE_Initialize();
	//Init and blink the LED, then disable
	vLED_Init();
	vLED_Blink_Delay(6,SHORT_DELAY);
	vCORE_Run();

}
