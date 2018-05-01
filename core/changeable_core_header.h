///////////////////////////////////////////////////////////////////////////////
//! \file changeable_core_header.h
//! \brief The header file for the core that a user is allowed to change and
//!	adjust settings.
//!
//!
//! @addtogroup core
//! @{
//!
//! @addtogroup hdr Core Changeable Header
//! This is the header file that should be adjusted so the core can be used
//! with any SP Board transducer functions.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Samuel Boegli
//     Wireless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//
//	edited by CP
//*****************************************************************************

#ifndef CHANGEABLE_CORE_HEADER_H_
#define CHANGEABLE_CORE_HEADER_H_



//! @name SP Board Software Version Variable
//!
//! @{
//!\def SOFTWAREVERSION
//! \brief The software version running on the SP Board. Not the Core version.
#define SOFTWAREVERSION "SCC v1.00 "
//! @}

//! @name SP Board ID Variables
//! These variables are used to store the name of the SP Board.
//! @{
#define  ID_PKT_HI_BYTE1	0x53;
#define  ID_PKT_LO_BYTE1	0x54;
#define  ID_PKT_HI_BYTE2	0x4D;
#define  ID_PKT_LO_BYTE2	0x00;
#define  ID_PKT_HI_BYTE3	0x00;
#define  ID_PKT_LO_BYTE3	0x00;
#define  ID_PKT_HI_BYTE4	0x00;
#define  ID_PKT_LO_BYTE4	0x00;
//!@}



// Functions visible to the core.  Adding these functions makes the core scalable to any application
// since the core does not need to know anything about how many transducers there are.
uint8 ucMain_FetchData(uint8 * pBuff);
void vMain_FetchLabel(uint8 ucTransNum, uint8 * pucArr);
uint16 uiMainDispatch(uint8 ucCmdTransNum, uint8 ucCmdParamLen, uint8 *ucParam);
void vMAIN_RequestSensorType(uint8 ucChannel);
uint8 ucMAIN_ReturnSensorType(uint8 ucSensorCount);
uint8 ucMain_getNumTransducers(void);
uint8 ucMain_getSampleDuration(uint8 ucTransNum);
uint8 ucMain_getTransducerType(uint8 ucTransNum);
#endif /* CHANGEABLE_CORE_HEADER_H_ */

