//This code file was generated by ComBuilder on 17.03.2014 18:01:05

/**********************************************************************************
File Description:
For general sensor information see '../sensors/'ADCSensor.h

All messages have to be understand by Nios2 and Linux-PC. Please use the flag
CENTRAL_ECU_BUILD to distinguish the different builds!

**********************************************************************************/


/**********************************************************************************
SubMessage Description:
Contains information about connected channels.
**********************************************************************************/
// TODO: put somewhere sensible
#define CENTRAL_ECU_BUILD
// Include guard for different build types
#ifndef CENTRAL_ECU_BUILD
#include "adc_interface.h"
#include "properties.h"
#endif /* CENTRAL_ECU_BUILD */

// Export Interfaces
#include "ADCInfoMessage.h"

// Import Interfaces

/*
 * Basic Constructor. Used to initialise new message-objects.
 */
CADCInfoMessage::CADCInfoMessage(alt_u8 uiChannelNumbers[])
{
    setHeader(10, getLength(), 0);

    m_uiChannelCount = 0;
    m_uiChannelNumbers[0] = uiChannelNumbers[0];
    m_uiChannelNumbers[1] = uiChannelNumbers[1];
}

/*
 * Constructor. Used to reassemble object from Byte stream.
 * pMessage : Byte-Array containing the fields of the subMessage
 * iLength  : Length of the said array
 */
CADCInfoMessage::CADCInfoMessage(alt_u8 *pMessage, int iLength)
{
    parseHeader(pMessage, iLength);
    if(m_bValid)
    {
        m_bValid = false;
        parseMessage(pMessage+4, iLength-4);
    }

}

/*
 * Basic Destructor.
 */
CADCInfoMessage::~CADCInfoMessage()
{
}

/*
 * Overrided from CCarMessage.
 */
void CADCInfoMessage::doAction()
{
#ifndef CENTRAL_ECU_BUILD
	alt_u8 uiNumbers[] = ADC_CHANNEL_NUMBERS;
	char uiAliases[] = ADC_CHANNEL_ALIAS_STRINGS;
	answerMessage(ADC_CONNECTED_CHANNELS, uiNumbers, (alt_u8*) uiAliases);
#endif /* CENTRAL_ECU_BUILD */

}

/*
 * Overrided from CCarMessage.
 */
void CADCInfoMessage::answerMessage(alt_u8 uiChannelCount, alt_u8 uiChannelNumbers[], alt_u8 uiChannelAliasStrings[])
{
	alt_u32 i = 0;
    m_uiChannelCount = uiChannelCount;
    for(i = 0; i < 7; i++)
    	m_uiChannelAliasStrings[i] = uiChannelAliasStrings[m_uiChannelNumbers[0]*7+i];
    for(i = 7; i < 14; i++)
		m_uiChannelAliasStrings[i] = uiChannelAliasStrings[m_uiChannelNumbers[1]*7+i];
		
	m_uiChannelNumbers[0] = uiChannelNumbers[m_uiChannelNumbers[0]];
	m_uiChannelNumbers[1] = uiChannelNumbers[m_uiChannelNumbers[1]];
    m_uiFlags = m_uiFlags | 0x01; // Set response flag
}

/*
 * Overrided from CCarMessage.
 */
bool CADCInfoMessage::getBytes(alt_u8 *pMessage)
{
    CCarMessage::getBytes(pMessage);

    pMessage[4] = m_uiChannelCount;
    pMessage[5] = 0;
    pMessage[6] = 0;
    pMessage[7] = 0;
    pMessage[8] = m_uiChannelNumbers[0];
    pMessage[9] = m_uiChannelAliasStrings[0];
    pMessage[10] = m_uiChannelAliasStrings[1];
    pMessage[11] = m_uiChannelAliasStrings[2];
    pMessage[12] = m_uiChannelAliasStrings[3];
    pMessage[13] = m_uiChannelAliasStrings[4];
    pMessage[14] = m_uiChannelAliasStrings[5];
    pMessage[15] = m_uiChannelAliasStrings[6];
    pMessage[16] = m_uiChannelNumbers[1];
    pMessage[17] = m_uiChannelAliasStrings[7];
    pMessage[18] = m_uiChannelAliasStrings[8];
    pMessage[19] = m_uiChannelAliasStrings[9];
    pMessage[20] = m_uiChannelAliasStrings[10];
    pMessage[21] = m_uiChannelAliasStrings[11];
    pMessage[22] = m_uiChannelAliasStrings[12];
    pMessage[23] = m_uiChannelAliasStrings[13];
    return m_bValid;
}

/*
 * Overrided from CCarMessage.
 */
alt_u32 CADCInfoMessage::getLength()
{
    return 24;
}

/*
 * Overrided from CCarMessage.
 */
void CADCInfoMessage::parseMessage(alt_u8 *pMessage, int iLength)
{
    if(iLength < 20)
        return;

    m_uiChannelCount = pMessage[0];
    m_uiChannelNumbers[0] = pMessage[4];
    m_uiChannelAliasStrings[0] = pMessage[5];
    m_uiChannelAliasStrings[1] = pMessage[6];
    m_uiChannelAliasStrings[2] = pMessage[7];
    m_uiChannelAliasStrings[3] = pMessage[8];
    m_uiChannelAliasStrings[4] = pMessage[9];
    m_uiChannelAliasStrings[5] = pMessage[10];
    m_uiChannelAliasStrings[6] = pMessage[11];
    m_uiChannelNumbers[1] = pMessage[12];
    m_uiChannelAliasStrings[7] = pMessage[13];
    m_uiChannelAliasStrings[8] = pMessage[14];
    m_uiChannelAliasStrings[9] = pMessage[15];
    m_uiChannelAliasStrings[10] = pMessage[16];
    m_uiChannelAliasStrings[11] = pMessage[17];
    m_uiChannelAliasStrings[12] = pMessage[18];
    m_uiChannelAliasStrings[13] = pMessage[19];

    m_bValid = true;

}
