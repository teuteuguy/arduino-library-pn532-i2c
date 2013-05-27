/**************************************************************************/
/*! 
    @file     PN532_I2C.cpp
    @author   teuteuguy
	@license  
	
	@section  HISTORY

    v0.1 - Creation

*/
/**************************************************************************/
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>

#include "PN532_I2C.h"

//#define PN532_I2C_DEBUG
//#define PN532_EZLINK_DEBUG


#define PN532_PACKBUFFSIZ 64
byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

byte pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
byte pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};


/**************************************************************************/
/*! 
    @brief  Sends a single byte via I2C

    @param  x    The byte to send
*/
/**************************************************************************/
static inline void wiresend(uint8_t x) 
{
	#if ARDUINO >= 100
		Wire.write((uint8_t)x);
	#else
		Wire.send(x);
	#endif
}

/**************************************************************************/
/*! 
    @brief  Reads a single byte via I2C
*/
/**************************************************************************/
static inline uint8_t wirerecv(void) 
{
	#if ARDUINO >= 100
		return Wire.read();
	#else
		return Wire.receive();
	#endif
}
/*
char * printHex(int num, int precision) {
      char tmp[16];
      char format[128];

//      sprintf(format, "0x%%.%dX", precision);
      sprintf(format, "%%.%dX", precision);
	//Serial.print(format);
      sprintf(tmp, format, num);
	return tmp;//Serial.print(tmp);
}
*/
char * print8bitHex(int num) {
	char tmp[16];
	char format[128];
    sprintf(format, "%%.%dX", 2);
	sprintf(tmp, format, num);
	return tmp;
}


/**************************************************************************/
/*! 
    @brief  Creates a PN532_I2C class

    @param  irq       Location of the IRQ pin
    @param  reset     Location of the RSTPD_N pin
*/
/**************************************************************************/
PN532_I2C::PN532_I2C(uint8_t irq, uint8_t reset) {
  _pin_irq = irq;
  _pin_reset = reset;

  pinMode(_pin_irq, INPUT);
  pinMode(_pin_reset, OUTPUT);
}


/**************************************************************************/
/*! 
    @brief  Initializes the PN532 hardware (I2C)
*/
/**************************************************************************/
boolean PN532_I2C::init() {
	Wire.begin();
	
	// Reset the PN532  
	digitalWrite(_pin_reset, HIGH);
	digitalWrite(_pin_reset, LOW);
	delay(400);
	digitalWrite(_pin_reset, HIGH);
	
	uint32_t versiondata = getPN532FirmwareVersion();
	
	if (! versiondata) {
		#ifdef PN532_I2C_DEBUG
			Serial.println("PN532_I2C::init: PN532 Board not connected.");
		#endif
		//while (1); // halt
		return false;
	}
	#ifdef PN532_I2C_DEBUG
		Serial.print("PN532_I2C::init: PN5"); Serial.print((versiondata>>24) & 0xFF, HEX);
		Serial.print(" chip (Firmware ver. ");
		Serial.print((versiondata>>16) & 0xFF, DEC);
		Serial.print('.'); Serial.print((versiondata>>8) & 0xFF, DEC);
		Serial.println(") found.");
	#endif
	
	pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
	pn532_packetbuffer[1] = 0x01; // normal mode;
	pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
	pn532_packetbuffer[3] = 0x01; // use IRQ pin!

	if (! sendCommandCheckAck(pn532_packetbuffer, 4)) return false;
	// read data packet
	wirereaddata(pn532_packetbuffer, 8);
	return  (pn532_packetbuffer[6] == 0x15);
}


/**************************************************************************/
/*! 
    @brief  Checks the firmware version of the PN5XX chip

    @returns  The NP532's firmware version and ID
*/
/**************************************************************************/
uint32_t PN532_I2C::getPN532FirmwareVersion(void) {
	uint32_t response;
	pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
	
	if (! sendCommandCheckAck(pn532_packetbuffer, 1)) return 0;
	
	// read data packet
	wirereaddata(pn532_packetbuffer, 12);
	
	// check some basic stuff
	if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {
		#ifdef PN532_I2C_DEBUG
			Serial.println("PN532_I2C::getPN532FirmwareVersion: Firmware doesn't match!");
		#endif
		return 0;
	}
	
	response = pn532_packetbuffer[7];
	response <<= 8;
	response |= pn532_packetbuffer[8];
	response <<= 8;
	response |= pn532_packetbuffer[9];
	response <<= 8;
	response |= pn532_packetbuffer[10];
	
	return response;
}

/**************************************************************************/
/*! 
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes 
    @param  timeout   timeout before giving up
    
    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
boolean PN532_I2C::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {
	uint16_t timer = 0;
	
	// write the command
	wiresendcommand(cmd, cmdlen);
	
	// Wait for chip to say its ready!
	while (wirereadstatus() != PN532_I2C_READY) {
		if (timeout != 0) {
			timer+=10;
			if (timer > timeout) return false;
		}
		delay(10);
	}

	#ifdef PN532_I2C_DEBUG
		Serial.println("PN532_I2C::sendCommandCheckAck: IRQ received");
	#endif
	
	// read acknowledgement
	if (!readackframe()) {
		#ifdef PN532_I2C_DEBUG
			Serial.println("PN532_I2C::sendCommandCheckAck: No ACK frame received!");
		#endif
		return false;
	}
	
	return true; // ack'd command
}

/**************************************************************************/
/*! 
    @brief  Tries to read the PN532 ACK frame (not to be confused with 
	        the I2C ACK signal)
*/
/**************************************************************************/
boolean PN532_I2C::readackframe(void) {
	uint8_t ackbuff[6];
	
	wirereaddata(ackbuff, 6);
	
	return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}

/**************************************************************************/
/*! 
    @brief  Checks the IRQ pin to know if the PN532 is ready
	
	@returns 0 if the PN532 is busy, 1 if it is free
*/
/**************************************************************************/
uint8_t PN532_I2C::wirereadstatus(void) {
	uint8_t x = digitalRead(_pin_irq);

	if (x == 1)
		return PN532_I2C_BUSY;
	else
		return PN532_I2C_READY;
}

/**************************************************************************/
/*! 
    @brief  Reads n bytes of data from the PN532 via I2C

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void PN532_I2C::wirereaddata(uint8_t* buff, uint8_t n) {
	uint16_t timer = 0;
	
	delay(2);
	
	#ifdef PN532_I2C_DEBUG
		Serial.print("PN532_I2C::wirereaddata: Reading: 0x");
	#endif
	
	// Start read (n+1 to take into account leading 0x01 with I2C)
	Wire.requestFrom((uint8_t)PN532_I2C_ADDRESS, (uint8_t)(n+2));
	// Discard the leading 0x01
	wirerecv();
	for (uint8_t i=0; i<n; i++) {
		delay(1);
		buff[i] = wirerecv();
		#ifdef PN532_I2C_DEBUG
			//Serial.print(" 0x");
			//Serial.print(buff[i], HEX);
			Serial.print(" "); Serial.print(print8bitHex(buff[i]));
		#endif
	}
	// Discard trailing 0x00 0x00
	// wirerecv();
	
	#ifdef PN532_I2C_DEBUG
		Serial.println();
	#endif
}

/**************************************************************************/
/*! 
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes 
*/
/**************************************************************************/
void PN532_I2C::wiresendcommand(uint8_t* cmd, uint8_t cmdlen) {
	uint8_t checksum;
	
	cmdlen++;
	
	#ifdef PN532_I2C_DEBUG
		Serial.print("PN532_I2C::wiresendcommand: Sending: 0x");
	#endif

	delay(2);     // or whatever the delay is for waking up the board

	// I2C START
	Wire.beginTransmission(PN532_I2C_ADDRESS);
	checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
	wiresend(PN532_PREAMBLE);
	wiresend(PN532_PREAMBLE);
	wiresend(PN532_STARTCODE2);

	wiresend(cmdlen);
	wiresend(~cmdlen + 1);

	wiresend(PN532_HOSTTOPN532);
	checksum += PN532_HOSTTOPN532;

	#ifdef PN532_I2C_DEBUG
		Serial.print(" "); Serial.print(print8bitHex(PN532_PREAMBLE));
		Serial.print(" "); Serial.print(print8bitHex(PN532_PREAMBLE));
		Serial.print(" "); Serial.print(print8bitHex(PN532_STARTCODE2));
		Serial.print(" "); Serial.print(print8bitHex(cmdlen));
		Serial.print(" "); Serial.print(print8bitHex((uint8_t)(~cmdlen + 1)));
		Serial.print(" "); Serial.print(print8bitHex(PN532_HOSTTOPN532));
		/*Serial.print(" 0x"); Serial.print(PN532_PREAMBLE, HEX);
		Serial.print(" 0x"); Serial.print(PN532_PREAMBLE, HEX);
		Serial.print(" 0x"); Serial.print(PN532_STARTCODE2, HEX);
		Serial.print(" 0x"); Serial.print(cmdlen, HEX);
		Serial.print(" 0x"); Serial.print(~cmdlen + 1, HEX);
		Serial.print(" 0x"); Serial.print(PN532_HOSTTOPN532, HEX);
		*/
	#endif

	for (uint8_t i=0; i<cmdlen-1; i++) {
		wiresend(cmd[i]);
		checksum += cmd[i];
		#ifdef PN532_I2C_DEBUG
			Serial.print(" "); Serial.print(print8bitHex(cmd[i]));
			//Serial.print(" 0x"); Serial.print(cmd[i], HEX);
		#endif
	}

	wiresend(~checksum);
	wiresend(PN532_POSTAMBLE);

	// I2C STOP
	Wire.endTransmission();

	#ifdef PN532_I2C_DEBUG
		//Serial.print(" 0x"); Serial.print(~checksum, HEX);
		//Serial.print(" 0x"); Serial.print(PN532_POSTAMBLE, HEX);
		Serial.print(" "); Serial.print(print8bitHex((uint8_t)(~checksum)));
		Serial.print(" "); Serial.print(print8bitHex(PN532_POSTAMBLE));
		Serial.println();
	#endif
}


/**************************************************************************/
/*! 
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
boolean PN532_I2C::waitUntilReady(uint16_t timeout) {
	uint16_t timer = 0;
	while(wirereadstatus() != PN532_I2C_READY) {
		if (timeout != 0) {
			timer += 10;
			if (timer > timeout) {
				return false;
			}
		}
		delay(10);
	}
	return true;
}

boolean PN532_I2C::checkForEZLink(uint8_t * ezlink, float * balance) {
	pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	pn532_packetbuffer[1] = 1;
	pn532_packetbuffer[2] = 3;
	pn532_packetbuffer[3] = 0;

	#ifdef PN532_EZLINK_DEBUG 
		Serial.println("PN532_I2C::checkForEZLink: Searching for EZLink Cards around");
	#endif

	if (!sendCommandCheckAck(pn532_packetbuffer,4)){//,1000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Could not search for EZLink Cards");
		#endif
		return false;
	}

	//  if (!waitUntilReady(30000)) {
	if (!waitUntilReady(2000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Chip timedout");
		#endif
		return false;
	}

	wirereaddata(pn532_packetbuffer, sizeof(pn532_packetbuffer));

	if (pn532_packetbuffer[0] == 0 &&
		pn532_packetbuffer[1] == 0 &&
		pn532_packetbuffer[2] == 0xff) {
		uint8_t length = pn532_packetbuffer[3];
		if (pn532_packetbuffer[4] != (uint8_t)(~length+1)) {
			#ifdef PN532_EZLINK_DEBUG
				Serial.print("PN532_I2C::checkForEZLink: ERROR - Invalid length from checking the incoming message (0x");
				Serial.print(print8bitHex(length)); Serial.print(" / 0x");
				Serial.print(print8bitHex(uint8_t(~length + 1)));
				Serial.println(")");
			#endif
			return false;
		}
		if (pn532_packetbuffer[5] == PN532_PN532TOHOST &&
			pn532_packetbuffer[6] == PN532_RESPONSE_INLISTPASSIVETARGET) {
			if (pn532_packetbuffer[7] != 1) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.print("PN532_I2C::checkForEZLink: ERROR - More than 1 tag found (Total: ");
					Serial.print(pn532_packetbuffer[7]);
					Serial.println(")");
				#endif
				return false;
			}
			
			inListedTag = pn532_packetbuffer[8];
			#ifdef PN532_EZLINK_DEBUG
				Serial.print("PN532_I2C::checkForEZLink: Found Tag number: ");
				Serial.println(inListedTag);
			#endif
		} else {
			#ifdef PN532_EZLINK_DEBUG
				Serial.println("PN532_I2C::checkForEZLink: ERROR - Unexpected response to EZLink search");
			#endif
			return false;
		} 
	} else {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Preamble missing in response");
		#endif
		return false;
	}

	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;
	pn532_packetbuffer[2] = 144;//90;
	pn532_packetbuffer[3] = 50;//32;
	pn532_packetbuffer[4] = 3;
	pn532_packetbuffer[5] = 0;
	pn532_packetbuffer[6] = 0;
	pn532_packetbuffer[7] = 0;

	if (!sendCommandCheckAck(pn532_packetbuffer, 8)){//, 1000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Could not read EZLink data.");
		#endif
		return false;
	}
	
	if (!waitUntilReady(1000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Chip timedout");
		#endif
		return false;
	}
	wirereaddata(pn532_packetbuffer, 64);
	//memcpy(fullanswer, pn532_packetbuffer, 64);
	//printbuffer(pn532_packetbuffer, 64);
	// 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
	// 00 00 FF 64 9C D5 41 00 02 01 00 0D E2 00 00 00 11 11 73 20 30 96 26 14 A2 0E 22 CC FF 14 14 FF FF
	//                                               ->                       <-
	// DBS card
	// 00 00 FF 64 9C D5 41 00 02 01 00 0D E2 00 00 00 11 11 73 20 30 96 26 14 A2 0E 22 CC FF 14 14 FF FF
	//                                                 XX XX XX XX XX XX XX XX 
	if (pn532_packetbuffer[0] == 0 &&
		pn532_packetbuffer[1] == 0 &&
		pn532_packetbuffer[2] == 0xff) {
		uint8_t length = pn532_packetbuffer[3];
		if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
			#ifdef PN532_EZLINK_DEBUG
				Serial.print("PN532_I2C::checkForEZLink: ERROR - Invalid length from checking the incoming message (0x");
				Serial.print(print8bitHex(length)); Serial.print(" / 0x");
				Serial.print(print8bitHex(uint8_t(~length + 1)));
				Serial.println(")");
			#endif
			return false;
		}
		if (pn532_packetbuffer[5] == PN532_PN532TOHOST && // D5
			pn532_packetbuffer[6] == PN532_RESPONSE_INDATAEXCHANGE && // 41
			pn532_packetbuffer[7] == 0 // OK (1 = Error)
			) {
				//if (pn532_packetbuffer[16] == 0x94) printbuffer(pn532_packetbuffer, 64);
				//for (int i = 0; i < 8; i ++) {
				//	ezlink[i] = pn532_packetbuffer[i + 16];
				//	//Serial.print(printHex(pn532_packetbuffer[i + 16], 2));
				//}
				memcpy(ezlink, pn532_packetbuffer + 16, 8);
				*balance = (float)(pn532_packetbuffer[11] * 256 + pn532_packetbuffer[12]) / 100;
				#ifdef PN532_EZLINK_DEBUG
					Serial.print("PN532_I2C::checkForEZLink: EZLink CAN:");
					for (int i = 0; i < 8; i ++) {
						Serial.print(" "); Serial.print(print8bitHex(ezlink[i]));
					}
					Serial.print(" with balance of ");
					Serial.println(*balance);
				#endif
		} else {
			#ifdef PN532_EZLINK_DEBUG
				Serial.println("PN532_I2C::checkForEZLink: ERROR - Unexpected response to EZLink read");
			#endif
			return false;
		} 
	} else {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Preamble missing in response");
		#endif
		return false;
	}
	
	pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
	pn532_packetbuffer[1] = 1;

	if (!sendCommandCheckAck(pn532_packetbuffer, 2)){//, 1000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Could not release.");
		#endif
		return false;
	}
	
	if (!waitUntilReady(1000)) {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Chip timedout");
		#endif
		return false;
	}
	wirereaddata(pn532_packetbuffer, 64);
	
	if (pn532_packetbuffer[0] == 0 &&
		pn532_packetbuffer[1] == 0 &&
		pn532_packetbuffer[2] == 0xff) {
		uint8_t length = pn532_packetbuffer[3];
		if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
			#ifdef PN532_EZLINK_DEBUG
				Serial.print("PN532_I2C::checkForEZLink: ERROR - Invalid length from checking the incoming message (0x");
				Serial.print(print8bitHex(length)); Serial.print(" / 0x");
				Serial.print(print8bitHex(uint8_t(~length + 1)));
				Serial.println(")");
			#endif
			return false;
		}
		if (pn532_packetbuffer[5] == PN532_PN532TOHOST && // D5
			pn532_packetbuffer[6] == PN532_RESPONSE_INRELEASE && // 41
			pn532_packetbuffer[7] == 0 // OK (1 = Error)
			) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.println("PN532_I2C::checkForEZLink: EZLink released");
				#endif
		} else {
			#ifdef PN532_EZLINK_DEBUG
				Serial.println("PN532_I2C::checkForEZLink: ERROR - Unexpected response to release");
			#endif
			return false;
		} 
	} else {
		#ifdef PN532_EZLINK_DEBUG
			Serial.println("PN532_I2C::checkForEZLink: ERROR - Preamble missing in response");
		#endif
		return false;
	}
	return true;
}

boolean PN532_I2C::checkForEZLink_Transparent(uint8_t * ezlink, float * balance) {
	static uint8_t checkForEZLink_state = 0;
	
	switch (checkForEZLink_state) {
		case 0:
			pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
			pn532_packetbuffer[1] = 1;
			pn532_packetbuffer[2] = 3;
			pn532_packetbuffer[3] = 0;
			
			#ifdef PN532_EZLINK_DEBUG 
				Serial.println("PN532_I2C::checkForEZLink: Searching for EZLink Cards around");
			#endif
			
			if (!sendCommandCheckAck(pn532_packetbuffer, 4, 100)) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.println("PN532_I2C::checkForEZLink: ERROR - NP532 is not responding");
				#endif
			} else {
				checkForEZLink_state = 1;
			}
			//wiresendcommand(pn532_packetbuffer, 4);
			break;
		case 1:
			// Wait for NP532 to detect something that could be an EZLink
			if ( wirereadstatus() == PN532_I2C_READY ) {
				checkForEZLink_state = 2;
			}
			break;
		case 2:
			wirereaddata(pn532_packetbuffer, sizeof(pn532_packetbuffer));
			
			if (pn532_packetbuffer[0] == 0 &&
				pn532_packetbuffer[1] == 0 &&
				pn532_packetbuffer[2] == 0xff &&
				pn532_packetbuffer[4] == (uint8_t)(~pn532_packetbuffer[3] + 1) &&
				pn532_packetbuffer[5] == PN532_PN532TOHOST &&
				pn532_packetbuffer[6] == PN532_RESPONSE_INLISTPASSIVETARGET &&
				pn532_packetbuffer[7] == 1) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.println("PN532_I2C::checkForEZLink: NP532 found an EZLink");
				#endif
				checkForEZLink_state = 3;
			} else {
				checkForEZLink_state = 0;
			}
			break;
		case 3:
			pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
			pn532_packetbuffer[1] = 1;
			pn532_packetbuffer[2] = 144;//90;
			pn532_packetbuffer[3] = 50;//32;
			pn532_packetbuffer[4] = 3;
			pn532_packetbuffer[5] = 0;
			pn532_packetbuffer[6] = 0;
			pn532_packetbuffer[7] = 0;
			
			#ifdef PN532_EZLINK_DEBUG 
				Serial.println("PN532_I2C::checkForEZLink: Request read EZLink Card data");
			#endif
			
			if (!sendCommandCheckAck(pn532_packetbuffer, 8, 100)) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.println("PN532_I2C::checkForEZLink: ERROR - NP532 is not responding");
				#endif
			} else {
				checkForEZLink_state = 4;
			}
			break;
		case 4:
			// Wait for card to reply to the read request
			if ( wirereadstatus() == PN532_I2C_READY ) {
				checkForEZLink_state = 5;
			}
			break;
		case 5:
			wirereaddata(pn532_packetbuffer, sizeof(pn532_packetbuffer));
			
			if (pn532_packetbuffer[0] == 0 &&
				pn532_packetbuffer[1] == 0 &&
				pn532_packetbuffer[2] == 0xff &&
				pn532_packetbuffer[4] == (uint8_t)(~pn532_packetbuffer[3] + 1) &&
				pn532_packetbuffer[5] == PN532_PN532TOHOST &&
				pn532_packetbuffer[6] == PN532_RESPONSE_INDATAEXCHANGE &&
				pn532_packetbuffer[7] == 0) {
					
				memcpy(ezlink, pn532_packetbuffer + 16, 8);
				*balance = (float)(pn532_packetbuffer[11] * 256 + pn532_packetbuffer[12]) / 100;
				
				#ifdef PN532_EZLINK_DEBUG
					Serial.print("PN532_I2C::checkForEZLink: EZLink CAN:");
					for (int i = 0; i < 8; i ++) {
						Serial.print(" "); Serial.print(print8bitHex(ezlink[i]));
					}
					Serial.print(" with balance of ");
					Serial.println(*balance);
				#endif
				checkForEZLink_state = 6;
//				return true;
			} else {
				checkForEZLink_state = 0;
			}
			break;
		case 6:
			pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
			pn532_packetbuffer[1] = 1;
			
			#ifdef PN532_EZLINK_DEBUG 
				Serial.println("PN532_I2C::checkForEZLink: Request release of EZLink Card");
			#endif
			
			if (!sendCommandCheckAck(pn532_packetbuffer, 8, 100)) {
				#ifdef PN532_EZLINK_DEBUG
					Serial.println("PN532_I2C::checkForEZLink: ERROR - NP532 is not responding");
				#endif
			} else {
				checkForEZLink_state = 7;
			}
			break;
		case 7:
			// Wait for card to reply to the read request
			if ( wirereadstatus() == PN532_I2C_READY ) {
				checkForEZLink_state = 8;
			}
			break;
		case 8:
			wirereaddata(pn532_packetbuffer, sizeof(pn532_packetbuffer));
			
			checkForEZLink_state = 0;

			if (pn532_packetbuffer[0] == 0 &&
				pn532_packetbuffer[1] == 0 &&
				pn532_packetbuffer[2] == 0xff &&
				pn532_packetbuffer[4] == (uint8_t)(~pn532_packetbuffer[3] + 1) &&
				pn532_packetbuffer[5] == PN532_PN532TOHOST &&
				pn532_packetbuffer[6] == PN532_RESPONSE_INRELEASE &&
				pn532_packetbuffer[7] == 0) {
				return true;
			}
			break;
		default:
			checkForEZLink_state = 0;
			break;
	}
	
	return false;
}
