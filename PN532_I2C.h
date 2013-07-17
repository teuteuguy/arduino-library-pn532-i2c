/**************************************************************************/
/*! 
    @file     PN532_I2C.h
    @author   teuteuguy
	@license  
	
	@section  HISTORY

    v0.1  - Creation
	
*/
/**************************************************************************/

#ifndef PN532_I2C_h
#define PN532_I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>

// PN532 I2C Shield uses the following pins
// Analog 4 => I2C
// Analog 5 => I2C
// Digital 2 => IRQ
// Digital 3 => Reset (not sure if really used)


// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INRELEASE             (0x52)

// PN532 Responses
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)
#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INRELEASE            (0x53)

#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

#define PN532_I2C_ADDRESS                   (0x48 >> 1)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)


class PN532_I2C {
	public:
					PN532_I2C(uint8_t pin_irq, uint8_t pin_reset);
		bool 		init(void);
		bool	 	checkForEZLink(uint8_t * ezlink, float * balance);
		bool	 	checkForEZLink_Transparent(uint8_t * ezlink, float * balance);
		
	private:
		uint8_t		_pin_irq, _pin_reset;
		uint8_t		inListedTag; // Tag number of inlisted tag.
		
		uint32_t	getPN532FirmwareVersion(void);
		bool		sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout = 1000);

		bool		readackframe(void);
		uint8_t		wirereadstatus(void);
		void		wirereaddata(uint8_t* buff, uint8_t n);
		void		wiresendcommand(uint8_t* cmd, uint8_t cmdlen);
		bool		waitUntilReady(uint16_t timeout);
};

#endif