arduino-library-pn532-i2c
==============

This library is based on adafruit PN532's implementation of NFC.
Created for EZLink reading via PN532

## Usage
boolean 	checkForEZLink(uint8_t * ezlink, float * balance);
boolean 	checkForEZLink_Transparent(uint8_t * ezlink, float * balance);

Transparent mode enables non blocking mode. Work in progress.

## Dependancies

* Arduino
* PN532 NFC controller board. Available at [Adafruit Industries][0].

## Limitations

* Library handles EZLink only for now. 

##To Do


## Thanks To

[0]: http://www.adafruit.com/products/789