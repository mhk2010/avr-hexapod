//**********************************************************************//
//FILE: i2c_eeprom.h 
//AUTHOR: Adam Kadolph
//DATE:	12-5-2008
//DESCRIPTION: Header file to declare i2c eeprom functions
//**********************************************************************//

#ifndef i2c_eeprom_h
#define i2c_eeprom_h

//define EEPROM R/W addresses
#define  EEPROMA_W			0xA0
#define  EEPROMA_R			0xA1

//declare eeprom functions
uint8_t i2c_EEPROM_byte_read(uint16_t addr);
void i2c_EEPROM_byte_write(uint16_t addr, uint8_t data);

#endif
