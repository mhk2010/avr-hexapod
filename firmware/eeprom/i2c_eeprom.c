//**********************************************************************//
//FILE: i2c_eeprom.c 
//AUTHOR: Adam Kadolph
//DATE:	12-7-2008
//DESCRIPTION: C file to handle i2c eeprom byte and page read/write operations
//**********************************************************************//

#include <avr/io.h>
#include <stdio.h>
#include "i2c_eeprom.h"
#include <i2c.h>

uint8_t i2c_EEPROM_byte_read(uint16_t addr)
{
	uint8_t low,high,status,data;
#ifndef _SIMULATION

	high = (addr>>8);
	low = addr;
	
	status=i2c_start();
	if(status)
	{
		i2c_stop();
		return 0;
	}
	
	status=i2c_sendAddr(EEPROMA_W);
	if(status)
	{
		i2c_stop();
		return 0;
	}

	status=i2c_sendData(high);
	if(status)
	{
		i2c_stop();
		return 0;
	}

	status=i2c_sendData(low);
	if(status)
	{
		i2c_stop();
		return 0;
	}

	status=i2c_repeatStart();
	if(status)
	{
		i2c_stop();
		return 0;
	}
	
	status=i2c_sendAddr(EEPROMA_R);
	if(status)
	{
		i2c_stop();
		return 0;
	}

	data = i2c_receiveData_NACK();
	if(data == ERROR_CODE)
   	{
		i2c_stop();
	   	return 0;
   	}
	i2c_stop();
#endif
	return(data);
}

/////////////////////////////////////////////////////////////////
void i2c_EEPROM_byte_write(uint16_t addr, uint8_t data)
{
	uint8_t low,high,status;
#ifndef _SIMULATION

	high = (addr>>8);
	low = addr;

	status=i2c_start();
	if(status == 1)
	{
		i2c_stop();
		return;
	}
	
	status=i2c_sendAddr(EEPROMA_W);
	if(status == 1)
	{
		i2c_stop();
		return;
	}

	status=i2c_sendData(high);
	if(status == 1)
	{
		i2c_stop();
		return;
	}

	status=i2c_sendData(low);
	if(status == 1)
	{
		i2c_stop();
		return;
	}

	status=i2c_sendData(data);
	if(status == 1)
	{
		i2c_stop();
		return;
	}
	i2c_stop();
#endif
}

