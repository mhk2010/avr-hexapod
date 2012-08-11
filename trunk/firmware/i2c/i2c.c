//**********************************************************************//
//FILE: twi.c 
//AUTHOR: Adam Kadolph, Martin Metal
//DATE:	12-5-08
//DESCRIPTION: This file handles the Two Wired Serial Interface to the AVR
//**********************************************************************//

#include "i2c.h"
#include <avr/io.h>
#include <util/delay.h>

// -------------------------------------------------------
//   7   |   6  |   5   |   4   |   3  |   2  | 1 |  0   | 
// TWINT | TWEA | TWSTA | TWSTO | TWWC | TWEN | x | TWIE |
// -------------------------------------------------------

void i2c_init(uint8_t mode)
{
	TWBR = (F_CPU / 100000UL - 16) / 2;	//set bit rate for TWI, SCL f=100kHz, fclk=8Mhz
	TWSR = 0x00;						//no prescaler value

	if( (uint8_t) I2C_FAST == (uint8_t) mode ) {
		TWBR = (F_CPU / 400000UL - 16) / 2;	//set bit rate for TWI, SCL f=400kHz, fclk=8Mhz
		TWSR = 0x00;						//no prescaler value
	}
}


unsigned char i2c_start(void)
{
	uint8_t i;

	//TWI transmit start
	TWCR = (1<<TWINT)| (1<<TWSTA)| (1<<TWEN); // | (1<<TWEA);
	
	for(i=0;i<100;i++) {
		if(TWCR & (1<<TWINT)) {
			if ((TWSR & STATUS_MASK) == I2C_START)	//check TWI status register
				return(0);
			else
				return(TWSR & STATUS_MASK);
		}
	}
	return(TWSR & STATUS_MASK); // ERROR, send the error status to calling app.
}


unsigned char i2c_repeatStart(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 	//Send START condition
	while (!(TWCR & (1<<TWINT)));   			//Wait for TWINT flag set. This indicates that the
		  										//START condition has been transmitted
	if ((TWSR & STATUS_MASK) == I2C_REPEAT_START)	//Check value of TWI Status Register
		return(0);
	else
		return(TWSR & STATUS_MASK);
}


unsigned char i2c_stop(void)
{
//	uint8_t i;

	//TWI transmit stop
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	//transmit stop condition

//	if(!(TWCR & (1<<TWINT)));		//wait for TWINT flag set to indicate stop condition has been transmitted
//	for(i=0;i<100;i++) {
//		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	//transmit stop condition
//		if(TWCR & (1<<TWINT)) {
//			return(0);
//		}
//	}
//	TWCR = 0;
//	TWCR |= _BV(TWINT);
//	return(TWSR & STATUS_MASK); // ERROR, send the error status to calling app.
	return(0); 
}


void i2c_slave( unsigned char addr )
{
	TWAR = addr;
	
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWIE) | (1<<TWEA);
	/* Enable TWI and enable interrupt, 
	 * enable acknowledgement on own
	 * address detection 
	 */
}



#ifdef _I2C_SLAVE_RECEIVER

unsigned char i2c_sl_recData( unsigned char * data )

{
	unsigned int status;

	/*
	 * Wait for interrupt on I2C bus to occur
	 */
	while(!(TWCR & (1<<TWINT)));

	status = (TWSR & STATUS_MASK);

	/*
	 * Send back the received data if the interrupt
	 * generated on the I2C bus is announcing
	 * incoming data
	 */
	if( (status == SR_DATA_ACK) || (status == SR_DATA_NACK) )					
	{
		TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT)| (1<<TWEA);
		*data = TWDR;
		return 0;
	}

	/* renew the status register
	 * - Enable TWI-interface and release TWI pins
	 * - Keep interrupt enabled and clear the flag
	 * - Acknowledge on any new requests.
	 */
	TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT)| (1<<TWEA);

	*data = 0x0;
	return status;
}

#endif


unsigned char i2c_sendAddr(unsigned char addr, unsigned char flags )
{
	//TWI transmit address in Master Transmit Mode
	unsigned char STATUS;
   
	if((addr & 0x01) == 0) 
		STATUS = MT_SLA_ACK;
	else
		STATUS = MR_SLA_ACK; 
	
	TWDR = addr;					//load SLA+W into TWDR register
	TWCR = (1<<TWINT)|(1<<TWEN) | flags ;

	while(!(TWCR & (1<<TWINT)));	//wait for TWINT flag set to indicate that SLA_W has been transmitted 
	
	if ((TWSR & STATUS_MASK) == STATUS)//check value of TWI status that ACK bit has been received
		return(0);
	else 
		return((TWSR & STATUS_MASK));
}


unsigned char i2c_sendData(unsigned char data)
{
	//TWI transmit data in Master Transmit Mode
	TWDR = data;					//load data into TWDR register
	TWCR = (1<<TWINT)|(1<<TWEN);

	while(!(TWCR & (1<<TWINT)));		//wait for TWINT flag set to indicate data has been transmitted
		
	if ((TWSR & STATUS_MASK) == MT_DATA_ACK)//check value of TWI status register
		return(0);
	else
		return(TWSR & STATUS_MASK);
}


unsigned char i2c_receiveData( uint8_t * data, uint8_t flags )
{
	unsigned char ret;
  
	if( flags == 0x00 )
		TWCR = (1<<TWINT)|(1<<TWEN); // (1<<TWEA)|(1<<TWINT)|(1<<TWEN); go for NACK data
	else
		TWCR = flags;
  
	while (!(TWCR & (1<<TWINT)));	   	   //Wait for TWINT flag set. This indicates that the
   		 		   					   		//data has been received
	*data = TWDR;
	ret = (TWSR & STATUS_MASK);

	if( flags & (1<<TWEA) ) {
		if( ret == MR_DATA_ACK )
			return(0);
		else
			return(ret);
	} else {
		if( ret == MR_DATA_NACK )
			return(0);
		else
			return(ret);
	}
}


/* 
unsigned char i2c_receiveData_ACK(void)
{
  unsigned char data;
  
  TWCR = (1<<TWEA)|(1<<TWINT)|(1<<TWEN);
  
  while (!(TWCR & (1<<TWINT)));	   	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //data has been received
  if ((TWSR & STATUS_MASK) != MR_DATA_ACK)    //Check value of TWI Status Register
   	  return(ERROR_CODE);
  
  data = TWDR;
  return(data);
}


unsigned char i2c_receiveData_NACK(void)
{
  unsigned char data;
  
  TWCR = (1<<TWINT)|(1<<TWEN);
  
  while (!(TWCR & (1<<TWINT)));	   	   //Wait for TWINT flag set. This indicates that the
   		 		   					   //data has been received
  if ((TWSR & STATUS_MASK) != MR_DATA_NACK)    //Check value of TWI Status Register
   	  return(ERROR_CODE);
  
  data = TWDR;
  return(data);
}
*/


/* Following fucntions are required for I2C displays,
 * The initialization sequence required special handling.
 * The SRF02, DS1307 or EEPROM do not required this
 * extended mode at all. IS commented out, shall be ideally
 * wrapped inside a specific defines. That will be universal 

unsigned char i2c_start_co(unsigned char addr, unsigned char co )
{
	unsigned char status = 0;

	//TWI transmit start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);//send start condition
	
	while(!(TWCR & (1<<TWINT)));		//wait for TWINT flag set to indicate start condition has been transmitted
	
	if ((TWSR & STATUS_MASK) != I2C_START)	//check TWI status register
		return(I2C_START);

	status = i2c_sendAddr((unsigned char) addr, 0x00 );
	if( status )
	{
		i2c_stop();
		return( status );
	}

	_delay_ms( 1 );

	status = i2c_sendData_ex((unsigned char) co );
	if(status)
	{
		i2c_stop();
		return( status );
	}

	return( 0 );
}


unsigned char i2c_sendData_ex(unsigned char data)
{
	//TWI transmit data in Master Transmit Mode
	TWDR = data;					//load data into TWDR register
	TWCR = (1<<TWINT)|(1<<TWEN);

	while(!(TWCR & (1<<TWINT)));		//wait for TWINT flag set to indicate data has been transmitted
		
	if( (TWSR & STATUS_MASK) == MT_DATA_ACK)
		return( 0 );

	if( (TWSR & STATUS_MASK) == MT_SLA_ACK)
		return( 0 );

	return (TWSR & STATUS_MASK);
}
*/
