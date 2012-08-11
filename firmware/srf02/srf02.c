#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include <i2c.h>
//#include <i2c_eeprom.h>
#include "srf02.h"



uint8_t srf02_get_range_byte( uint8_t index, uint8_t *data )
{ //{{{
	if( index == LO_BYTE ) 
		return i2c_srf02_read_byte_ex( SRF02_DISTANCE_LO, 0x84, data );
	if( index == HI_BYTE ) 
		return i2c_srf02_read_byte_ex( SRF02_DISTANCE_HI, 0x84, data );
	*data = 0x00;
	return SRF02_WRONG_REGISTER;
} //}}}


uint8_t srf02_ping( void )
{ //{{{
	return i2c_srf02_write_byte( SRF02_CMD_ADDR, SRF02_CMD_GET_RANGE_CM );
} //}}}


uint8_t i2c_srf02_read_byte_ex( uint8_t addr, uint8_t flags, uint8_t * data )
{ //{{{
	uint8_t status;
#ifndef _I2C_OFF

	status=i2c_start();
	if(status) {
		i2c_stop();
		return(status | 0x01);
	}
	
	status=i2c_sendAddr(SRF02_W, 0x00);
	if(status) {
		i2c_stop();
		return(status | 0x02);
	}

	status=i2c_sendData(addr);
	if(status) {
		i2c_stop();
		return(status | 0x03);
	}

	status=i2c_repeatStart();
	if(status) {
		i2c_stop();
		return(status| 0x04);
	}
	
	status=i2c_sendAddr(SRF02_R, 0x40); // TWEA required
	if(status) {
		i2c_stop();
		return(status | 0x05);
	}

//	status = i2c_receiveData( data, 0x84 ); // ( TWINT | TWEN ), and no TWEA, NACK as last data requested.
	status = i2c_receiveData( data, flags ); // ( TWINT | TWEN ), and no TWEA, NACK as last data requested.
	if(status) {
		i2c_stop();
		return(status | 0x06);
	}

	status = i2c_stop();
	if(status) {
		return(status | 0x07);
	}

#endif
	return(status);

} //}}}


uint8_t i2c_srf02_write_byte(uint8_t addr, uint8_t byte)
{ //{{{
#ifndef _I2C_OFF
	uint8_t status;

	status=i2c_start();
	if(status)
	{
		i2c_stop();
		return status;
	}
	
	status=i2c_sendAddr(SRF02_W, 0x00);
	if(status)
	{
		i2c_stop();
		return status;
	}

	status=i2c_sendData( addr );
	if(status)
	{
		i2c_stop();
		return status;
	}

	status=i2c_sendData(byte);
	if(status)
	{
		i2c_stop();
		return status;
	}

	i2c_stop();
#endif

	return 0;
} //}}}


#ifdef _SRF02_DEBUG
/* Debug function for SRF02 ranging sensor.
 * Debug function is sending the status information
 * to the callee. Based on that the callee can
 * fine tune the I2C connection details
 */
uint8_t srf02_get_version_ex( uint8_t * data )
{ //{{{
//	uint8_t temp;

	return i2c_srf02_read_byte_ex( SRF02_CMD_ADDR, 0x84, data );
//	_delay_us( 50 );
//	return i2c_srf02_read_byte_ex( SRF02_CMD_ADDR, &temp );
	/* This is a workaround for the probrlem with SRF02 range finder
	 * This device seems to have a bug in the implementation of the
	 * i2c protocol. It did not react on the STOP signal. The SDA
	 * line in low state and TWI implementation in ATmega8 reports
	 * 0xF8 -> no relevant information available or TWINT flag
	 * is not set. This information remains on the bus after sending STOP
	 * information over the bus. The workaround forces the bus error 
	 * and accepts then STOP signal. 
	 */


//	i2c_EEPROM_byte_write( (uint16_t) addr++, (uint8_t) cntr );
//	_delay_ms(10); // IMPORTANT: between write operations must be 10 ms delay.

//	*data = (uint8_t) i2c_EEPROM_byte_read( (uint16_t) 0 );
	/* This is a test. EEPROM is acting on STOP condition and releases
	 * the SDA line. This proves the TWI/I2C implementation is working
	 * fine, the problem is the protocol implementation in SRF02.
	 */

	return 1;
} //}}}

#endif //_SRF02_DEBUG


/* Following functiona are not required for curretn application.
 * They might be however required by another application
 * and therefroe I publish them here as examples.
 */
/*
uint8_t srf02_is_data_ready( void )
{ //{{{
	uint8_t temp = 255;

	if( SRF02_VERSION != i2c_srf02_read_byte( SRF02_CMD_ADDR, NACK )) {
		temp = SRF02_DATA_NOT_READY;
	} else {
		temp = SRF02_DATA_READY;
	}
//	_delay_us( 50 );
//	i2c_srf02_read_byte( SRF02_CMD_ADDR );
	return temp;
} //}}}


uint8_t srf02_get_version( void )
{ //{{{
	uint8_t temp;

	temp = i2c_srf02_read_byte( SRF02_CMD_ADDR, NACK );
//	_delay_us( 50 );
//	i2c_srf02_read_byte( SRF02_CMD_ADDR );
	return temp;
} //}}}

uint8_t srf02_get_range_hi( void  )
{ //{{{
	return i2c_srf02_read_byte( SRF02_DISTANCE_HI, NACK );
//	return i2c_srf02_read_byte( SRF02_MIN_DISTANCE_HI, NACK );
} //}}}


uint8_t srf02_get_range_lo( void )
{ //{{{
	return i2c_srf02_read_byte( SRF02_DISTANCE_LO, NACK );
//	return i2c_srf02_read_byte( SRF02_MIN_DISTANCE_LO, NACK );
} //}}}


uint8_t i2c_srf02_read_byte( uint8_t addr, ack_t ack )
{ //{{{
	uint8_t data = 0;
#ifndef _I2C_OFF
	uint8_t status;

	status=i2c_start();
	if(status) {
		i2c_stop();
		return 0;
	}
	
	status=i2c_sendAddr(SRF02_W, 0x00);
	if(status) {
		i2c_stop();
		return 0;
	}

	status=i2c_sendData(addr);
	if(status) {
		i2c_stop();
		return 0;
	}

	status=i2c_repeatStart();
	if(status) {
		i2c_stop();
		return 0;
	}
	
	status=i2c_sendAddr(SRF02_R, 0x40 ); //adding (1<<TWEA) flag
	if(status) {
		i2c_stop();
		return 0;
	}

	if( ack == ACK ) 
		data = i2c_receiveData_ACK();
	else
		data = i2c_receiveData_NACK();

	if(data == ERROR_CODE) {
		i2c_stop();
   	return 0;
  	}

	i2c_stop();
#endif

	return(data);
} //}}}

*/


/*
uint16_t srf02_get_range( void )
{
	uint16_t res;
	res = 0;

	if( SRF02_DATA_READY == srf02_data_ready() ) {
		_delay_ms(100);
		res = i2c_srf02_read_word( SRF02_DISTANCE_HI );
		_delay_ms( 100 );
		srf02_ping();
	}
	return res;
}


uint16_t i2c_srf02_read_word( uint8_t addr )
{
	uint8_t low, high;
	uint16_t data;

	high = (uint8_t) i2c_srf02_read_byte( addr++ );
	_delay_us(400);
	low = (uint8_t) i2c_srf02_read_byte( addr );

	data = (uint16_t) (high<<8);
 	data |= (uint16_t) low;

	return data;
}
*/
/* These fucntions can be used generally. They are not active in this
 * application, which is dictated by the specific HW design. The SRF02
 * device is serviced in combination with servo shiled driver and the 
 * multiple interrupts generated by servo shield and I2C traffic 
 * can produce time conflicts and protocol problems. Therefore
 * this application utilizes byte-wise access synchronized with 
 * compare match interrupt handler.
 */

