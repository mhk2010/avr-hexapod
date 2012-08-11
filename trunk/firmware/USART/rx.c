/*
 *****************************************************************************
 *
 * File Name    : 'rx.c'
 * Title        : Receiver module
 * Author       : Martin Metal
 * Date         : 10/02/2010
 * Revised      : 17/03/2010
 * Version      : 1.0
 * Target MCU   : Atmel AVR Mega8
 * Editor Tabs  : 3
 * Description  : Implements the Receiver USART Module.
 *                Receiver requires one generic function only for receiving
 *                one byte at time. 
 *
 ******************************************************************************
 */

#include "usart.h"

/*
 * Receiving a byte from USART line can be also achieved
 * in non-blocking mode by implementing interrupt handler
 * in the main code The interrupt handler example:
 *
 *		volatile uint8_t usart_data;
 *
 *		ISR(USART_RXC_vect)
		{
			uint8_t sreg = SREG;

			cli();
	
			usart_data = USART_receive_byte();

			SREG = sreg;
			sei();
		}
 */



uint8_t USART_receive_byte( void )
{
	uint8_t c;
	
	// Wait until a byte has been received
	// while((UCSRA&(1<<RXC)) == 0);
	// while( bit_is_clear( UCSRA, RXC ));
	loop_until_bit_is_set( UCSRA, RXC );

	// Return received data
	c = UDR;  

	if (bit_is_set(UCSRA, FE)) {
		c = 0xFF;
	// USART_send_str_p ( (uint8_t*) PSTR("Transmission Error\r\n"));
	}

	return c;
}
