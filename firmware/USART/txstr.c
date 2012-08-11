
/*
 *****************************************************************************
 *
 * File Name    : 'txstr.c'
 * Title        : Transmiter USART routines 
 * Author       : Martin Metal
 * Date         : 10/02/2010
 * Revised      : 17/03/2010
 * Version      : 1.0
 * Target MCU   : Atmel AVR Mega8
 * Editor Tabs  : 3
 * Description  : The USART routines for sending the strings. 
 *
 ******************************************************************************
 */

/*
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
*/

#include "usart.h"

// Send the string defined in the program memory (static strigs)
void USART_send_str_P ( uint8_t * value )
{
	char tmp = '*';

#ifndef _SIMULATION

	for ( tmp = pgm_read_byte((const char*) value); tmp; ++value, tmp = pgm_read_byte((const char*) value) )
	{
		while ( !( UCSRA & (1<<UDRE)) );
		UDR=tmp;
	}
#endif
}


// Sends the string defined in the SRAM memory
void USART_send_str ( uint8_t * value )
{
#ifndef _SIMULATION
	while( *value )
	{
		while ( !( UCSRA & (1<<UDRE)) );
		UDR=(uint8_t) *value++;
	}
#endif
}


