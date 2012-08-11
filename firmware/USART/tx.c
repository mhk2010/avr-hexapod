
/*
 *****************************************************************************
 *
 * File Name    : 'tx.c'
 * Title        : Transmiter USART routines 
 * Author       : Martin Metal
 * Date         : 10/02/2010
 * Revised      : 17/03/2010
 * Version      : 1.0
 * Target MCU   : Atmel AVR Mega8
 * Editor Tabs  : 3
 * Description  : The USART routines for sending the data packs
 *                The initialization expicilty activates the Transmitter
 *                only, avoid to activate the Receiver part and interrupt.
 *                The Transmitter is using the TX pin on Atmega, the RX
 *                pin is not used. By activating USART completely the 
 *                USART does not work, while the RX signal is not present. 
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


/*
 * Initialization for the TX modul
 * avoids activating the RX part.
 * By activating the RX the USART expects
 * the signal line RX exist. It cannot
 * be satisfied for the TX modul
 */
void USART_tx_init()
{
#ifndef _SIMULATION
	//Set baud rate
	UBRRL=(uint8_t) UBRRVAL;		//low byte
	UBRRH=(uint8_t) (UBRRVAL>>8);	//high byte
	//
	//Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);	
	
	//Enable Transmitter only, do not activate interrupt and Receiver. 
	UCSRB=(1<<TXEN);
	
	//enable global interrupts
	set_sleep_mode(SLEEP_MODE_IDLE);
#endif
}


void USART_send_byte( uint8_t b )
{
	loop_until_bit_is_set( UCSRA, UDRE );
	UDR=(uint8_t) b;
}


void USART_send_word( uint16_t data )
{
	USART_send_byte( (uint8_t) data );
	USART_send_byte( (uint8_t) (data>>8));	
}
