#ifndef __USART_H__
#define __USART_H__

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>    
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/sfr_defs.h>

/*
 * The baud rate for the radio link should not
 * exceed 1200 baud, for security reasons 
 * I recommend to stay by 600 baud. That 
 * is still reasonably fast for the given
 * application and should produce stable signal
 */

//#define BAUDRATE 600
#define BAUDRATE 19200

//calculate UBRR value
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)


	/*************/
	/* functions */
	/*************/

void USART_init( void );
/* Full intiialization of the USART module.
 */

void USART_tx_init( void );
/* Initialize the transmitter module only
 */

void USART_send_str_P( uint8_t * );
void USART_send_str( uint8_t *  );
void USART_send_byte( uint8_t );
void USART_send_word( uint16_t );
uint8_t USART_receive_byte(void);
/* USART manipulation routines for sending
 * and retrieving data. 
 */

#endif
