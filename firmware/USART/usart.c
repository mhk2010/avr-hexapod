/*
 *****************************************************************************
 *
 * File Name    : 'usart.c'
 * Title        : Generic USART fucntions
 * Author       : Martin Metal
 * Date         : 10/02/2010
 * Revised      : 17/03/2010
 * Version      : 1.0
 * Target MCU   : Atmel AVR Mega8
 * Editor Tabs  : 3
 * Description  : Implements the full initialization of USART 
 *                The full initialization is required for debugging purposes,
 *                when the debug information is sent over the USART to PC. The
 *                communicaiton is birectional and therefore both receiver
 *                and transmitter modul of the USART block must be initialized 
 *
 ******************************************************************************
 */
#include "usart.h"

/*
 * Full initialization of the USART interface. It activates
 * both receiver and transmitter part of the module. It is
 * required for bi-directional data trasmission. The radiolink
 * offers on-directional data transmission only and therefore 
 * is required to use different initialization routime for
 * transmitter and receiver
 */
void USART_init()
{
#ifndef _SIMULATION
	//Set baud rate
	UBRRL=(uint8_t) UBRRVAL;		//low byte
	UBRRH=(uint8_t) (UBRRVAL>>8);	//high byte
	//
	//Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);	
	
	//Enable Transmitter and Receiver and Interrupt on receive complete
	UCSRB=(1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	
	//enable global interrupts
	set_sleep_mode(SLEEP_MODE_IDLE);
#endif
}
