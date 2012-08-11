//#include <avr/io.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <util/delay.h>

#include "hexapod.h"

unsigned char sreg;
uint8_t	oop;
servos_t	servos;
uint16_t step;
angles_t	uhly;
uint8_t i;
uint8_t buffer[32];

/* 
 * Inicializace struktur
 */

volatile uint16_t servo_table[12] = {2429, 2429, 2429, 2429, 2429, 2429, 2429, 2429, 2429, 2429, 2429, 2429} ;
volatile uint8_t	servo_index;
volatile ocr1a_t ocr1a;

#ifdef USART_DEBUG
//{{{
volatile uint8_t usart_data;

/* USART Interrupt handler
 * -----------------------
 * This routine handles the interrupt flag "USART RX" signal.
 * Fetches the character, check the framing error
 * and evaluates the received byte. Either sends it 
 * to LCD for display or performs an LCD instruction
 */
ISR(USART_RXC_vect)
{
	/*
	uint8_t sreg = SREG;

	cli();
	
	usart_data = USART_receive_byte();

	SREG = sreg;
	sei();
	*/

	uint8_t c;

	//Store data to temp
	c = UDR;   
	// Check the framing error, defect frames are skipped and must be re-transmitted.
	if (bit_is_clear(UCSRA, FE)) {
		usart_data = c;
	} else {
		USART_send_str_p ( (uint8_t*) PSTR("Transmission Error\r\n"));
	}
}

#endif //USART_DEBUG
//}}}

ISR(TIMER1_COMPA_vect)
{
	ocr1a.cntr += 1;
	ocr1a.cntr %= COUNT_OF_ARMS;

	if( 0 == ocr1a.cntr ) {
		ocr1a.bank ^= 1;
		if( ocr1a.bank ) {
			SERVOSHIELD_CTRL |= _BV(MR_BANK_1);
			SERVOSHIELD_CTRL &= ~_BV(MR_BANK_0);
		} else {
			SERVOSHIELD_CTRL |= _BV(MR_BANK_0);
			SERVOSHIELD_CTRL &= ~_BV(MR_BANK_1);
		}
	} 

	SERVOSHIELD_CTRL |= _BV(CP0);
	_delay_us(2);
	OCR1A = (uint16_t) servo_table[servo_index];
	servo_index += 1;
	servo_index %= COUNT_OF_SERVOS;
	SERVOSHIELD_CTRL &= ~_BV(CP0);
// shodi CP1 signal znova na nulu po 2us.
}


int main()
{
#ifdef USART_DEBUG
//{{{
	usart_data = 0x0;
#endif
//}}}

	ocr1a.cntr = 0;
	ocr1a.bank = 1;
	step = 2249;
	
	arms_init();

	servo_shield_port_init();
	servo_shield_init();
	servo_shield_enable();

#ifdef USART_DEBUG
//{{{
	USART_init();
#endif
//}}}

	sei();

	while(1) {

#ifdef USART_DEBUG
//{{{
		switch( usart_data )
		{
			case 'v':
				sprintf_P( (char*) buffer, PSTR("OCR1A=%d\r\n"), step );
				USART_send_str( (uint8_t*) buffer );
				usart_data = 0x0;
				break;

			case 'p':
				step += 10; 
				usart_data = 0x0;
				break;

			case 'm':
				step -=10; 
				usart_data = 0x0;
				break;
		
			default:
				break;
		}
#endif
//}}}

		servos.index += 1;
		servos.index %= COUNT_OF_ARMS;

		if( servos.index == 0 ) {
			// servo.index = 0 is the flag "DIRTY=false". The buffer
			// is ready and can be filled in the servo control latch
			fill_servo_buffer();
		}

		sleep_mode();
	}

	return 0;
}

void servo_shield_port_init() 
{
	SERVOSHIELD_DDR |= _BV(MR_BANK_0) | _BV(MR_BANK_1) | _BV(CP0);
/* Configure the data direction for servo port (B).
 * Pin MR (PB0) and PB1 (CP0) are configured for output
 */
}

void servo_shield_init( void )
{
	unsigned char sreg;

//   TCCR1A |= _BV(COM1A0);
//   TCCR1A |= _BV(COM1A0) | _BV(COM1A1);
// configure "set OC1A output high on compare match"
	TCCR1A &= ~_BV(COM1A0);
	TCCR1A &= ~_BV(COM1A1);
// confiure OC1A output pins for normal operation

	TCCR1B = _BV(CS11) | _BV(WGM12);
/* CS11 - use CLK/8 prescale value
 * WGM1[3-0] = [0,1,0,0] = CTC mode.
 * Generate interrupt on T1 = OCR1A, set output pin high
 * and clear the counter. Start counting from zero until
 * the OCR1A value is reached again.
 */

	SERVOSHIELD_CTRL |= _BV(MR_BANK_0);
	SERVOSHIELD_CTRL |= _BV(MR_BANK_1);
	_delay_us(10);
	SERVOSHIELD_CTRL &= ~_BV(MR_BANK_0);
//	SERVOSHIELD_CTRL &= ~_BV(MR_BANK_1);
// Send 10us strobe to 4017 - decade counter - to reset
	 
	sreg = SREG;
	cli();
	
	TCNT1 = (uint16_t) 0;
	// Set the Timer/counter1 to zero

	OCR1A = (uint16_t) 2249;
	/* Set the compare register counter to prescalled value. 
	 *
	 * The compae match limit is set dynamically in interrupt 
	 * handler. It generates the control signals for attached
	 * servos.
	 */

	SREG = sreg;
}


void servo_shield_disable( void )
{
	TIMSK &= ~_BV(OCIE1A);
	// Disable interrupt called by compare match.
}


void servo_shield_enable( void )
{
	TIMSK |= _BV(OCIE1A);
	// Enable interrupt called by compare match.
}

void fill_servo_buffer( void )
{
	sreg = SREG;
	cli();

	servo_table[11] = step;
	/* Servo buffer is rotated one byte left. 
	 */
	SREG = sreg;
}
