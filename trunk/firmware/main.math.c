//#include <avr/io.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <util/delay.h>

#include "hexapod.h"

double result;
unsigned char sreg;
uint8_t	oop;
servos_t	servos;
double step;
angles_t	uhly;
uint8_t i;
uint8_t buffer[32];

/* 
 * Inicializace struktur
 */

//uint8_t sampling_rate_convertor[] EEMEM = {43, 22, 12, 9, 7, 6, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0};

//static display_data_t voltage = {{3345,3155,2964,2774,2584,2393},{8,9,10,11,12,13},1,0,6} ;

volatile uint16_t servo_table[12];
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
	step = 0;
	
	arms_init();

	servo_shield_port_init();
	servo_shield_init();
//	servo_shield_enable();

#ifdef USART_DEBUG
//{{{
	USART_init();
#endif
//}}}

	sei();

	while(1) {
		switch( oop ) 
		{
		case 'f':
			break;

		default:
			break;
		}

#ifdef USART_DEBUG
//{{{
		switch( usart_data )
		{
			case 'v':
				for( i = 0; i < 6; i++ ) {
//					sprintf_P( (char*) buffer, PSTR("x=%d,c=%d,Gama=%d,Beta=%d\r\n"), (uint8_t) step, (uint8_t) uhly.s, (uint8_t) (uhly.gama * 100), (uint8_t) (uhly.beta*100));
//					USART_send_str( (uint8_t*) buffer );
					sprintf_P( (char*) buffer, PSTR("x=%d, Servo=%d: rameno=%d - loket=%d\r\n"), (uint8_t) step, i, (uint16_t) servos.arm[i].servo[0].angle, (uint16_t) servos.arm[i].servo[1].angle );
					USART_send_str( (uint8_t*) buffer );
				}
/*
				sprintf_P( (char*) buffer, PSTR("Bank0: %d,%d,%d,%d,%d,%d\r\n"), servo_table[0],servo_table[1],servo_table[2],servo_table[3],servo_table[4], servo_table[5]);
				USART_send_str( (uint8_t*) buffer );
				sprintf_P( (char*) buffer, PSTR("Bank1: %d,%d,%d,%d,%d,%d\r\n"), servo_table[6],servo_table[7],servo_table[8],servo_table[9],servo_table[10], servo_table[11]);
				USART_send_str( (uint8_t*) buffer );
*/
				usart_data = 0x0;
				break;

			case 'p':
				step = step +	10;
				usart_data = 0x0;
				break;

			case 'm':
				step = step -	10;
				usart_data = 0x0;
				break;
		
			default:
				break;
		}
#endif
//}}}

		uhly = angles( (double) step, (arm_t*) &(servos.arm[servos.index]) );
		servos.arm[servos.index].servo[0].angle = servo_ctrl( (double) uhly.gama + (double) servos.arm[servos.index].gama_0 );
		servos.arm[servos.index].servo[1].angle = servo_ctrl( (double) uhly.beta + (double) BETA_0 );

#ifdef USART_DEBUG
//{{{
		sprintf_P( (char*) buffer, PSTR("x=%d,i=%d,c=%d,G=%d,B=%d\r\n"), (uint8_t) step, servos.index, (uint8_t) uhly.s, (uint8_t) (uhly.gama * 100), (uint8_t) (uhly.beta*100));
		USART_send_str( (uint8_t*) buffer );
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

	servo_table[11] = servos.arm[0].servo[0].angle;
	servo_table[0] = servos.arm[0].servo[1].angle;
	servo_table[1] = servos.arm[1].servo[0].angle;
	servo_table[2] = servos.arm[1].servo[1].angle;
	servo_table[3] = servos.arm[2].servo[0].angle;
	servo_table[4] = servos.arm[2].servo[1].angle;
	servo_table[5] = servos.arm[3].servo[0].angle;
	servo_table[6] = servos.arm[3].servo[1].angle;
	servo_table[7] = servos.arm[4].servo[0].angle;
	servo_table[8] = servos.arm[4].servo[1].angle;
	servo_table[9] = servos.arm[5].servo[0].angle;
	servo_table[10] = servos.arm[5].servo[1].angle;
	/* Servo buffer is rotated one byte left. 
	 */
	SREG = sreg;
}


void arms_init( void )
{
	// Prava predni noha (a=104, alpha = 30);
	servos.arm[0].gama_0 = 0.523598776;
	servos.arm[0].a = 104;

	servos.arm[1].gama_0 = 1.04719755;
	servos.arm[1].a = 104;

	servos.arm[2].gama_0 = 1.30899694;
	servos.arm[2].a = 54;

	servos.arm[3].gama_0 = 0.523598776;
	servos.arm[3].a = 104;

	servos.arm[4].gama_0 = 1.04719755;
	servos.arm[4].a = 104;

	servos.arm[5].gama_0 = 1.30899694;
	servos.arm[5].a = 54;

	servos.index = 0;
	servo_index = 0;

	servo_table[0] = 2249;
	servo_table[1] = 2249;
	servo_table[2] = 2249;
	servo_table[3] = 2249;
	servo_table[4] = 2249;
	servo_table[5] = 2249;
	servo_table[6] = 2249;
	servo_table[7] = 2249;
	servo_table[8] = 2249;
	servo_table[9] = 2249;
	servo_table[10] = 2249;
	servo_table[11] = 2249;
}
