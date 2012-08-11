#include "hexapod.h"

// *** Variable Declarations
// {{{
unsigned char sreg;
uint8_t	oop;
uint8_t step_last_known;

speed_t speed;
srf02_t srf02_data;

volatile srf02_cntrl_t srf02_cntrls;
volatile ocr1a_t servo_cntrls;
volatile servo_cntrl_t servo;
volatile move_t move;

volatile uint8_t semaphore;
//}}}

#ifdef MATH
//{{{
angles_t	uhly;
#endif
//}}}

#ifdef USART_DEBUG
//{{{
uint8_t i;
uint8_t buffer[32];
uint8_t servo_trimming_index = 10;
uint8_t usart_manual_cntrl = 0;
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
		USART_send_str_P ( (uint8_t*) PSTR("Transmission Error\r\n"));
	}
}

#endif //USART_DEBUG
//}}}

ISR(TIMER1_COMPA_vect)
/* Interrupt handler for compare match.  The controller fires this interrupt
 * every time the internal counter reaches the pre-set value (OCR1A). The
 * handler than sends the control signal to a selected servo by strobing the
 * clock signal on the servo shield bank. The time delay between tha last
 * strobe and the current one defines the length of the control pulse for a
 * servo. The handler manages bank switching in servo shield. Every bank
 * connects 6 servos only, the robot requires 12 servos to move.
 */
{ //{{{
	servo_cntrls.cntr += 1;
	servo_cntrls.cntr %= COUNT_OF_ARMS;

	if( 0 == servo_cntrls.cntr ) {
		servo_cntrls.bank ^= 1;
		if( servo_cntrls.bank ) {
			SERVOSHIELD_CTRL |= _BV(MR_BANK_1);
			SERVOSHIELD_CTRL &= ~_BV(MR_BANK_0);
		} else {
			SERVOSHIELD_CTRL |= _BV(MR_BANK_0);
			SERVOSHIELD_CTRL &= ~_BV(MR_BANK_1);
		}
	} 
	/* First task: swap the servo shield bank every 6 servos. 
	 * Each johnson counter control 6 servos only. 
	 */

	SERVOSHIELD_CTRL |= _BV(CP0);
	_delay_us(2);
#ifdef _TEST_MODE
	OCR1A = (uint16_t) 2207;
#else
	OCR1A = (uint16_t) (servo.current[servo_cntrls.servo_index] > 0 ? servo.current[servo_cntrls.servo_index] : 2207);
#endif
	servo_cntrls.servo_index += 1;
	servo_cntrls.servo_index %= COUNT_OF_SERVOS;
	if( 0 == servo_cntrls.servo_index ) {
		servo_cntrls.dirty ^= 1;
	}
	SERVOSHIELD_CTRL &= ~_BV(CP0);
	/* Second task: strobe the CP0 pin of 4017 for minimum 2us. 
	 * load the control byte from latch buffer into register OCR1A.
	 * Invalidate the latch buffer content after emptying latch buffer.
	 * by setting the "dirty" flag in servo_cntrls structure.
	 */

	if( ENABLE == srf02_cntrls.status ) {
		srf02_cntrls.status = DISABLE;
		srf02_cntrls.semaphore = SIGNALED;
	}
	/* Third task: unblock access to srf02 device via I2C bus
	 */

} //}}}

ISR(TIMER0_OVF_vect)
/* Interrupt handler for Timer0 overflow. This timer is used to define the 
 * timebase for derivation of the distance data deliverred by the SRF02 device. 
 * Derivation is used to determine the character of the obstacle detected by the
 * vision system of the robot. This handler trigers in fixed time intervals
 * (constant time base)
 *
 * The timer handler activates the routine for ranging.
 */
{	//{{{
	if( srf02_cntrls.reading_delay_cntr++ > srf02_cntrls.reading_delay_lim ) {
		srf02_cntrls.reading_delay_cntr = 0;
		srf02_cntrls.status = ENABLE;
	}

}	//}}}

int main() // ******************** M A I N ****************************
{ //{{{
#ifdef USART_DEBUG
//{{{
	usart_data = 0x0;
#endif
//}}}

	init();
	fill_servo_page( (move_t) move );  							// Init the future structure
	fill_servo_page( (move_t) move );  							// Init the past structure
	fill_servo_buffer( (move_t) move, (speed_t) speed );  // Init the current structure

	_delay_ms(1000); // Hold on for 1 second to establish stable main

	i2c_init(I2C_SLOW);			// Initialize bus for ranging device
	srf02_init();					// Initialize HW for ranging
	srf02_cntrl( DISABLE ); 	// Stop ranging

	servo_shield_port_init();		// Initialize HW for servo shield
	servo_shield_init();				// Initialize servo shield
	servo_shield_cntrl( ENABLE );	// enable servo shield control circuit

	wdt_enable(WDTO_1S); // restart the processor if something hangs for longer than 1 second.

#ifdef USART_DEBUG
//{{{
	USART_init();
	_delay_ms(200);
	USART_send_str_P( (uint8_t*) PSTR("START\r\n" ));
#endif
//}}}

	sei();

#ifndef USART_DEBUG 
// {{{ Automatically activate ranging when not in debug mode.
		srf02_cntrl( ENABLE );
#endif // }}}

	while(1) {
#ifdef USART_DEBUG //{{{
		switch( usart_data )
		{
			case 'i':
#ifndef USART_DEBUG_SMALL_SIZE
				sprintf_P( (char*) buffer, PSTR("step=%d\r\n"), move.stepper_current );
				USART_send_str( (uint8_t*) buffer );
#else				
				USART_send_str_P( (uint8_t*) PSTR("step="));
				itoa( (int) move.stepper_current, (char*) buffer, 10 );
				USART_send_str( (uint8_t*) buffer );
				USART_send_str_P( (uint8_t*) PSTR("\r\n"));
#endif // USART_DEBUG_SMALL_SIZE

				for( i=0; i<COUNT_OF_SERVOS;i++ ) {
#ifndef USART_DEBUG_SMALL_SIZE
					sprintf_P( (char*) buffer, PSTR("Servo=%d,ORC1A=%d\r\n"), i, servo.current[i] );
					USART_send_str( (uint8_t*) buffer );
#else
					USART_send_str_P( (uint8_t*) PSTR("Servo="));
					itoa( (int) i, (char*) buffer, 10 );
					USART_send_str( (uint8_t*) buffer );
					USART_send_str_P( (uint8_t*) PSTR(",OCR1A="));
					itoa( (int) servo.current[i], (char*) buffer, 10 );
					USART_send_str( (uint8_t*) buffer );
					USART_send_str_P( (uint8_t*) PSTR("\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
				}
				break;

			case 'N':
				servo_trimming_index += 1;
				servo_trimming_index %= COUNT_OF_SERVOS;
				USART_send_str_P( (uint8_t*) PSTR("trimmed servo="));
				itoa( (int) servo_trimming_index, (char*) buffer, 10 );
				USART_send_str( (uint8_t*) buffer );
				USART_send_str_P( (uint8_t*) PSTR("\r\n"));
				break;

			case '!':
				for( i=0; i<COUNT_OF_SERVOS;i++ ) {
					servo.current[i] = 2207;
				}
				break;

			case '+':
					servo.current[servo_trimming_index] += 6;
				break;

			case '-':
					servo.current[servo_trimming_index] -= 6;
				break;

			case 'R':
				srf02_cntrl( ENABLE );
				break;

			case 'Q':
				srf02_cntrl( DISABLE );
				break;

			case 'm':
				usart_manual_cntrl ^= 1;
				break;
		
			case 'f':
				move.cntrl = FORWARD;
				break;
		
			case 'r':
				move.cntrl = REVERSE;
				break;
		
			case 's':
				move.cntrl = STOP;
				break;

			case 't':
				move.movement_request = REQUEST_CURVED;
		
			default:
				break;
		}

		usart_data = 0x0;
#endif
//}}}

		if( srf02_cntrls.semaphore ) {
//{{{
			srf02_cntrls.semaphore ^= SIGNALED;

			memcpy_P( &srf02_data.onmessage, (PGM_VOID_P) &VTable[srf02_cntrls.app_vector], 3 );
			srf02_cntrls.app_vector = srf02_data.onmessage.ret;
			srf02_cntrls.app_vector += srf02_data.onmessage.call();
#ifdef USART_TRACING // {{{ TODO ************************************************************************
			sprintf_P( (char*) buffer, PSTR("Fx=%d\r\n"), srf02_cntrls.app_vector );
			USART_send_str( (uint8_t*) buffer );
#endif //}}}
		}
		/* SRF02 control block:
		 * Communication with the SRF02 device is synchronized
		 * with the other interrupts in this application. This code
		 * performs single operation on SRF02 device in time window between
		 * two interrupts only. Each operation is performed as a dynamic 
		 * procedure retrieved from VTable. VTable has 5 handlers. 
		 * The return code of each handler determines the 
		 * code of the next called handler. Each handler is called in 2ms
		 * intervals (main interrupt) and the process is blocked by semaphore.
		 * The semaphore let the trafic per defined time, minimum is 3 seconds.
		 * Maximum is 39 secods (the robot performs full turn without checking
		 * the distance.
		 */
//}}}

		if( servo_cntrls.dirty ) {
			//{{{
			servo_cntrls.dirty ^= SIGNALED;
#ifdef USART_TRACING // {{{
			itoa( (int) speed.index, (char*) buffer, 10 );
			USART_send_str( (uint8_t*) buffer );
			USART_send_str_P( (uint8_t*) PSTR("->"));
#endif // }}}

			if( speed.index < speed.max ) {
				speed.index += 1;
				fill_servo_buffer( (move_t) move, (speed_t) speed );
			} else {
				if( move.cntrl != STOP ) {
					if( move.stepper_current == 0 ) {
						/* TODO: this mechanism combined with the move structure
						 * does not work well. Registering the cntrl is not efficient,
						 * modifying directly move.cntrl is breaking the physicvs again.
						 * It required complete review of the moves when 4 banks are
						 * involved, especially if the the lwo high banks support just
						 * one way thorugh put
						 */
						swap_bank_servo_page();
						srf02_cntrl( ENABLE );
					}
					fill_servo_page( (move_t) move );
					speed.index = 0;
				}
			  	if( move.cntrl == REVERSE ) {
			  		if( move.stepper_current == 0 ) {
			  			move.stepper_current = STEP_TAB_SIZE;
			  		} 
			  	}
			  	move.stepper_current += move.cntrl; 
			  	move.stepper_current %= STEP_TAB_SIZE;

#ifdef USART_DEBUG // {{{
				if( 1 == usart_manual_cntrl ) {
					move.cntrl = STOP;
				}
#endif //}}}
#ifdef USART_TRACING // {{{
			  	USART_send_str_P( (uint8_t*) PSTR("+"));
#endif //}}}
			}
			/* The movement backbone. 
			 * This block gets executed every time the servo shield processes
			 * the complete latch buffer. The servo shield requests new positions
			 * and this block delivers them:
			 *
			 *  1. calculates linear regression between past and future
			 *     position. (between two tabulated entries)
			 *
			 *  2. It copies the next block of tabulated entries from 
			 *     program memory into future buffer.
			 *
			 *  3. It performs a step. The data for the step is calculated
			 *     in cooperation with input from the ultrasonic sonar.
			 *
			 * Movement uses tabulated entries - 18 per one cyclic movemement
			 * of an arm. Additionally, the linear regression is used 
			 * to determine position between two tabulated entries. The parameter
			 * "speed" control the number of substeps between the tabelated
			 * entries. Movemement is autonomous and takes input from vision
			 * system. This block control all available movements:
			 *
			 *  STRAIGHT - REVERSE
			 *  STRAIGHT - STOP
			 *  STRAIGHT - FORWARD
			 *  CURVED - REVERSE -> RIGHT
			 *  CURVED - STOP
			 *  CURVED - FORWARDS -> LEFT
			 */
		}
		//}}}

		if( srf02_cntrls.dirty  ) {
//{{{
			srf02_cntrls.dirty ^= SIGNALED;

			process_ranging_data();
			memcpy_P( &srf02_data.onmessage, (PGM_VOID_P) &MTable[determine_next_move(get_app_vector( srf02_data, move, srf02_cntrls ))], 3 );
			srf02_data.onmessage.call();

			srf02_cntrls.app_vector = MSG_PING;
			srf02_cntrls.error_code = 0x00;
		}
		/* This section evaluates the sonar data and determines move direction.
		 * Detailed algorithm is describe un hexapod.h
		 */
//}}}

		wdt_reset();
		sleep_mode(); // may not be required, as the robot will repeatedly search for direction?
	}

	return 0;
} //}}}

signal_t semaphore_signal( resource_id_t res )
{ //{{{
	if( (_BV(res)) == (semaphore & _BV(res))) {
		return (signal_t) SIGNAL;
	}
	else {
		semaphore |= _BV(res);
	}
	return (signal_t) CLEAR;
} //}}}

void semaphore_signal_all( uint8_t preset )
{ //{{{
	semaphore = preset;
} //}}}

void semaphore_release( resource_id_t res )
{ //{{{
	semaphore &= ~_BV(res);
} //}}}

void servo_shield_port_init() // ********* F U N C T I O N S ********* 
{ //{{{
	SERVOSHIELD_DDR |= _BV(MR_BANK_0) | _BV(MR_BANK_1) | _BV(CP0);
/* Configure the data direction for servo port (B).
 * Pin MR (PB0) and PB1 (CP0) are configured for output
 */
} //}}}

void servo_shield_init( void )
{ //{{{
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

	OCR1A = (uint16_t) 2207;
	/* Set the compare register counter to prescalled value. 
	 *
	 * The compae match limit is set dynamically in interrupt 
	 * handler. It generates the control signals for attached
	 * servos.
	 */

	SREG = sreg;
} //}}}

void servo_shield_cntrl( type_t mode )
{ //{{{
	switch( mode ) 
	{
		case DISABLE:
			TIMSK &= ~_BV(OCIE1A);
			// Disable interrupt called by compare match.
		break;

		case ENABLE:
			TIMSK |= _BV(OCIE1A);
			// Enable interrupt called by compare match.
		break;
	}
} //}}}

void srf02_init( void )
{ //{{{
	unsigned char sreg;

	TCCR0 = _BV(CS02) | _BV(CS00); 
	/* Timer0: Prescaller set to clk/1024.
	 * 12 000 000 / 1024 / 256 (8-bit counter) = 45.8 
	 * The counting limit in interrupts is set to 135 (3 seconds)
	 * The counting limit in interrupts is set to 180 (4 seconds)
	 * This defines the base for derivation.
	 */

	sreg = SREG;
	cli();
	
	TCNT0 = (uint8_t) 0;
	// Set the Timer/counter0 to zero

	SREG = sreg;
} //}}}

void srf02_cntrl( type_t mode )
{ //{{{
	switch( mode ) 
	{
		case DISABLE:
			TIMSK &= ~_BV(TOIE0);
			// Disable interrupt called by overflow on Timer0
		break;

		case ENABLE:
			TIMSK |= _BV(TOIE0);
			// Enable interrupt called by overflow on Timer0
		break;
	}
} //}}}

void fill_servo_page( move_t m )
{ //{{{
	uint8_t i;
	uint8_t servo_latch_ptr;

	servo_latch_ptr = 10;


	sreg = SREG;
	cli();
	// Disable interrupts while writing the latch buffer. 
	
	for( i = 0; i < COUNT_OF_ARMS; i++ ) {
		servo_latch_ptr += 1;
		servo_latch_ptr %= COUNT_OF_SERVOS;
#ifdef _PROGMEM_DATA
		servo.past[servo_latch_ptr] = servo.future[servo_latch_ptr];
		servo.future[servo_latch_ptr] = (uint16_t) pgm_read_word( &run[m.movement].step[m.stepper_current].arm[i].servo_shoulder );
#else //{{{
		servo.past[servo_latch_ptr] = servo.future[servo_latch_ptr];
		eeprom_busy_wait();
		servo.future[servo_latch_ptr] = (uint16_t) eeprom_read_word( &run[m.movement].step[m.stepper_current].arm[i].servo_shoulder );
#endif
//}}}
		 
		servo_latch_ptr += 1;
		servo_latch_ptr %= COUNT_OF_SERVOS;

#ifdef _PROGMEM_DATA
		servo.past[servo_latch_ptr] = servo.future[servo_latch_ptr];
		servo.future[servo_latch_ptr] = (uint16_t) pgm_read_word( &run[m.movement].step[m.stepper_current].arm[i].servo_elbow );
#else //{{{
		servo.past[servo_latch_ptr] = servo.future[servo_latch_ptr];
		eeprom_busy_wait();
		servo.future[servo_latch_ptr] = (uint16_t) eeprom_read_word( &run[m.movement].step[m.stepper_current].arm[i].servo_elbow );
#endif
//}}}
	}
	/* Fill the servo latch buffer. Apply rotation by 1 byte left, so that the slot No.12 contain
	 * the OCR1A data for first servo. The slot No.1 than contain value for second servo, etc.
	 */ 

	SREG = sreg;
} //}}}

void fill_servo_buffer( move_t m, speed_t s )
{ //{{{
	uint8_t i;
	int16_t temp;

	sreg = SREG;
	cli();
	// Disable interrupts while writing the latch buffer. 
	
	for( i = 0; i < COUNT_OF_SERVOS; i++ ) {
		temp = servo.future[i] - servo.past[i]; 
      temp *= s.index;
		temp /= s.max;
      temp += servo.past[i];
		servo.current[i] = (uint16_t) temp;
	}
	/* Fill the servo latch buffer based on the linear regression 
	 * beween two tabelated tabeleated entries. It already works
	 * with shifted buffer ( one byte left ).
	 */ 

	SREG = sreg; // enable interrupts again.
} //}}}

void swap_bank_servo_page( void )
{ /// {{{
	uint8_t r;

	r  = (  move.movement == STRAIGHT ? 1 : 0 );
	r |= (( move.movement == REQUEST_CURVED ? 1 : 0 )<<BIT_MOV_REQUEST_CURVED);
	r |= (( move.movement == CURVED ? 1 : 0)<<BIT_MOV_CURVED);
	r |= (( move.movement == REQUEST_STRAIGHT ? 1 : 0 )<<BIT_MOV_REQUEST_STRAIGHT);

	r |= (( move.movement_request == REQUEST_CURVED ? 1 : 0 )<<BIT_CNTRL_REQUEST_CURVED);
	r |= (( move.movement_request == CURVED ? 1 : 0)<<BIT_CNTRL_CURVED);
	r |= (( move.movement_request == REQUEST_STRAIGHT ? 1 : 0 )<<BIT_CNTRL_REQUEST_STRAIGHT);
	r |= (( move.movement_request == STRAIGHT ? 1 : 0 )<<BIT_CNTRL_STRAIGHT);
	/* Calculate the bit array of the movement status/requests
	 */

	switch( r )
	{
		case 0x11: // STRAIGHT -> REQUESTING_CURVED
			semaphore_signal_all( 0xFF );
			move.movement = REQUEST_CURVED;
			move.movement_request = CURVED;
//			move.cntrl_request = move.cntrl;
			move.cntrl = FORWARD;
			break;

		case 0x22: // REQUESTING_CURVED -> CURVED
			move.movement = CURVED;
			move.cntrl = move.cntrl_request;
			semaphore_release( CNTRL_REQUESTED );
			semaphore_release( MOVEMENT_REQUESTED );
//			semaphore_release( CNTRL );
//			srf02_cntrl( ENABLE );
			break;

		case 0x44: // CURVED -> REQUESTING_STRAIGHT
			semaphore_signal_all( 0xFF );
			move.movement = REQUEST_STRAIGHT;
			move.movement_request = STRAIGHT;
//			move.cntrl_request = move.cntrl;
			move.cntrl = FORWARD;
			break;

		case 0x88: // REQUESTING_STRAIGHT -> STRAIGHT
			move.movement = STRAIGHT;
			move.cntrl = move.cntrl_request;
			semaphore_release( CNTRL_REQUESTED );
			semaphore_release( MOVEMENT_REQUESTED );
//			semaphore_release( CNTRL );
//			srf02_cntrl( ENABLE );
			break;

		default:
			if( !semaphore_signal( CNTRL_REQUESTED )) {
				move.cntrl = move.cntrl_request;
			}
//			semaphore_release( CNTRL_REQUESTED );
			semaphore_signal_all( 0x00 );
	}

#ifdef USART_DEBUG // {{{
	USART_send_str_P( (uint8_t*) PSTR("Bank="));
	itoa( (int) move.movement, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(", CNTRL="));
	itoa( (int) move.cntrl, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(", R="));
	itoa( (int) r, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(" Semaphore= "));
	itoa( (int) semaphore, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR("\r\n"));
#endif //}}}

} /// }}}

void init( void )
{ //{{{
	semaphore = 0;

	servo_cntrls.cntr = 0;
	servo_cntrls.bank = 1;
	servo_cntrls.dirty = 0;
	servo_cntrls.servo_index = 0;
	step_last_known = 100; // INFINITY

	move.movement = STRAIGHT;
	move.movement_request = STRAIGHT;
	move.cntrl = FORWARD;
	move.stepper_current = 0;
	move.stepper_previous = 100;

	speed.max = LINREG_NO_OF_STEPS;
	speed.index = speed.max;

	srf02_cntrls.app_vector = MSG_PING;
	srf02_cntrls.ranging_delay = 0;
	srf02_cntrls.reading_delay_cntr = 0;
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_0;
	srf02_cntrls.der21_limit = DERIVATION_2_LIMIT;
	srf02_cntrls.dirty = 0;
	srf02_cntrls.semaphore = 0;
	srf02_cntrls.status = DISABLE;
	srf02_cntrls.error_code = 0;
} //}}}

move_vector_t determine_next_move( uint8_t vec )
{ //{{{
//              direction is REVERSED (D) --------+  |
//               movement is STRAIGHT (m) -------+|  |
//        second derivation negative (d2) ------+||  |                                          
//         first derivation negative (d1) -----+|||  |                                         
//                  Range under limit (A) ----+||||  |
//                              Error (E) ---+|||||  | (BIN MASK) = (HEX MASK)
//                                           ||||||  | 
   if( 0x20 == ( vec & 0x20 )) //         (xx1xxxxx) | (00100000) = 0x20
      return (move_vector_t) MSG_M_STOP;//   ||||||  |
   if( 0x00 == ( vec & 0x30 )) //         (xx00xxxx) | (00110000) = 0x30
      return (move_vector_t) MSG_M_FORW;//   ||||||  |
   if( 0x1C == ( vec & 0x3C )) //         (xx0111xx) | (00111100) = 0x3C
      return (move_vector_t) MSG_M_REV;//    ||||||  |
   if( 0x1A == ( vec & 0x3E )) //         (xx011010) | (00111110) = 0x3E
      return (move_vector_t) MSG_M_TURN_0;// ||||||  |
   if( 0x13 == ( vec & 0x3B )) //         (xx010011) | (00111011) = 0x3B
      return (move_vector_t) MSG_M_TURN_1;// ||||||  |
   if( 0x19 == ( vec & 0x3F )) //         (xx011001) | (00111111) = 0x3F
      return (move_vector_t) MSG_M_LEFT;//   ||||||  |
   if( 0x18 == ( vec & 0x3F )) //         (xx011000) | (00111111) = 0x3F
      return (move_vector_t) MSG_M_RIGHT;
   return (move_vector_t) MSG_M_NOP;
	// See function prototype for detailed description of selected algorithm.
} //}}}

void process_ranging_data( void )
{ //{{{
	srf02_data.der11 = srf02_data.range1 - srf02_data.range0; // first derivation of distance
	srf02_data.der21 = srf02_data.der11 - srf02_data.der10;   // second derivation of the distance

	srf02_data.range0 = srf02_data.range1;
	srf02_data.der10 = srf02_data.der11;
	srf02_data.der20 = srf02_data.der21;
} //}}}

uint8_t get_app_vector( srf02_t d, move_t m, srf02_cntrl_t c )
{ //{{{
	uint8_t r;

	r = ((c.error_code >= 1 ? 1 : 0)<<5);
	r |= ((d.range1 < SAFE_RANGE_LIMIT ? 1 : 0 )<<4);
	r |= ((d.der11 < DERIVATION_1_LIMIT ? 1 : 0)<<3) | ((d.der21 < c.der21_limit ? 1 : 0)<<2);
	r |= ((m.movement == STRAIGHT ? 1 : 0)<<1) | (m.cntrl == REVERSE ? 1 : 0);

#ifdef USART_DEBUG //{{{
	USART_send_str_P( (uint8_t*) PSTR("Vector="));
	itoa( (int) r, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(" - "));
#endif //}}}

	return r;
} //}}}

uint8_t on_msg_nop( void ) // * SRF02 handlers *************************
{ //{{{
/*
#ifdef _SRF02_DEBUG
	uint8_t debug_data = 255;

	srf02_data.range1 = (uint16_t) srf02_get_version_ex( &debug_data );
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("%d (%d)\r\n"), debug_data, srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Version="));
	itoa( (int) debug_data, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(", E="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR("\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif
*/
	return 0;
} //}}}

uint8_t on_msg_ping( void )
{ //{{{
	srf02_ping();
	srf02_cntrls.status = ENABLE;
	return 0;
} //}}}

uint8_t on_msg_check( void )
{ // {{{ 
	uint8_t ret = 0;

	if( srf02_cntrls.ranging_delay++ > SRF02_RANGING_DELAY ) {
		srf02_cntrls.ranging_delay = 0;
		ret = 1;
	}
	srf02_cntrls.status = ENABLE;
	return ret;

//	ret = (uint8_t) srf02_is_data_ready();
//	srf02_cntrls.status = ENABLE;
//	return ret;
} //}}}

uint8_t on_msg_get_range_hi( void )
{ //{{{
	uint8_t byte, ret;

	ret = (uint8_t) srf02_get_range_byte( HI_BYTE, &byte );

	if( ret & 0x07 ) {  // check for error. If error, then repeat the sonar signal
		srf02_cntrls.error_code = ( ret & 0x07 );
		srf02_cntrls.dirty = SIGNALED;
		srf02_cntrls.app_vector = MSG_PING;
		return 0;
	}
	if(( ret & 0xF8 ) == SRF02_WRONG_REGISTER )
		while(1); // Fire the exception, watchdog resets the computer.

	srf02_data.range1 = (uint16_t) (byte<<8);
	srf02_cntrls.status = ENABLE;
	return 0;
} //}}}

uint8_t on_msg_get_range_lo( void )
{ //{{{
	uint8_t byte, ret;

	ret = (uint8_t) srf02_get_range_byte( LO_BYTE, &byte );

	if( ret & 0x07 ) {
		srf02_cntrls.error_code = ( ret & 0x07 );
		srf02_cntrls.dirty = SIGNALED;
		return 0;
	}
	if(( ret & 0xF8 ) == SRF02_WRONG_REGISTER )
		while(1); // Fire the unhadled exception, watchdog resets the computer.

	srf02_data.range1 |= (uint8_t) byte;
	srf02_cntrls.dirty = SIGNALED;
	return 0;
} //}}}

uint8_t on_msg_m_nop( void ) // * Movement Handlers *******************
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". NO CHANGE\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	return 0;
} //}}}

uint8_t on_msg_m_stop( void ) 
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	 sprintf_P( (char*) buffer, PSTR("Ranging Error=%d\r\n"), srf02_cntrls.error_code );
	 USART_send_str( (uint8_t*) buffer );
#else
	 USART_send_str_P( (uint8_t*) PSTR("Ranging Error="));
	 itoa( (int) srf02_cntrls.error_code, (char*) buffer, 10 );
	 USART_send_str( (uint8_t*) buffer );
	 USART_send_str_P( (uint8_t*) PSTR(" STOP.\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
				//}}}
	move.cntrl = STOP;
	return 0;
} //}}}

uint8_t on_msg_m_forw( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". REQUEST_STRAIGHT\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( MOVEMENT_REQUESTED )) {
		move.movement_request = REQUEST_STRAIGHT;
		srf02_cntrl( DISABLE );
	}
	if( semaphore_signal( CNTRL_REQUESTED )) {
		move.cntrl_request = FORWARD;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_0;
	srf02_cntrls.der21_limit = DERIVATION_2_LIMIT;
	return 0;
} //}}}

uint8_t on_msg_m_rev( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". REQUEST_STRAIGHT\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( MOVEMENT_REQUESTED ) ){
		move.movement_request = REQUEST_STRAIGHT;
		srf02_cntrl( DISABLE );
	}
	if( semaphore_signal( CNTRL_REQUESTED ) ){
		move.cntrl_request = REVERSE;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_1;
	srf02_cntrls.der21_limit = DERIVATION_2_LIMIT_BLOCKING;
	return 0;
} //}}}

uint8_t on_msg_m_turn0( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". REQUEST_CURVED\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( MOVEMENT_REQUESTED ) ){
		move.movement_request = REQUEST_CURVED;
		srf02_cntrl( DISABLE );
	}
	if( semaphore_signal( CNTRL_REQUESTED ) ){
		move.cntrl_request = REVERSE;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_1;
	return 0;
} //}}}

uint8_t on_msg_m_turn1( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". REQUEST_CURVED\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( MOVEMENT_REQUESTED ) ){
		move.movement_request = REQUEST_CURVED;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_1;
	return 0;
} //}}}

uint8_t on_msg_m_left( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". FORWARD\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( CNTRL_REQUESTED ) ){
		move.cntrl_request = FORWARD;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_2;
	return 0;
} //}}}

uint8_t on_msg_m_right( void )
{ //{{{
#ifdef USART_DEBUG //{{{
#ifndef USART_DEBUG_SMALL_SIZE
	sprintf_P( (char*) buffer, PSTR("Range [cm]=%d\r\n"), srf02_data.range1 );
	USART_send_str( (uint8_t*) buffer );
#else
	USART_send_str_P( (uint8_t*) PSTR("Range [cm]="));
	itoa( (int) srf02_data.range1, (char*) buffer, 10 );
	USART_send_str( (uint8_t*) buffer );
	USART_send_str_P( (uint8_t*) PSTR(". REVERSE\r\n"));
#endif // USART_DEBUG_SMALL_SIZE
#endif // USART_DEBUG
//}}}
	if( semaphore_signal( CNTRL_REQUESTED ) ){
		move.cntrl_request = REVERSE;
		srf02_cntrl( DISABLE );
	}
	srf02_cntrls.reading_delay_lim = SRF02_READING_DELAY_3;
	return 0;
} //}}}

