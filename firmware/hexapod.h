#ifndef __MAIN_H__
#define __MAIN_H__
/*
 *****************************************************************************
 *
 * File Name    : hexapod.h
 * Title        : Firmware for HEXAPOD robot
 * Author       : Martin Metal
 * Date         : 18/03/2012
 * Revised      : 18/03/2012
 * Version      : 1.0
 * Target MCU   : Atmel AVR ATMega8
 * Editor Tabs  : 3
 * Description  : The central header file for the HEXAPOD project.
 * Remark		 : Folding in VIM requires <<<,>>> instead of typical {{{,}}}
 *
 
 * Hexapod is a robot having 6 independently moving arms. Each arm constitutes
 * two servos.  One servo is a shoulder and moves the arm around an axe
 * perpendicular to the plane of movement. The other servo - elbow servo -
 * controls the movement of the arm in the plane parallel to the plane of the
 * movement of the whole robot. 
 *
 * The arm constitutes three fixed parts, COXA, FEMUR and TIBIA. While COXA is
 * connected to body and to FEMUR, FEMUR then connects to TIBIA. This
 * construction uses servo between body and COXA, no servo between COXA and
 * FEMUR and one servo between FEMUR and TIBIA.  Additionally, the length of
 * COXA is given as the length of the servo housing plus small casing around
 * the servo housing. The angle between COXA and FEMUR cannot be changed and is
 * fixed at 45 degrees. This limitation reduces the spectrum of available
 * movements, but still should be sufficient to achieve reasonably smooth
 * movement of the robot in straight forward run and curved run. The detailed
 * dimensions of an arm are given later in this file.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/io.h>    
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <math.h>

#include <i2c.h>
#include <srf02.h>
//#include <i2c_eeprom.h>


#ifdef USART_DEBUG
//<<<
	#include "USART/usart.h"
#endif
//>>>

#define SIGNALED	  						0x01

#define TRUE								0x01
#define FALSE								0x00

#define SRF02_RANGING_DELAY			0x37 // 77 ms is device blocked when ranging is active.

//#define SRF02_READING_DELAY			0xB4 // reading is triggered every 4 seconds.
#define SRF02_READING_DELAY_3S  		0x89 // reading is triggered every 3 seconds.
#define SRF02_READING_DELAY_9S 		0x1AE // reading is triggered every 10 seconds.
#define SRF02_READING_DELAY_12S 		0x225 // reading is triggered every 12 seconds.
#define SRF02_READING_DELAY_18S 		0x34A // reading is triggered every 20 seconds.
#define SRF02_READING_DELAY_26S 		0x4a6 // reading is triggered every 23 seconds.
#define SRF02_READING_DELAY_28S 		0x501 // reading is triggered every 30 seconds.
#define SRF02_READING_DELAY_39S 		0x6f9 // reading is triggered every 34 seconds.

#define SRF02_READING_DELAY_0			0x53  // 1.8s
#define SRF02_READING_DELAY_1			0x181 // 8.5s
#define SRF02_READING_DELAY_2			0x2A5 // 14.8s
#define SRF02_READING_DELAY_3			0x415 // 22.8s
/* Reading delay counter is calculated:
 *
 *    DELAY = SEC * CLK / 262144
 *
 * where SEC is number of seconds they should take,
 */

#define BIT_MOV_REQUEST_CURVED		0x1
#define BIT_MOV_CURVED					0x2
#define BIT_MOV_REQUEST_STRAIGHT		0x3
#define BIT_CNTRL_REQUEST_CURVED		0x4
#define BIT_CNTRL_CURVED				0x5
#define BIT_CNTRL_REQUEST_STRAIGHT	0x6
#define BIT_CNTRL_STRAIGHT				0x7
/* Semaphore bits
 */


#ifdef _DINOSAURIA
#define STEP_TAB_SIZE					0x12 // 18 tabelated steps
#define LINREG_NO_OF_STEPS				0x07 // 7 steps between tabulated entries is calculated (linear regression)
#else
#define STEP_TAB_SIZE					0x10 // 16 tabelated steps
#define LINREG_NO_OF_STEPS				0x0A // 10 steps between tabulated entries is calculated (linear regression)
#endif

#define SAFE_RANGE_LIMIT				0x37 // 55 cm is the critical limit and the robot starts reacting
#define DERIVATION_1_LIMIT				-2   // -2 is the minimum 1st derivation value before robot turns
#define DERIVATION_2_LIMIT_BLOCKING -600 // Block the 2nd derivation result
#define DERIVATION_2_LIMIT				-10  // Evaluate the second derivation .

#define SERVOSHIELD_DDR					DDRD
#define SERVOSHIELD_CTRL				PORTD
#define MR_BANK_0							PD5
#define MR_BANK_1							PD6
#define CP0									PD7
/* HW Definition for the servo shield. Servo shield constitutes two Johnson
 * decade counter (two banks) with 10 decoded outputs (4017). The application 
 * utilizes 3 control pins:
 *
 *  +---------------------------+-----------+-----------+
 *  |     Name                  |  4017 pin | PIN ATmega|
 *  +---------------------------+-----------+-----------+
 *  | Master Reset (bank 0)     |    MR     |    PD5    |
 *  | Master Reset (bank 1)     |    MR     |    PD6    |
 *  | clock input H->L (common) |   CP1N    |    GND    |
 *  | clock input L->H (common) |   CP0     |    PD7    |
 *  +---------------------------+-----------+-----------+
 *
 * The application does not utilize the CP1N pin. It is permanently connected
 * to low level signal.
 *
 * The counter generates low state on output Q1-Q9 and high status on Q0 after
 * reset. Q0 output is therefore not used in the application. The counter is
 * clocked on the CP0 input.  First rising edge on CP0 brings the output Q1 to
 * high state.  Next clock signal on CP0 sets Q1 to low again and drives Q2
 * high. Signal propagates with every rising edge on CP0 until MR signal is
 * sent. Theoretically one Johnson decade counter can control 9 servos, the
 * implementation reduces this count to 6 servos only. The reason being is the
 * time interval between two control PWM pulses to each servo (20ms).
 *
 * Servo requires a control signal ranging from 544us to 2400us, where 544 us
 * set the servo to the angle position 0 and 2400us to the angle position 180
 * degrees. The 1472us sets the centered position, 90 degrees.  Thus the
 * equation for ideal servo:
 *
 * 		y = 590.87 * x + 544 [us]
 *
 * Typical servos are designed for control pulses ranging from 0.8 through 2.2
 * ms. It means that 0 and 180 degree limits are not achievable. The hexapod
 * design is aware of this limitation and using a corrected equation to
 * determine the angles based on the known control pulses. The experiment 
 * for the used servos yielded the equation in this form:
 *
 * 		Y = 546 * X + 622  [us]
 *
 */

#define 	FEMUR													105
#define 	TIBIA													152
#define	HEIGHT												75
#define	COUNT_OF_ARMS										6
#define	COUNT_OF_SERVOS									12
#define	BETA_0												1.57079633
/* Define the physical parameters of the robot. The dimensions do not contain length
 * of COXA. We consider parameter HEIGHT instead defined as the intersection of the
 * longitudinal axis of FEMUR and rotational axis of the shoulder joint
 * (intersection of COXA with the body). This simplification is possible due to the
 * fact that the angle between COXA and FEMUR is fixed. 
 *
 *
 *               AXIS|
 *                   |         ELBOW
 *                   |         (Beta)
 *                   |           /\
 *                   |    FEMUR /  \
 *          SHOULDER |         /    \
 *                ____________/      \
 *           (Gamma) | COXA           \
 *                   |      /          \
 *                   |                  \
 *                   |    /              \
 *                   |                    \ TIBIA (152 mm)
 *                   |  /                  \
 *                   |                      \
 *            _______|/                      \
 *            A      |                        \
 *            |      |                         \
 *         HEIGHT    |                          \
 *            |      |                           \
 *            V______|                            \
 *
 *
 * The angle Beta is the angle between FEMUR and TIBIA and controls the 'reach' of
 * the arm. It indirectly control also the height of the body above the surface,
 * because of the fixed angle between COXA and FEMUR. The angle gamma is the
 * rotational angle of the shoulder joint. The coordinated change of both Gamma and
 * Beta can achieve to move the tip of the TIBIA along the linear line. The physical
 * dimensions allows to achieve the length of the step up to 90 mm. The mechanical
 * properties of used servos and construction material however limits the length
 * of step to 70mm.
 *
 * Considering the plane of the movement as two dimensional (x,y) one can use the
 * geometric formulas to calculate the angles gamma and beta to bring an arm in
 * given x,y position. We always consider z=0, and the robot achieves best, symmetric
 * results on smooth surfaces only.
 */


enum resource_id_t {
	MOVEMENT = 0,
	MOVEMENT_REQUESTED,
	CNTRL,
	CNTRL_REQUESTED
};
typedef enum resource_id_t resource_id_t;


enum signal_t {
	SIGNAL = 0,
	CLEAR
};
typedef enum signal_t signal_t;


enum type_t {
	DISABLE = 0,
	ENABLE
};
typedef enum type_t type_t;
/* Generic type for ON and OFF status.
 */

enum direction_t {
	STRAIGHT = 0,
	CURVED,
	REQUEST_CURVED,
	REQUEST_STRAIGHT
};
typedef enum direction_t direction_t;
/* Declaring the movements types for the robot. Straight forward
 * movement can be backwards and forwards, curved direction includes
 * turn left and turn right. That is all for this time being.
 */

enum dir_cntrl_t {
	REVERSE = -1,
	STOP = 0,
	FORWARD = 1
};
typedef enum dir_cntrl_t dir_cntrl_t;
/* Declaring the decrement or increment of the step counter.  Robot crawls
 * forwards, when step counter increments, Robot crawls backwards, when step
 * counter decrements.  Robot turns red for incrementing counter, and turns
 * left for decrementing counter.
 */

struct move_t {
	uint8_t			stepper_previous;
	uint8_t			stepper_current;
	direction_t 	movement_request;
	direction_t 	movement;
	dir_cntrl_t	  	cntrl_request;
	dir_cntrl_t	  	cntrl;
};
typedef struct move_t move_t;
/* Tracking container for the movement.
 */

struct ocr1a_t {
	uint8_t cntr;
	uint8_t bank;
	uint8_t dirty;
	uint8_t servo_index;
};
typedef struct ocr1a_t ocr1a_t;
/* This structure keeps track on the bank switching.  The servo shield uses two
 * decade counters (4017).  The counter keeps track what servo is currently
 * controlled, and the bank takes care about associated decade counter.  This
 * shield can control up to 16 servos, but one decade counter can drive 9
 * servos at maximum. Therefore two circuits and bank switching among them.
 */ 

struct servo_cntrl_t {
	uint16_t past[12];
	uint16_t current[12];
	uint16_t future[12];
};
typedef struct servo_cntrl_t servo_cntrl_t;
/* This is the latch buffer for servo. It offers three registers for servo
 * data. The steps are tabulated and the distances between tabulated values
 * will be connected by means of linear approximation. The step data then
 * follow linear regression between two tabulated points. The number of steps
 * along the linear regression determines the speed of the movement of the arm.
 */

struct speed_t {
	uint8_t max;
	uint8_t index;
};
typedef struct speed_t speed_t;
/* Defines number of steps along the linear regression calculated for two
 * points in the servo_cntrl_t structure. The total number of steps defines the
 * overall velocity of moving the arms.  The velocity defines how realistic the
 * movements are.
 */


enum app_vector_t {
	MSG_NOP = 0,
	MSG_PING,
	MSG_CHECK,
	MSG_GET_RANGE_HI,
	MSG_GET_RANGE_LO
};
typedef enum app_vector_t app_vector_t;
/* Application uses handlers to service SRF02 range finder device.
 * This enumarator defines the codes for service routines.
 */


enum move_vector_t {
   MSG_M_STOP = 0,
   MSG_M_FORW,
   MSG_M_REV,
   MSG_M_TURN_0,
   MSG_M_TURN_1,
   MSG_M_LEFT,
   MSG_M_RIGHT,
	MSG_M_NOP
};
typedef enum move_vector_t move_vector_t;


struct _vtabentry_t {
   uint8_t 	(*call)( void );  // entry point for serviceable routine (handler)
   uint8_t	ret;      		   // return code 
};
typedef struct _vtabentry_t _vtabentry_t;
/* This is part of the data container for SRF02 device. Handling is specific,
 * as the I2C communication must be synchronized with the interrupts generated
 * by the servo shield (compare match). The communication with SRF02 is than
 * split into more functions, and the execution is split 
 */


struct srf02_cntrl_t {
	app_vector_t	app_vector;       	// stage in the ranging process (1-ping, 2-echo)
	uint8_t 			error_code;       	// error code delivered by ranging process.
	uint8_t 			ranging_delay;    	// defines the cycle that ranging device requires between 1 and 2.
	uint16_t			reading_delay_cntr;  // defines the cycle between two subsequent ranging cycles.
	uint16_t			reading_delay_lim;   // allows to store varying couting limit.
	int16_t			der21_limit;  			// allows to store varying limit of 2nd derivation
	type_t			status;           	// ranging status flag (see description)
	uint8_t			dirty;            	// ranging completed flag (see description)
	uint8_t			semaphore;        	// access control flag. Synchronization element.
};
typedef struct srf02_cntrl_t srf02_cntrl_t;
/* This structure controls the ranging process. It contains 3 flags:
 *
 *    status     Status flag is set by timer every 3 (or 4) seconds. 
 *               (derivation time base). The interrupt handler Timer0 sets this
 *               flag.  Once set, the ranging process can be triggered. Ranging
 *               takes two stages, first fire the burst, second read the echo.
 *               The time delay between both stages takes up to 77 ms minimum.  
 *               Status flag is also used to control synchronized access to I2C
 *               bus and avoiding the collision with the other interrupt driven
 *               events (USART, servo shield, timer). 
 *
 *    semaphore  is an access flag. Semaphore accompanies the status flag. Once
 *               the status flag is set, the next interrupt called on compare
 *               match (servo shield) releases the semaphore. The main cycle
 *               then enters the routine for performing the ranging routine. It
 *               also sets the semaphore so only first stage of ranging can
 *               happen. The application then further waits for next semaphore
 *               to continue the ranging process. It happens first after the
 *               ranging interval expires and is again synchronized with the
 *               servo shield (status flag). 
 *
 *    dirty      update flag. Once ranging is accomplished (both successfully
 *               or erroneous) this flag gets set and main cycle enters the
 *               procedure for using the ranging data for movement translation.
 *               For this the routine needs the ranging data. 
 *
 * The SRF02 hardware required ~70ms to accomplish one ranging cycle. This
 * structure holds the counter, that is update every time the servo shield
 * enters the interrupt handler. The interval is preset to number that
 * corresponds with 80-100 ms between the ranging cycles. The recommended
 * method (reading the register No.0) does not work. The device continues to
 * deliver version number even within the ping - echo mode.  
 */


struct srf02_t {
	uint16_t 		range0;
	uint16_t 		range1;
	int16_t  		der10;
	int16_t  		der11;
	int16_t 	   	der20;
	int16_t 		   der21;
	_vtabentry_t 	onmessage;
};
typedef struct srf02_t srf02_t;
/* Container for ranging data. The structure contains pointer the handler method
 * as well as the ranging data itself. The ranging data are used by routines designed
 * to determine the movement of the robot.
 */


extern uint8_t on_msg_nop( void );
extern uint8_t on_msg_ping( void );
extern uint8_t on_msg_check( void );
extern uint8_t on_msg_get_range_hi( void );
extern uint8_t on_msg_get_range_lo( void );

static _vtabentry_t VTable[] PROGMEM = {
/*	0	(0)*/	{	on_msg_nop,					0	},
/*	1	(2)*/	{	on_msg_ping,				2	}, 
/*	2	(3)*/	{	on_msg_check,				2	},
/*	3	(4)*/	{	on_msg_get_range_hi,		4	},
/*	4	(0)*/	{	on_msg_get_range_lo,		1	}
};
/* Service handlers for SRF02 device. The handlers are called from within the
 * main loop, after the semaphore for a single call gets released. One call is
 * a simple I2C exchange, one byte retrieval/write. This operation takes well
 * bellow 0.7us, what is minimal time gap between the two interrupts caused by
 * compare match. The access to the handler is thus synchronized and it cannot
 * happen that I2C data exchange happens when compare match handler fires. This
 * helps to control exact pulses for servos and avoid any collision on I2C bus. 
 */


extern uint8_t on_msg_m_stop( void );
extern uint8_t on_msg_m_forw( void );
extern uint8_t on_msg_m_rev( void );
extern uint8_t on_msg_m_turn0( void );
extern uint8_t on_msg_m_turn1( void );
extern uint8_t on_msg_m_left( void );
extern uint8_t on_msg_m_right( void );
extern uint8_t on_msg_m_nop( void );

static _vtabentry_t MTable[] PROGMEM = {
/* 0 */		{	on_msg_m_stop,				0	},
/* 1 */		{	on_msg_m_forw,				0	},
/* 2 */		{	on_msg_m_rev,				0	},
/* 3 */		{	on_msg_m_turn0,  			0	},
/* 4 */		{	on_msg_m_turn1,  			0	},
/* 5 */		{	on_msg_m_left,				0	},
/* 6 */		{	on_msg_m_right,  			0	},
/* 7 */		{	on_msg_m_nop,  			0	}
};
/* Prototypes of handlers used to service the movement. Function
 * 'determine_next_move()' delivers the index of the handler from this vtable.
 * The application logic then executes the handler. Handler code configures the
 * movement structures and create log entries, if configured for it.
 */

struct arm_t {
	uint16_t		servo_shoulder;
	uint16_t		servo_elbow;
};
typedef struct arm_t arm_t;

struct step_t {
	arm_t	arm[6];
};
typedef struct step_t step_t;

struct run_t {
	step_t step[STEP_TAB_SIZE];
};
typedef struct run_t run_t;
/* System of structures used to describe the movement of the hexapod.  The
 * movement data are calculated outside this project and are just inserted into
 * this container. The design uses 16 tabulated points for each arm. Those 16
 * points are are synchronized between all the arms and servos. The top most
 * structure refines the "run", consists of 16 steps. Each step requires 6
 * arms, two servos each.
 */ 

#ifdef _PROGMEM_DATA
//<<<
#ifndef _DINOSAURIA // <<<
const run_t run[] __attribute__ ((section(".progmem"))) = {{{
/*
 *----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------           
 *          |   Servo 0  |  Servo 1  |  Servo 2  |  Servo 3  |  Servo 4  |  Servo 5  |
 *  Draha   |   Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |
 *----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------*/           
//<<< This is an older version.
/*   0 * 	{{{1361,2796},{2506,2074},{2218,3091},{1361,2266},{2530,1820},{2219,2054}}},
 *  10 * 	{{{1390,2834},{2455,2092},{2327,3089},{1391,2228},{2479,1802},{2329,2056}}},
 *  27 * 	{{{1472,2913},{2329,2122},{2559,3069},{1473,2149},{2353,1772},{2560,2076}}},
 *  52 * 	{{{1633,3003},{2140,2138},{2802,3009},{1634,2059},{2164,1756},{2803,2136}}},
 *  77 * 	{{{1876,3063},{1951,2122},{2963,2919},{1877,1999},{1975,1772},{2964,2226}}},
 *  94 * 	{{{2107,3083},{1825,2092},{3046,2840},{2109,1979},{1849,1802},{3046,2305}}},
 * 104 * 	{{{2217,3086},{1774,2074},{3075,2802},{2218,1977},{1798,1820},{3075,2343}}},
                                                                                        
 * 104 * 	{{{2217,2586},{1774,2574},{3075,2302},{2218,1477},{1798,2320},{3075,1843}}},
 *  94 * 	{{{2107,2583},{1825,2592},{3046,2340},{2109,1479},{1849,2302},{3046,1805}}},
 *  77 * 	{{{1876,2563},{1951,2622},{2963,2419},{1877,1499},{1975,2272},{2964,1726}}},
 *  52 * 	{{{1633,2503},{2140,2638},{2802,2509},{1634,1559},{2164,2256},{2803,1636}}},
 *  27 * 	{{{1472,2413},{2329,2622},{2559,2569},{1473,1649},{2353,2272},{2560,1576}}},
 *  10 * 	{{{1390,2334},{2455,2592},{2327,2589},{1391,1728},{2479,2302},{2329,1556}}},
 *   0 * 	{{{1361,2296},{2506,2574},{2218,2591},{1361,1766},{2530,2320},{2219,1554}}} 
 */
//>>>
/*	0	*/	{{{1889,2783},{2367,2579},{1877,2789},{1955,1749},{2493,1821},{1913,1815}}},
/*	0	*/	{{{1889,2783},{2367,2079},{1877,2789},{1955,2249},{2493,1821},{1913,2315}}},
/*	6	*/	{{{1935,2795},{2321,2091},{1923,2801},{2001,2237},{2448,1809},{1959,2303}}},
/*	18	*/	{{{2029,2813},{2235,2107},{2017,2819},{2087,2221},{2353,1792},{2045,2287}}},
/*	35	*/	{{{2170,2822},{2086,2118},{2158,2828},{2236,2210},{2212,1782},{2194,2276}}},
/*	53	*/	{{{2319,2811},{1945,2109},{2307,2817},{2377,2220},{2063,1793},{2335,2286}}},
/*	64	*/	{{{2405,2795},{1851,2091},{2393,2801},{2472,2237},{1977,1809},{2430,2303}}},
/*	70	*/	{{{2451,2783},{1805,2079},{2439,2789},{2517,2249},{1931,1821},{2475,2315}}},
																										
/*	70	*/	{{{2451,2783},{1805,2579},{2439,2789},{2517,1749},{1931,1821},{2475,1815}}},
/*	70	*/	{{{2451,2283},{1805,2579},{2439,2289},{2517,1749},{1931,2321},{2475,1815}}},
/*	64	*/	{{{2405,2295},{1851,2591},{2393,2301},{2472,1737},{1977,2309},{2430,1803}}},
/*	53	*/	{{{2319,2311},{1945,2609},{2307,2317},{2377,1720},{2063,2293},{2335,1786}}},
/*	35	*/	{{{2170,2322},{2086,2618},{2158,2328},{2236,1710},{2212,2282},{2194,1776}}},
/*	18	*/	{{{2029,2313},{2235,2607},{2017,2319},{2087,1721},{2353,2292},{2045,1787}}},
/*	6	*/	{{{1935,2295},{2321,2591},{1923,2301},{2001,1737},{2448,2309},{1959,1803}}},
/*	0	*/	{{{1889,2283},{2367,2579},{1877,2289},{1955,1749},{2493,2321},{1913,1815}}}
/* This is the structure for straight forward movement. The recorded OCR1A entries helps
 * instruct the servos to move forwards or backwards. Direction is controlled by 
 * incrementing or decrementing the step counter.
 */
}},{{
//<<< This is an older version ...
/*   0 * 	{{{1361,2796},{1748,2074},{2126,3090},{1361,2267},{1771,1820},{2125,2055}}},
 *  10 * 	{{{1399,2844},{1813,2098},{2268,3091},{1399,2218},{1836,1796},{2266,2054}}},
 *  52 * 	{{{1484,2922},{1940,2130},{2507,3076},{1484,2140},{1964,1764},{2506,2069}}},
 *  94 * 	{{{1661,3013},{2140,2147},{2775,3018},{1661,2050},{2164,1747},{2775,2126}}},
 * 104 * 	{{{1930,3070},{2340,2130},{2952,2928},{1929,1993},{2364,1764},{2952,2217}}},
 * 104 * 	{{{2170,3085},{2468,2098},{3037,2850},{2169,1977},{2491,1796},{3037,2295}}},
 * 104 * 	{{{2311,3084},{2533,2074},{3075,2801},{2310,1978},{2556,1820},{3075,2343}}},
               
 * 104 * 	{{{2311,2584},{2533,2574},{3075,2301},{2310,1478},{2556,2320},{3075,1843}}},
 * 104 * 	{{{2170,2585},{2468,2598},{3037,2350},{2169,1477},{2491,2296},{3037,1795}}},
 * 104 * 	{{{1930,2570},{2340,2630},{2952,2428},{1929,1493},{2364,2264},{2952,1717}}},
 *  94 * 	{{{1661,2513},{2140,2647},{2775,2518},{1661,1550},{2164,2247},{2775,1626}}},
 *  52 * 	{{{1484,2422},{1940,2630},{2507,2576},{1484,1640},{1964,2264},{2506,1569}}},
 *  10 * 	{{{1399,2344},{1813,2598},{2268,2591},{1399,1718},{1836,2296},{2266,1554}}},
 *   0 * 	{{{1361,2296},{1748,2574},{2126,2590},{1361,1767},{1771,2320},{2125,1555}}}
 */
//>>>
/*	0	*/	{{{1889,2783},{2367,2579},{1877,2789},{2517,1749},{1931,1821},{2475,1815}}},
/*	0	*/	{{{1889,2783},{2367,2079},{1877,2789},{2517,2249},{1931,1821},{2475,2315}}},
/*	6	*/	{{{1935,2795},{2321,2091},{1923,2801},{2472,2237},{1977,1809},{2430,2303}}},
/*	18	*/	{{{2029,2813},{2235,2107},{2017,2819},{2377,2220},{2063,1793},{2335,2286}}},
/*	35	*/	{{{2170,2822},{2086,2118},{2158,2828},{2236,2210},{2212,1782},{2194,2276}}},
/*	53	*/	{{{2319,2811},{1945,2109},{2307,2817},{2087,2221},{2353,1792},{2045,2287}}},
/*	64	*/	{{{2405,2795},{1851,2091},{2393,2801},{2001,2237},{2448,1809},{1959,2303}}},
/*	70	*/	{{{2451,2783},{1805,2079},{2439,2789},{1955,2249},{2493,1821},{1913,2315}}},
																										
/*	70	*/	{{{2451,2783},{1805,2579},{2439,2789},{1955,1749},{2493,1821},{1913,1815}}},
/*	70	*/	{{{2451,2283},{1805,2579},{2439,2289},{1955,1749},{2493,2321},{1913,1815}}},
/*	64	*/	{{{2405,2295},{1851,2591},{2393,2301},{2001,1737},{2448,2309},{1959,1803}}},
/*	53	*/	{{{2319,2311},{1945,2609},{2307,2317},{2087,1721},{2353,2292},{2045,1787}}},
/*	35	*/	{{{2170,2322},{2086,2618},{2158,2328},{2236,1710},{2212,2282},{2194,1776}}},
/*	18	*/	{{{2029,2313},{2235,2607},{2017,2319},{2377,1720},{2063,2293},{2335,1786}}},
/*	6	*/	{{{1935,2295},{2321,2591},{1923,2301},{2472,1737},{1977,2309},{2430,1803}}},
/*	0	*/	{{{1889,2283},{2367,2579},{1877,2289},{2517,1749},{1931,2321},{2475,1815}}}
/* This structure makes hexapod to turn left or right. The direction of the curve
 * is controlled by incrementing or decrementing the step counter.
 */
}}};
#else // >>> _DINOSAURIA
const run_t run[] __attribute__ ((section(".progmem"))) = {{{
/*----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------           
 *          |   Servo 0  |  Servo 1  |  Servo 2  |  Servo 3  |  Servo 4  |  Servo 5  |
 *  Draha   |   Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |
 *----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------*/           
// ARM MOVEMENT 2-5-1-3-0-4 - STRAIGHT
/*   0 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*   0 */	{{{2038,2814},{2346,2613},{2583,2289},{2073,1710},{2257,1811},{1982,1798}}},
/*  14 */	{{{2070,2817},{2378,2610},{2021,2289},{2039,1710},{2226,1804},{1951,1805}}},
/*  14 */	{{{2112,2820},{2419,2604},{2021,2789},{1998,1712},{2187,1796},{1913,1815}}},
/*  14 */	{{{2153,2822},{2458,2596},{2059,2799},{1956,1715},{2147,1790},{1913,2315}}},
/*  28 */	{{{2187,2822},{2489,2589},{2090,2806},{1924,1718},{2114,1787},{2475,2315}}},
/*  28 */	{{{2228,2820},{2527,2579},{2129,2814},{1883,1724},{2072,1784},{2475,1815}}},
/*  28 */	{{{2270,2817},{2527,2079},{2170,2820},{1844,1732},{2031,1782},{2437,1805}}},
/*  42 */	{{{2302,2814},{1965,2079},{2202,2823},{1813,1739},{1997,1782},{2406,1798}}},
/*  42 */	{{{2343,2808},{1965,2579},{2244,2826},{1775,1749},{1956,1784},{2367,1790}}},
/*  42 */	{{{2382,2800},{2003,2589},{2285,2828},{1775,2249},{1914,1787},{2327,1784}}},
/*  56 */	{{{2413,2793},{2034,2596},{2319,2828},{2337,2249},{1882,1790},{2294,1781}}},
/*  56 */	{{{2451,2783},{2073,2604},{2360,2826},{2337,1749},{1841,1796},{2252,1778}}},
/*  56 */	{{{2451,2283},{2114,2610},{2402,2823},{2299,1739},{1802,1804},{2211,1776}}},
/*  70 */	{{{1889,2283},{2146,2613},{2434,2820},{2268,1732},{1771,1811},{2177,1776}}},
/*  70 */	{{{1889,2783},{2188,2616},{2475,2814},{2229,1724},{1733,1821},{2136,1778}}},
/*  70 */	{{{1927,2793},{2229,2618},{2514,2806},{2189,1718},{1733,2321},{2094,1781}}},
/*   0 */	{{{1958,2800},{2263,2618},{2545,2799},{2156,1715},{2295,2321},{2062,1784}}}
/* This is the structure for straight forward movement. The recorded OCR1A entries helps
 * instruct the servos to move forwards or backwards. Direction is controlled by 
 * incrementing or decrementing the step counter.
 */
}},{{
// ARM MOVEMENT 2-5-1-3-0-4 - CURVED
/*   0 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*   0 */	{{{1214,2814},{2346,2613},{3557,2289},{1158,1710},{1771,1811},{3247,1798}}},
/*  14 */	{{{1265,2817},{2378,2610},{2790,2289},{1211,1710},{1802,1804},{3300,1805}}},
/*  14 */	{{{1326,2820},{2419,2604},{2790,2789},{1263,1712},{1841,1796},{3343,1815}}},
/*  14 */	{{{1378,2822},{2458,2596},{2833,2799},{1324,1715},{1882,1790},{3343,2315}}},
/*  28 */	{{{1431,2822},{2489,2589},{2886,2806},{1375,1718},{1914,1787},{2576,2315}}},
/*  28 */	{{{1492,2820},{2527,2579},{2934,2814},{1424,1724},{1956,1784},{2576,1815}}},
/*  28 */	{{{1544,2817},{2527,2079},{2983,2820},{1472,1732},{1997,1782},{2619,1805}}},
/*  42 */	{{{1595,2814},{1965,2079},{3034,2823},{1525,1739},{2031,1782},{2664,1798}}},
/*  42 */	{{{1644,2808},{1965,2579},{3095,2826},{1568,1749},{2072,1784},{2720,1790}}},
/*  42 */	{{{1700,2800},{2003,2589},{3147,2828},{1568,2249},{2114,1787},{2769,1784}}},
/*  56 */	{{{1745,2793},{2034,2596},{3200,2828},{801,2249},{2147,1790},{2820,1781}}},
/*  56 */	{{{1788,2783},{2073,2604},{3261,2826},{801,1749},{2187,1796},{2872,1778}}},
/*  56 */	{{{1788,2283},{2114,2610},{3313,2823},{844,1739},{2226,1804},{2933,1776}}},
/*  70 */	{{{1021,2283},{2146,2613},{3364,2820},{889,1732},{2257,1811},{2986,1776}}},
/*  70 */	{{{1021,2783},{2188,2616},{3413,2814},{945,1724},{2295,1821},{3038,1778}}},
/*  70 */	{{{1064,2793},{2229,2618},{3469,2806},{994,1718},{2295,2321},{3099,1781}}},
/*   0 */	{{{1117,2800},{2263,2618},{3514,2799},{1045,1715},{1733,2321},{3150,1784}}}
/* This structure makes hexapod to turn left or right. The direction of the curve
 * is controlled by incrementing or decrementing the step counter.
 */
}},{{
// ARM MOVEMENT 2-5-1-3-0-4 - REQUEST_CURVED
/*   0 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*   0 */	{{{1997,2308},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  14 */	{{{1165,2308},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  14 */	{{{1165,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  14 */	{{{1165,2808},{2304,2116},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  28 */	{{{1165,2808},{2304,2116},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  28 */	{{{1165,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  28 */	{{{1165,2808},{2304,2616},{2583,2289},{2114,1712},{2295,1821},{2021,1790}}},
/*  42 */	{{{1165,2808},{2304,2616},{3557,2289},{2114,1712},{2295,1821},{2021,1790}}},
/*  42 */	{{{1165,2808},{2304,2616},{3557,2789},{2114,1712},{2295,1821},{2021,1790}}},
/*  42 */	{{{1165,2808},{2304,2616},{3557,2789},{2114,2212},{2295,1821},{2021,1790}}},
/*  56 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,2212},{2295,1821},{2021,1790}}},
/*  56 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{2295,1821},{2021,1790}}},
/*  56 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{2295,2321},{2021,1790}}},
/*  70 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,2321},{2021,1790}}},
/*  70 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{2021,1790}}},
/*  70 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{2021,2290}}},
/*   0 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,2290}}}
/* This structure makes hexapod to turn left or right. The direction of the curve
 * is controlled by incrementing or decrementing the step counter.
 */
}},{{
// ARM MOVEMENT 6-1-5-2-4-3: REQUEST_STRAIGHT
/*   0 */	{{{1165,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*   0 */	{{{1165,2308},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  14 */	{{{1997,2308},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  14 */	{{{1997,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  14 */	{{{1997,2808},{2304,2116},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  28 */	{{{1997,2808},{2304,2116},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  28 */	{{{1997,2808},{2304,2616},{3557,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  28 */	{{{1997,2808},{2304,2616},{3557,2289},{1097,1712},{1733,1821},{3199,1790}}},
/*  42 */	{{{1997,2808},{2304,2616},{2583,2289},{1097,1712},{1733,1821},{3199,1790}}},
/*  42 */	{{{1997,2808},{2304,2616},{2583,2789},{1097,1712},{1733,1821},{3199,1790}}},
/*  42 */	{{{1997,2808},{2304,2616},{2583,2789},{1097,2212},{1733,1821},{3199,1790}}},
/*  56 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,2212},{1733,1821},{3199,1790}}},
/*  56 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{1733,1821},{3199,1790}}},
/*  56 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{1733,2321},{3199,1790}}},
/*  70 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,2321},{3199,1790}}},
/*  70 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{3199,1790}}},
/*  70 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{3199,2290}}},
/*   0 */	{{{1997,2808},{2304,2616},{2583,2789},{2114,1712},{2295,1821},{2021,2290}}}
/* This structure makes hexapod to turn left or right. The direction of the curve
 * is controlled by incrementing or decrementing the step counter.
 */
}}};
#endif // _DINOSAURIA
//>>>
#else // _EEPROM_DATA
//<<<
const run_t run[] __attribute__ ((section(".eeprom"))) = {{{
/*
 *----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------           
 *          |   Servo 0  |  Servo 1  |  Servo 2  |  Servo 3  |  Servo 4  |  Servo 5  |
 *  Draha   |   Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |  Sh | El  |
 *----------+------+-----+-----+-----+-----+-----+-----+-----+-----------+------------*/           
/*   0 */	{{{1361,2700},{2647,2200},{2004,2980},{3134,2428},{1743,2729},{2439,2733}}},
/*  10 */	{{{1455,2803},{2575,2229},{2156,2988},{3087,2486},{1820,2760},{2274,2741}}},
/*  27 */	{{{1701,2929},{2218,2286},{2734,2929},{2765,2676},{2207,2822},{1647,2677}}},
/*  52 */	{{{2123,2988},{1861,2229},{3031,2753},{2140,2741},{2594,2760},{1325,2487}}},
/*  77 */	{{{2432,2981},{1789,2200},{3074,2700},{1975,2732},{2671,2729},{1279,2429}}},
/*  94 */	{{{2432,2981},{1789,2200},{3074,2700},{1975,2732},{2671,2729},{1279,2429}}},
/* 104 */	{{{2432,2981},{1789,2200},{3074,2700},{1975,2732},{2671,2729},{1279,2429}}},
                                                
/* 104 */	{{{2432,2481},{1789,2700},{3074,2200},{1975,3032},{2671,2429},{1279,2729}}},
/* 104 */	{{{2432,2481},{1789,2700},{3074,2200},{1975,3032},{2671,2429},{1279,2729}}},
/* 104 */	{{{2432,2481},{1789,2700},{3074,2200},{1975,3032},{2671,2429},{1279,2729}}},
/*  94 */	{{{2123,2488},{1861,2729},{3031,2253},{2140,3041},{2594,2460},{1325,2787}}},
/*  52 */	{{{1701,2429},{2218,2786},{2734,2429},{2765,2976},{2207,2522},{1647,2977}}},
/*  10 */	{{{1455,2303},{2575,2729},{2156,2488},{3087,2786},{1820,2460},{2274,3041}}},
/*   0 */	{{{1361,2200},{2647,2700},{2004,2480},{3134,2728},{1743,2429},{2439,3033}}}
/* This is the structure for straight forward movement. The recorded OCR1A entries helps
 * instruct the servos to move forwards or backwards. Direction is controlled by 
 * incrementing or decrementing the step counter.
 */
}},{{
/*   0 */	{{{3135,1685},{2671,1985},{2439,1382},{3134,2428},{2671,2729},{2439,2733}}},
/*  10 */	{{{3089,1627},{2594,1954},{2274,1374},{3087,2486},{2594,2760},{2274,2741}}},
/*  52 */	{{{2767,1437},{2207,1892},{1649,1438},{2765,2676},{2207,2822},{1647,2677}}},
/*  94 */	{{{2140,1373},{1820,1954},{1327,1628},{2140,2741},{1820,2760},{1325,2487}}},
/* 104 */	{{{1975,1381},{1743,1985},{1280,1686},{1975,2732},{1743,2729},{1279,2429}}},
/* 104 */	{{{1975,1381},{1743,1985},{1280,1686},{1975,2732},{1743,2729},{1279,2429}}},
/* 104 */	{{{1975,1381},{1743,1985},{1280,1686},{1975,2732},{1743,2729},{1279,2429}}},

/* 104 */	{{{1975,1681},{1743,1685},{1280,1986},{1975,3032},{1743,2429},{1279,2729}}},
/* 104 */	{{{1975,1681},{1743,1685},{1280,1986},{1975,3032},{1743,2429},{1279,2729}}},
/* 104 */	{{{1975,1681},{1743,1685},{1280,1986},{1975,3032},{1743,2429},{1279,2729}}},
/*  94 */	{{{2140,1673},{1820,1654},{1327,1928},{2140,3041},{1820,2460},{1325,2787}}},
/*  52 */	{{{2767,1737},{2207,1592},{1649,1738},{2765,2976},{2207,2522},{1647,2977}}},
/*  10 */	{{{3089,1927},{2594,1654},{2274,1674},{3087,2786},{2594,2460},{2274,3041}}},
/*   0 */	{{{3135,1985},{2671,1685},{2439,1682},{3134,2728},{2671,2429},{2439,3033}}}
/* This structure makes hexapod to turn left or right. The direction of the curve
 * is controlled by incrementing or decrementing the step counter.
 */
}}};
#endif // _PROGMEM_DATA
//>>>
/* TABULATED MOVEMENT DATA
 * -----------------------
 * This implementation does not utilize the real time conversion of the x-y-z
 * data into the alpha-beta-gamma angles. It uses tabulated entries instead.
 * The calculation is done inside a help data - kinetics.odt. Resulting
 * values are copied over into this static structures.
 */

#ifdef MATH
//<<<
struct _M_angles_t {
	double		beta;
	double 		gama;
	uint8_t		s;
};
typedef struct _M_angles_t _M_angles_t;

struct _M_servo_t {
	uint16_t		angle;
	uint8_t		reversed;
};
typedef struct _M_servo_t _M_servo_t;

struct _M_arm_t {
	_M_servo_t	servo[2];
	double		gama_0;
	double		a;
};
typedef struct _M_arm_t _M_arm_t;

struct _M_servos_t {
	_M_arm_t		arm[6];
	uint8_t	index;
};
typedef struct _M_servos_t _M_servos_t;
/* These structures are required when x,y,z -> alpha,beta,gamma
 * conversion is calculated inside the controller itself. The
 * calculation is however rather resource consuming and higher model 
 * AVR must be used. Mega8 does not have enough main memory to
 * host the mathematical library and logic of movement. 
 */
//>>>
#endif //MATH
/* With this flag activated the formware performs the real-time calculation
 * of the servo angles based on the x-y-z coordinates of the tip of the 
 * arm. This structures are used from within the main.math.c only.
 */


   /***********************/
   /* FUNCTION PROTOTYPES */
   /***********************/

void servo_shield_port_init(); 
/* HW configuration method. Configured pins PD5 though PD7 as output pins.
 * Those pins then controls the Master Reset (MR) of both used decade counters
 * and clocking them.
 */

void servo_shield_init( void );
/* This command configures the internal, 16-bit timer1 for the CTC mode
 * (compare match). Controller generates compare match interrupt when the
 * internal counter overflows the preset value (OCR1A). This routine also sets
 * the speed of counting by setting the desired prescaler. The prescaler
 * divides the system clock by factor (8,64,256,or 1024). 
 */
 
void servo_shield_cntrl ( type_t );
/* This method enables or disables the interrupt generated by compare match
 * overflow.
 */

void srf02_init( void );
/* This function initializes the timer/counter0.  It configures the counting
 * clock and resets the counter to zero.
 */

void srf02_cntrl( type_t );
/* Enables or disables the overflow interrupt on Timer0
 */

void fill_servo_page( move_t );
/* This function copies one page of the preset movement instructions from EEPROM
 * or PROGMEM into the servo latch buffer. The latch buffer has three stages.
 * One page contain instructions for 12 servos. The whole page is transferred
 * into the first stage of the latch buffer. The copy operation shifts the
 * content of obsoleted first stage into the second stage before overwriting the
 * first stage with new data. The third stage holds the current data for servo
 * movement and is updated by 'fill_servo_buffer()' routine only. 
 */

void fill_servo_buffer( move_t, speed_t );
/* Movement function. This function determines the current data for a servo
 * based of the content of the first and second stage of the latch buffer.
 * Technically, the function calculates the linear regression between two
 * tabulated entries. The move_t structure defines the two pointers to the field
 * of tabulated entries for the movement and speed_t brings then data to
 * calculate the linear regression between tabulated points. User can define the
 * granularity of the regression, which in turns translates into smoothness of
 * the movement and speed of the movement. Carefully selected data produces
 * realistic movements of the arms of the robot. 
 */

void swap_bank_servo_page( void );
/* Global function: this routine process the queue generated by ranging process.
 * Ranging is rise the change request and stores it in the queue. This manager
 * process the queue in the moment where the full block of servo instructions
 * get processed (each block has 18 rows - a full step). This manager vistually
 * swaps the banks that are taken to fill the servo latch buffer.
 */

void init( void );
/* Performs basic initialization of the basic status. 
 */

move_vector_t determine_next_move( uint8_t vec );
/* This function determines the move direction based on ranging data. The robot
 * movement is based on 6 input factors.
 *
 *    ERR  - Ranging error
 *    ABS  - Distance to the detected object in centimeters.
 *    der1 - first derivation of distance (time)
 *    der2 - second derivation of distance (time)
 *    mov  - current movement (STRAIGHT, CURVED)
 *    dir  - current direction (REVERSE, STOP, FORWARD)
 *
 * The logical diagram using those data is captured here:
 *
 *        V
 *        |
 *        |
 *        V          MSG_M_STOP
 *       ERR? --1--> d:STOP 
 *        |
 *        0
 *        |
 *        V                MSG_M_FORW
 *    ABS < LIMIT? ---0--> m:STAIGHT
 *        |                d:FORWARD
 *        1                i:3sec
 *        |                d2l:-6
 *        |
 *        V            MSG_M_NOP          MSG_M_NOP         MSG_M_TURN_1
 *    der1 < 0? --0--> m:STRAIGHT? --1--> d:REVERSE? --1--> m:curved
 *        |            i:11sec            i:11sec           i:11sec
 *        1                
 *        |
 *        V                               MSG_M_TURN_0
 *    der2 < 0? --0--> m:STRAIGHT? --1--> m:CURVED
 *        |                |              d:REVERSE
 *        1                0              i:11sec
 *        |                |
 *        V                V             MSG_M_LEFT
 *    MGS_M_REV        d:REVERSE? --1--> d:FORWARD
 *    m:STRAIGHT           |             i:22sec
 *    d:REVERSE            0
 *    i:11sec              |
 *    d2l:-200             V
 *                     MSG_M_RIGHT
 *                     d:REVERSE   
 *                     i:34sec
 *
 * This diagram is converted into a vector and vector is delivered to the
 * function that determined the action (indicated by the --SET-- elements in
 * the diagram).
 *
 * The vector is calculated according to this schema:
 *
 * (76543210)
 *    ||||||
 *    |||||+- D : direction bit. The bit represents direction status
 *    |||||       the bit = 1 if direction = REVERSE, otherwise 0
 *    ||||+-- M : movement bit. This bit represents movement status
 *    ||||        the bit = 1 if movement = STRAIGHT, otherwise 0
 *    |||+--- d2: 2nd derivation of distance. The bit is 1 if d2 < 0.
 *    |||         The negative second derivation indicates that the robot 
 *    |||         accelerates to approach an obstacle. In reality it means 
 *    |||         that an obstacle accelerates against the robot.
 *    ||+---- d1: 1st derivation of distance. The bit is 1 if d1 < 0.
 *    ||          The negative first derivation indicates that the robot
 *    ||          steadily approaches stable, fixed obstacle (not moving).
 *    |+----- A : Range status. The bit is 1 if range is bellow safe limit, 
 *    |           otherwise 0.
 *    +------ E : Error bit. This bit is set 1 if ranging structure contains
 *                no data or wrong data. The error may occur if ranging device
 *                fails or timing collision occur. Robot stops is this flag
 *                is set to 1.      
 *               
 *  The supported combination of control bits describes following table:
 *
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  | ERR | ABS | der1 | der2 | mov | dir | handler | Handler synopsis       |
 *  |     |     |      |      |     |     |  code   |                        |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  1  |  x  |  x   |  x   |  x  |  x  |    0    | d:STOP                 |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  0  |  x   |  x   |  x  |  x  |    1    | m:STRAIGHT, d:FORWARD  |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  1  |  1   |  1   |  x  |  x  |    2    | m:STRAIGHT, d:REVERSE  |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  1  |  1   |  0   |  1  |  x  |    3    | m:CURVED, d:REVERSE    |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  1  |  0   |  x   |  1  |  1  |    4    | m:CURVED               |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  1  |  1   |  0   |  0  |  1  |    5    | d:FORWARD              |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *  |  0  |  1  |  1   |  0   |  0  |  0  |    6    | d:REVERSE              |
 *  +-----+-----+------+------+-----+-----+---------+------------------------+
 *
 *  This function delivers the index (handler code) of the handler and the
 *  handler muss be called by means of standard VTable.
 */ 


void process_ranging_data( void );
/* Takes the latest ranging and determine first and second derivative product
 * All results are stored in the global structure srf02_t.
 */

uint8_t get_app_vector( srf02_t d, move_t m, srf02_cntrl_t c );
/* this is a helper function that calculates the vector for the routine
 * determine_next_move().
 */


signal_t semaphore_signal( resource_id_t res );
/* Sempahore accessing a resource under the ID given in argument.
 * Functional return "SIGNAL" if the sempahore is already set,
 * otherwise signals the sempahore and return CLEAR. 
 */

void semaphore_release( resource_id_t res );
/* Releases semaphore for object speficied by the parameter
 */

void semaphore_signal_all( uint8_t );
/* Presets the status of the semaphore to value given in parameter
 */ 

#ifdef MATH
//<<<
angles_t angles( double, _M_arm_t* );
uint16_t servo_ctrl ( double );
/* Routines calculating the value for angles Gamma and Beta (Alpha is fixed,
 * while the connection between COXA and FEMUR is fixed at 45 degrees). The 
 * input parameter is the track value (0-104 mm).
 */
#endif
//>>>

#ifdef USART_DEBUG
//<<<
void	USART_init();
#endif
//>>>

#endif
