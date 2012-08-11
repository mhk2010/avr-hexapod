#include "hexapod.h"

angles_t angles( double x, arm_t *arm )
{
	// gama - servo[0]
	// beta - servo[1]
	angles_t	A;
	double s;
	double temp;

	s = sqrt((arm->a*arm->a) + (x*x) - (2*arm->a*x*cos(arm->gama_0)));
	A.s = (uint8_t) lrint( s );

	A.gama = acos(((arm->a - (2*x*cos(arm->gama_0)))/s)); 
	temp = (double) arm->a * (double) arm->a;
	temp = temp + (double) (s * s);
	temp = temp - ((double) x * (double) x);
	A.gama = acos((temp/(2 * (double) arm->a * (double) s)));

	temp = (double) FEMUR * (double) FEMUR;
	temp = temp + ((double) TIBIA * (double) TIBIA);
	temp = temp - ((double) HEIGHT * (double) HEIGHT);
	temp = temp - ((double) s * (double) s);
	A.beta = acos((temp/(2 * (double)FEMUR * (double)TIBIA)));

	return A;
}

uint16_t servo_ctrl ( double angle )
{
	double temp;

	temp = angle / (double) M_PI;
	temp = temp + 1;
	temp = (double) F_CPU * temp / 8000;
	return (uint16_t) (lrint( temp ) - 1);
}
