#include "math.h"
#include <stdio.h>
#include <motor_led/e_epuck_ports.h>
#define PI 3.14159265358979

void wait(long num) {
	long i;
	for(i=0;i<num;i++);
}

int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

float norm_angle(float angle) {
	float angle_out;
	if (angle >= 0.0)
	{
		angle_out = fmodf(angle +PI,2.0*PI)-PI; //Making sure the angle is inside the interval [-PI,PI]
	} else {
		angle_out = fmodf(angle -PI,2.0*PI)+PI; //Making sure the angle is inside the interval [-PI,PI]
	}
	return angle_out;
}

int isNaN(float number)
{
	if (!(number<0.0) & ((int)number == -1)) //NaN is not smaller than 0, but assumes value -1 if converted to integer
	{
		return 1;
	} else
	{
		return 0;
	}
}

int find_max_id (int* vector, unsigned int vec_length)
{
	int i;
	int max_id = 0;
	for (i=1; i<vec_length; i++)
	{
		if (vector[i]>vector[max_id])
		{
			max_id = i;
		}
	}	
	return max_id;	
}