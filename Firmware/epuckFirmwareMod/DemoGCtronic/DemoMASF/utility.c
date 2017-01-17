#include "math.h"
#include <motor_led/e_epuck_ports.h>
#define PI 3.14159265358979

void wait(long num) {
	long i;
	for(i=0;i<num;i++);
}

int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

//float norm_angle(float angle) {
//if (angle >= 0.0)
//{
//	angle = fmodf(angle +PI,2.0*PI)-PI; //Making sure the angle is inside the interval [-PI,PI]
//} else {
//	angle = fmodf(angle -PI,2.0*PI)+PI; //Making sure the angle is inside the interval [-PI,PI]
//}
//}
