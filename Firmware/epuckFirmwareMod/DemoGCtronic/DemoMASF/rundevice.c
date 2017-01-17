#include "p30f6014A.h"
// Controlled driving
// Based on Asercom.c
// ATr (talex@student.ethz.ch)

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include <motor_led/e_epuck_ports.h> //General initialization
#include <motor_led/e_init_port.h> //General initialization
#include <motor_led/e_led.h> //LED operation
#include <motor_led/e_motors.h> //Motor operation
#include <uart/e_uart_char.h> //Communication
#include <motor_led/advance_one_timer/e_agenda.h> //Agenda functionality (periodic execution of a function)
#include <codec/e_sound.h> //Speaker operation
//#include <a_d/advance_ad_scan/e_ad_conv.h> //AD converter for Microphones, Proximity Sensors and Accelerometer
//#include <a_d/advance_ad_scan/e_micro.h> //Microphone operation
//#include <fft/e_fft.h> //Fast Fourier Transform using DSP library

#include "utility.h"

#include "DataEEPROM.h"
#include "memory.h"

#include "rundevice.h"


extern char buffer[BUFFER_SIZE];
extern int selector;
extern char c;

#define uart1_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart1_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)
#define uart2_send_static_text(msg) do { e_send_uart2_char(msg,sizeof(msg)-1); while(e_uart2_sending()); } while(0)
#define uart2_send_text(msg) do { e_send_uart2_char(msg,strlen(msg)); while(e_uart2_sending()); } while(0)

//#define SOUNDT 1500 //sound threshold
 #define PI 3.14159265358979

//const int NStepsPerRobRot = 1293; // Calculated: 1293; // number of motor steps needed to make a one full rotation (with equal wheel speeds in opposite directions)
//const int NStepsPerMeter = 7764;  // Calculated: 7764;  // number of motor steps needed to travel 1m (with equal wheel speeds)


//ePuck
static const float par_v_max = 12.88; // Maximal speed [cm/s];
static const float par_d_wheels = 4.1; // Diameter of the wheels [cm] 
static const float par_b_wheels = 5.3; // Distance between wheels [cm]

static float cs_x,cs_y,cs_a,es_x,es_y,es_a;


static float gamma,omega_A,omega_B,Length_straight, omega;
static float driven_Length;

static float dalpha_A ,dalpha_B,dl_C, dl_L; 
static int ul_A, ur_A, ul_B, ur_B, u_straight;

static int navigation_Ts = 200; // in 0.1 ms, don't set too small (i.e. 10), for "M,N"


static float closed_loops_Ts = 3.333; // [s] sampling time of the algorithm, the speed will be adjusted


//static float v_circ; // [m/s] average velocity, 6.44


static const float R = 2.65; // [cm] radius of circles
static int circle_flag = 0; // indicater in which phase of movement
static int change;// =  1; // boolean to indicate whether phase of movement has changed
static float angle_tol;// = 0.005; // [rad] tolerance on the angle

static int u_1k[2];
static int vmin = 200;


//-----------------------------------


void run_device(void) {

// init
	int iStop = 1;
	int iObstacleAvoidance = 0;


	static int	i,positionr,positionl,LED_nbr,action;
	static char first=0;



	int use_bt;
	selector = getselector(); //SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
	if(selector==10) {
		use_bt=0;
	} else {
		use_bt=1;
	}


	if(RCONbits.POR) {	// reset if power on (some problem for few robots)
		RCONbits.POR=0;
		RESET();
	}

	/*read HW version from the eeprom (last word)*/
	static int HWversion=0xFFFF;
	ReadEE(0x7F,0xFFFE,&HWversion, 1);
	

	uart1_send_static_text("\f\a"
			"Greetings, Master. \r\n"
			"WELCOME to the SerCom protocol on e-Puck\r\n"
			"the EPFL education robot type \"H\" for help\r\n");


	while(1) {
		
		while (e_getchar_uart1(&c)==0)
		{}
	
			
			while (c=='\n' || c=='\r') e_getchar_uart1(&c);

			buffer[0]=c;
			i = 1;

			do if (e_getchar_uart1(&c)) buffer[i++]=c;
			while (c!='\n' && c!='\r');				

			buffer[i++]='\0';

			if((buffer[0] != 'B') && (buffer[0] != 'b')) {
				buffer[0]=toupper(buffer[0]); // we accept lowercase letters except for 'b'
			}
			switch (buffer[0]) {
			case 'b':	// battery is ok?
				sprintf(buffer,"b,%d\r\n", BATT_LOW);	// BATT_LOW=1 => battery ok, BATT_LOW=0 => battery<3.4V
				//sprintf(buffer,"b,0\r\n"); //use this to test the battery exchange function
				uart1_send_text(buffer);
			
				break;
			case 'B': // set body led
				sscanf(buffer,"B,%d\r",&action);
				e_set_body_led(action);
				uart1_send_static_text("b\r\n");

				break;			
			case 'D': // set motor speed
				sscanf(buffer, "D,%d,%d\r", &u_1k[0], &u_1k[1]);
				e_set_speed_left(u_1k[0]);
				e_set_speed_right(u_1k[1]);
				//sprintf(buffer,"D,%d,%d\r\n", u_1k[0],u_1k[1]);
				//uart1_send_text(buffer);

				iStop = 0;
				break;
			case 'E': // read motor speed
				sprintf(buffer,"e,%d,%d\r\n",u_1k[0],u_1k[1]);
				uart1_send_text(buffer);
				break;
			case 'F':
				sscanf(buffer, "F,%d\r", &vmin);
			break;
			case 'H': // ask for help
					uart1_send_static_text("\n");
					uart1_send_static_text("\"A\"         Accelerometer\r\n");
					uart1_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
					uart1_send_static_text("\"b\"		  Battery ok?\r\n");
					uart1_send_static_text("\"C\"         Selector position\r\n");
					uart1_send_static_text("\"D,#,#\"     Set motor speed left,right\r\n");
					uart1_send_static_text("\"E\"         Get motor speed left,right\r\n");
					uart1_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
					uart1_send_static_text("\"H\"	      Help\r\n");
					uart1_send_static_text("\"L,#,#\"     Led number,0=off 1=on 2=inverse\r\n");
					uart1_send_static_text("\"P,#,#\"     Set motor position left,right\r\n");
					uart1_send_static_text("\"Q\"         Get motor position left,right\r\n");
					uart1_send_static_text("\"R\"         Reset e-puck\r\n");
					uart1_send_static_text("\"S\"         Stop e-puck and turn off leds\r\n");
					uart1_send_static_text("\"T,#\"       Play sound 1-5 else stop sound\r\n");
					uart1_send_static_text("\"U\"         Get microphone amplitude\r\n");
					uart1_send_static_text("\"V\"         Version of SerCom\r\n");

				break;
			case 'K': //similar to N, but without the sampling time, the robot just will drive as fast as possible
				// Use this function to drive from 'cs' to 'es'
				sscanf(buffer,"K,%f,%f,%f,%f,%f,%f\r\n",&cs_x,&cs_y,&cs_a,&es_x,&es_y,&es_a);	
				//sprintf(buffer,"K,%f,%f,%f,%f,%f,%f\r\n",cs_x,cs_y,cs_a, es_x,es_y,es_a);
				//uart1_send_text(buffer);
				e_destroy_agenda(Circle_Drive);
				circle_flag = 0;
				change = 1;
				navigation();
				break;
			break;
			case 'L': // set led
				sscanf(buffer,"L,%d,%d\r",&LED_nbr,&action);
				e_set_led(LED_nbr,action);
				uart1_send_static_text("l\r\n");
				break;
			case 'M':
			// Drives to predefined states, for testing
				sscanf(buffer,"M,%d\r\n",&action);
				cs_x = 0;
				cs_y = 0;
				cs_a = 0;
				if ( action == 1){

			
				es_x = 200;
				es_y = 0;
				es_a = 0;					
				}else if ( action == 2){
			
				es_x = 100;
				es_y = 0;
				es_a = 0;	
				}else if ( action == 3){

				es_x = 50;
				es_y = 0;
				es_a = 0;	

				}else if ( action == 4){

				es_x = 20;
				es_y = 0;
				es_a = -PI/2;	

				}
				circle_flag = 0;
				change = 1;
				sprintf(buffer,"M,%f,%f,%f,%f,%f,%f\r\n",cs_x,cs_y,cs_a, es_x,es_y,es_a);
				uart1_send_text(buffer);

				navigation();
				break;
			case 'N':
			// Use this function to drive from 'cs' to 'es'
				sscanf(buffer,"N,%f,%f,%f,%f,%f,%f\r\n",&cs_x,&cs_y,&cs_a,&es_x,&es_y,&es_a);	
				//sprintf(buffer,"N,%f,%f,%f,%f,%f,%f\r\n",cs_x,cs_y,cs_a, es_x,es_y,es_a);
				//uart1_send_text(buffer);
				e_destroy_agenda(Circle_Drive);
				circle_flag = 0;
				change = 1;
				navigation();
				break;
			case 'O':
			// Set sampling time
				sscanf(buffer,"O,%f\r\n",&closed_loops_Ts);
				//sprintf(buffer,"O,%f\r\n",closed_loops_Ts);
				//uart1_send_text(buffer);
				break;
			case 'P': // set motor position
				sscanf(buffer,"P,%d,%d\r",&positionl,&positionr);
				e_set_steps_left(positionl);
				e_set_steps_right(positionr);
				uart1_send_static_text("p\r\n");

				break;
			case 'Q': // read motor position
				sprintf(buffer,"Q,%d,%d\r\n",e_get_steps_left(),e_get_steps_right());
				uart1_send_text(buffer);

				break;
			case 'R': // reset
				uart1_send_static_text("r\r\n");
				RESET();
				break;
			case 'S': // stop
				//e_destroy_agenda(CtrlSetSpeed);
				e_destroy_agenda(Circle_Drive);
				u_1k[0]=0;
				u_1k[1]=0;
				e_set_speed_left(u_1k[0]);
				e_set_speed_right(u_1k[1]);

				iStop = 1;

				uart1_send_static_text("s\r\n");

				break;
			case 'T': // Play sound
					sscanf(buffer,"T,%d",&action);
					if(first==0){
						e_init_sound();
						first=1;
					}
					switch(action)
					{
						case 1: e_play_sound(0,720);break;
						default:
							e_close_sound();
							first=0;
							break;
					}				
					//sprintf(buffer,"t,%d\r\n",action);	
					uart1_send_static_text("t\r\n");

				break;

			case 'V': // get version information
				uart1_send_static_text("v,Hacked! Based on Version 1.2.2 August 2008 GCtronic\r\n");
				sprintf(buffer,"HW version: %X\r\n",HWversion);
				uart1_send_text(buffer);

				break;			
			case 'X': //turn ObstacleAvoidance on
				iObstacleAvoidance = 1;
				uart1_send_static_text("x\r\n");
				break;

			case '0': //turn ObstacleAvoidance off
				iObstacleAvoidance = 0;
				uart1_send_static_text("0\r\n");
				break;

			default:
				uart1_send_static_text("z,Command not found\r\n");
				break;
			}
				

	}
}




void navigation(void)
{
	cs_a = normalize_angle(cs_a);
	es_a = normalize_angle(es_a);

	float Length[4];
	float mLength[4];
	float circ_angle_tol = 0.001;

//------case left-left -------------
	float A_ll_x = cs_x-R*sinf(cs_a);
	float A_ll_y = cs_y+R*cosf(cs_a);

	float B_ll_x = es_x-R*sinf(es_a);
	float B_ll_y = es_y+R*cosf(es_a);

	float delta_x = B_ll_x-A_ll_x;	
	float delta_y = B_ll_y-A_ll_y;

	mLength[0] = sqrt(delta_x*delta_x+delta_y*delta_y);
	float gamma_ll=atan2f(delta_y,delta_x);

	gamma_ll=normalize_angle(gamma_ll);

	float alpha = gamma_ll - cs_a;
	if( fabsf(alpha) < circ_angle_tol )
	{
		alpha = 0;	
	}else
	{
		alpha = normalize_angle(alpha);
	}
	float beta = es_a - gamma_ll;
	if( fabsf(beta) < circ_angle_tol )
	{
		beta = 0;	
	}else
	{
		beta = normalize_angle(beta);
	}	



	Length[0] = mLength[0]+R*(alpha+beta);	


//------case right-right -------------
	float A_rr_x = cs_x+R*sinf(cs_a);
	float A_rr_y = cs_y-R*cosf(cs_a);

	float B_rr_x = es_x+R*sinf(es_a);
	float B_rr_y = es_y-R*cosf(es_a);

	delta_x = B_rr_x-A_rr_x;	
	delta_y = B_rr_y-A_rr_y;
	
	mLength[1] = sqrt(delta_x*delta_x+delta_y*delta_y);

	float gamma_rr=atan2f(delta_y,delta_x);

	gamma_rr=normalize_angle(gamma_rr);

	alpha = cs_a - gamma_rr;
	if( fabsf(alpha) < circ_angle_tol )
	{
		alpha = 0;	
	}else
	{
		alpha = normalize_angle(alpha);
	}
	beta = gamma_rr - es_a;
	if( fabsf(beta) < circ_angle_tol )
	{
		beta = 0;	
	}else
	{
		beta = normalize_angle(beta);
	}	

	Length[1]  = mLength[1]+R*(alpha+beta);

//------case left-right -------------
	float A_lr_x = cs_x-R*sinf(cs_a);
	float A_lr_y = cs_y+R*cosf(cs_a);

	float B_lr_x = es_x+R*sinf(es_a);
	float B_lr_y = es_y-R*cosf(es_a);

	delta_x = B_lr_x-A_lr_x;	
	delta_y = B_lr_y-A_lr_y;
	
	float L_sqr = delta_x*delta_x+delta_y*delta_y;

	float gamma_lr ,P_lr_x,P_lr_y; //,Q_lr_x,Q_lr_y ;
	float ZZ, zzx, zzy;
	if (L_sqr < 4*R*R){ // Lu would be imaginary, do not allow this
		Length[2] = 100000;
	}else{
		mLength[2] = sqrt(L_sqr-4*R*R);

		gamma_lr=asinf((2*R*delta_x+delta_y*mLength[2])/L_sqr);


		P_lr_x = A_lr_x+R*sinf(gamma_lr);
		P_lr_y = A_lr_y-R*cosf(gamma_lr);
		
		zzx = P_lr_x + mLength[2]*cosf(gamma_lr) - B_lr_x;
		zzy = P_lr_y + mLength[2]*sinf(gamma_lr) - B_lr_y;
		ZZ= sqrt(zzx*zzx + zzy*zzy);
		if( fabsf(ZZ-R) > 0.01) //choose the other gamme
		{
			gamma_lr = PI-gamma_lr;
			P_lr_x = A_lr_x+R*sinf(gamma_lr);
			P_lr_y = A_lr_y-R*cosf(gamma_lr);
		}
		gamma_lr = normalize_angle(gamma_lr);

		alpha = gamma_lr - cs_a;
		if( fabsf(alpha) < circ_angle_tol )
		{
			alpha = 0;	
		}else
		{
			alpha = normalize_angle(alpha);
		}
		beta = gamma_lr - es_a;
		if( fabsf(beta) < circ_angle_tol )
		{
			beta = 0;	
		}else
		{
			beta= normalize_angle(beta);
		}


		Length[2]  = mLength[2] + R*(alpha+beta);

	}
//------case right-left -------------
	float A_rl_x = cs_x+R*sinf(cs_a);
	float A_rl_y = cs_y-R*cosf(cs_a);

	float B_rl_x = es_x-R*sinf(es_a);
	float B_rl_y = es_y+R*cosf(es_a);

	delta_x = B_rl_x - A_rl_x;	
	delta_y = B_rl_y - A_rl_y;
	L_sqr = delta_x*delta_x+delta_y*delta_y;

	float gamma_rl, P_rl_x, P_rl_y;//,Q_rl_x,Q_rl_y;	

	if (L_sqr < 4*R*R){ // Lu would be imaginary, do not allow this
		Length[3] = 100000;
	}else{
		mLength[3] = sqrt(L_sqr-4*R*R);
		gamma_rl=asinf((-2*R*delta_x+delta_y*mLength[3])/L_sqr);
		
		P_rl_x = A_rl_x - R*sinf(gamma_rl);
		P_rl_y = A_rl_y + R*cosf(gamma_rl);

		zzx = P_rl_x + mLength[3]*cosf(gamma_rl) - B_rl_x;
		zzy = P_rl_y + mLength[3]*sinf(gamma_rl) - B_rl_y;
		ZZ= sqrt(zzx*zzx + zzy*zzy);
		if( fabsf(ZZ-R) > 0.01) //choose the other gamma
		{
			gamma_rl = PI - gamma_rl;
			P_rl_x = A_rl_x - R*sinf(gamma_rl);
			P_rl_y = A_rl_y + R*cosf(gamma_rl);
		}
		gamma_rl = normalize_angle(gamma_rl);
		
		alpha = cs_a - gamma_rl;
		if( fabsf(alpha) < circ_angle_tol )
		{
			alpha = 0;	
		}else
		{
			alpha = normalize_angle(alpha);
		}
		beta = es_a - gamma_rl;
		if( fabsf(beta) < circ_angle_tol )
		{
			beta = 0;	
		}else
		{
			beta = normalize_angle(beta);
		}

		Length[3] = mLength[3]+R*(alpha+beta);

	}
//--------CHOOSE TYPE OF MOVEMENT----------
// choose the shortest one
	float min_Length = 100000;
	int index = 0;
	int i;
	for (i =0;i<=3;i++)
	{
		if (Length[i] < min_Length)
		{
			min_Length = Length[i];
			index = i;
		}
	}	
	Length_straight = mLength[index];



//--------CALCULATE V-----------------
// This depends on N or K
	float v_circ, v_straight,maxTime, TimeCirc;
	float par_v_max_strich = par_v_max/(1+0.5/R*par_b_wheels);

	if (buffer[0] == 'N' || buffer[0] == 'M')
	{
		v_circ = min_Length/closed_loops_Ts;
		v_straight = v_circ; 
		omega = v_circ/R;

		if (v_circ > par_v_max_strich) // if constant speed is not posssible, allow max speed on straight
		{	
			TimeCirc = (min_Length - Length_straight) /par_v_max_strich;
			maxTime =  TimeCirc + Length_straight / par_v_max ;
			if(maxTime < closed_loops_Ts)
			{
				omega = par_v_max_strich/R;
				v_circ = par_v_max_strich;
				v_straight = Length_straight / (closed_loops_Ts-TimeCirc);
			}else  // it is not possible to go there in time
			{
			//uart1_send_static_text("ERROR: Not possible to reach target in time!\r\n");	
			return;
			}
		}


	} else if( buffer[0] == 'K' )
	{
		v_circ = par_v_max_strich;
		omega = v_circ / R;
		v_straight = par_v_max;
	}

	switch (index){
	case 0: // left-left
		gamma = gamma_ll;
		omega_A = omega;
		omega_B = omega;
		break;
		case 1: // right-right
		gamma = gamma_rr;
		omega_A = -omega;
		omega_B = -omega;
		break;
	case 2: // left-right
		gamma = gamma_lr;
		omega_A = omega;
		omega_B = -omega;
		break;
	case 3: // right-left
		gamma = gamma_rl;
		omega_A = -omega;
		omega_B = omega;
		break;
	}
//--------------------------------------------------------------------
		angle_tol = 0.5*omega*navigation_Ts/10000 + 0.0002;// adjust angle tolerance to velocity

		dalpha_A = omega_A*navigation_Ts/10000;
		dalpha_B = omega_B*navigation_Ts/10000;
	
		dl_C = v_circ * navigation_Ts/10000;
	
		dl_L = v_straight * navigation_Ts/10000;
	
	// Set the speeds
	
	
		ul_A = (int)( (v_circ  - omega_A*par_b_wheels/2) /par_v_max*1000);
		ur_A = (int)( (v_circ  + omega_A*par_b_wheels/2) /par_v_max*1000);
	
		ul_B = (int)( (v_circ  - omega_B*par_b_wheels/2) /par_v_max*1000);
		ur_B = (int)( (v_circ  + omega_B*par_b_wheels/2) /par_v_max*1000);
		
		u_straight = (int) ( v_straight / par_v_max * 1000);


//	sprintf(buffer,"%d,%d\r\n%d,%d\r\n,%d\r\n", ul_A,ur_A,ul_B,ur_B, u_straight);
//	uart1_send_text(buffer);
//	sprintf(buffer,"%f,%f\r\n", dl_C,dl_L);
//	uart1_send_text(buffer);


	driven_Length = 0;
	e_activate_agenda(Circle_Drive,navigation_Ts);

	
	return;
}


void Circle_Drive(void)
{	//uses ul_A, ur_A, u_straight, ul_B, ur_B, gamma, circle flag, angle tol, cs_a, es_a, dl_C, dl_L
	
	if (circle_flag == 0 && ( fabsf(gamma-cs_a) <= angle_tol || fabsf(fabsf(gamma-cs_a)-2*PI) <= angle_tol ) ) // check if leaving circle A
	{
		circle_flag = 1;
		change = 1;
	}
	else if (circle_flag == 1 && driven_Length > Length_straight) // check if arrived at Q
	{
		circle_flag = 2;
		change = 1;
		if(fabsf(es_a-cs_a) <= angle_tol || fabsf(fabsf(es_a-cs_a)-2*PI) <= angle_tol) // if cs_a == es_a, vehicle has arrived (rare case)
		{
			circle_flag = 3;
		}
	}
	else if ( circle_flag == 2 && ( fabsf(es_a-cs_a) <= angle_tol || fabsf(fabsf(es_a-cs_a)-2*PI) <= angle_tol ) ) // check if arrived at es
	{
		circle_flag = 3;
		change = 1;
	}		
//----------------------------------------------------------


	if (circle_flag == 0){

		u_1k[0] = ul_A;
		u_1k[1] = ur_A;

		cs_x += dl_C*cosf(cs_a);
		cs_y += dl_C*sinf(cs_a);
		cs_a += dalpha_A;
		cs_a = normalize_angle(cs_a);

	}else if (circle_flag == 1){

		u_1k[0] = u_straight;
		u_1k[1] = u_straight;		
		
		cs_x += dl_L*cosf(cs_a);
		cs_y += dl_L*sinf(cs_a);

		driven_Length += dl_L;

	}else if(circle_flag == 2){

		u_1k[0] = ul_B;
		u_1k[1] = ur_B;

		cs_x += dl_C*cosf(cs_a);
		cs_y += dl_C*sinf(cs_a);
		cs_a += dalpha_B;
		cs_a = normalize_angle(cs_a);

	}else if(circle_flag == 3){
		e_destroy_agenda(Circle_Drive);
		u_1k[0] = vmin;
		u_1k[1] = vmin;
		
//		if(buffer[0] == 'K' || buffer[0] == 'M')
//		{
//			sprintf(buffer,"%d\r\n", 1);
//			uart1_send_text(buffer);
//		}
//		uart1_send_static_text("I have arrived at:\r\n");
//		sprintf(buffer,"%f,%f,%f\r\n", cs_x,cs_y,cs_a);
//		uart1_send_text(buffer);
	}
	

	if(change)
	{
		e_set_speed_left(u_1k[0]);
		e_set_speed_right(u_1k[1]);
	}

	change = 0;
//	sprintf(buffer,"%f,%f,%f,%d,%d,%d\r\n", cs_x,cs_y,cs_a,u_1k[0],u_1k[1],circle_flag);
//	uart1_send_text(buffer);	
	return;
}



float normalize_angle(float alpha){
// wraps alpha into [0,2*pi]
	float beta;
	beta = fmodf(alpha,2*PI);
	if ( beta < 0){ beta += 2*PI;}
	return beta;
}

