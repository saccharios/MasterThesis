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
#include <a_d/advance_ad_scan/e_ad_conv.h> //AD converter for Microphones, Proximity Sensors and Accelerometer
#include <a_d/advance_ad_scan/e_micro.h> //Microphone operation
#include <fft/e_fft.h> //Fast Fourier Transform using DSP library

#include "init_EKF.h" 
#include "memory.h"
#include "runcontroldrive.h"
#include "utility.h"

extern char buffer[BUFFER_SIZE];
extern int e_mic_scan[3][MIC_SAMP_NB];
extern fractcomplex sigCmpx[FFT_BLOCK_LENGTH]; //Storage in Y-Memory for FFT
extern unsigned int e_last_mic_scan_id;
extern unsigned int e_dci_unavailable;
extern int selector;
extern char c;

#define uart1_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart1_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)
#define uart2_send_static_text(msg) do { e_send_uart2_char(msg,sizeof(msg)-1); while(e_uart2_sending()); } while(0)
#define uart2_send_text(msg) do { e_send_uart2_char(msg,strlen(msg)); while(e_uart2_sending()); } while(0)

//#define SOUNDT 1500 //sound threshold
#define PI 3.14159265358979
#define FFT_Prescaler 128 //Multiply microphone data with this value after subtracting the mean -> amplitude of 256 will be converted to 32768 (utilizing full range of 16bit int)
#define INVALID_PHASE -100.0 //Value for invalid phase, e.g. when calculating phase at frequency with magnitude close to 0

//global variables to allow communication between functions
static int u_1k[2];

int mic0_seq[MIC_SAMP_NB]; //Storage space for sequence	captured by microphone 0	
int mic1_seq[MIC_SAMP_NB]; //Storage space for sequence	captured by microphone 1
int mic2_seq[MIC_SAMP_NB]; //Storage space for sequence	captured by microphone 2

//fractional mic0_fft_real[FFT_BLOCK_LENGTH]  __attribute__ ((space(auto_psv), aligned (FFT_BLOCK_LENGTH*2)));
//fractional mic0_fft_real[FFT_BLOCK_LENGTH]  __attribute__ ((space(xmemory), aligned (FFT_BLOCK_LENGTH*2)));
//fractional mic0_fft_real[FFT_BLOCK_LENGTH]  __attribute__ ((space(ymemory), aligned (FFT_BLOCK_LENGTH*2)));

//Odometry parameters
const int NStepsPerRobRot = 1293; // Calculated: 1293; // number of motor steps needed to make a one full rotation (with equal wheel speeds in opposite directions)
const int NStepsPerMeter = 7764;  // Calculated: 7764;  // number of motor steps needed to travel 1m (with equal wheel speeds)

// Calibration or odometry-based measurements
int CalMotorSpeed = 500; 	//[0..1000] Motor speed for both wheels during calibration
int CalComplete[] = {0,0,0,0,0}; //Array of flags. Flag i is set after calibration of signal i is complete
int CalNStops = 0; 			// Number of stops per calibration step, in order to take measurements while not moving. If CalNStops = 0, then calibrate during driving
int CalNMeasPerStop = 10; 	//Number of measurements per stop (ignored if calibrating during driving)

//Signal to detect
unsigned int par_signal_id[NSIG]; //[0..4] id of the signal of interest in par_freqToDetect array.
//Signal to play
unsigned int par_playNote_id = 2; //[0..4] id of the signal to play

int tri_step_ctr = 0;
int tri_status = 0; //0 = waiting for signal
					//1 = playing note

// Sound synchronisation protocol. Used when a robot has to play sound and listen to another robot who is not playing constantly (selector4)
long soundDelay; // [microsec] last random delay
float par_soundDelayMax = 0.02; //[s] maximal length of a random delay, which is used to synchronize the robots

unsigned int iter_counter; //debug: control iteration counter
unsigned int meas_counter[NSIG]; //debug: number of successful measurements (i.e. signal of interest detected in FFT)

unsigned int par_AgendaDelayEKF;
// Flags for variable TsEKF
int flag_didFFT = 0;
int flag_didMeasUp = 0;
int flag_didCommUp = 0;
int flag_didSendComm = 0;
int flag_didSoundDelay = 0;

// Setpoint
float SP_d = 0.15; 		//[m] Controller input 1: Distance to the setpoint 
float SP_alpha = 0.0; 	//[rad] Controller input 2: Relative angle to the setpoint  (e.g. 0 = in front of robot, +pi/2 = to the left of the robot, -pi/2 = to the right of the robot, etc.)
float SP_User_x = -0.20;  //[m]  x-coordinate of the user-defined setpoint
float SP_User_y = -0.15;	//[m]  y-coordinate of the user-defined setpoint


// Formation

int RobForm_id = 8;	// id of the robot in the formation -> determines the setpoint within the formation
int LeadForm_id = 8;  
int FolForm_id = 5;
// 10 formation positions (NumPad with x-axis pointing up)
const int par_NFormPos = 10; // Number of formation positions
const float FormPos_x[] = {-0.4, -0.2, -0.2, -0.2, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2 };
const float FormPos_y[] = {0.0, 0.2, 0.0, -0.2, 0.2, 0.0, -0.2, 0.2, 0.0, -0.2};

//12 formation positions, relative coordinates from the leader 
//const float par_RForm = 0.2; // [m] radius of the formation
//const float FormSP_d[] = {par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm, par_RForm}; //equal distance from leader
//const float FormSP_alpha[] = {0.0, -PI*1.0/6.0, -PI*2.0/6.0, -PI*3.0/6.0, -PI*4.0/6.0, -PI*5.0/6.0, -PI, PI*5.0/6.0, PI*4.0/6.0, PI*3.0/6.0, PI*2.0/6.0, PI*1.0/6.0}; //30 degree steps, starting at 0.0 (12 o'clock) and proceeding clockwise



void run_controldrive(void) {

// init
	int iStop = 1;
	int iRunningEKF = 0;
	static char c1,c2;
	static int	i,mic_id,speedr,speedl,positionr,positionl,LED_nbr,action;
	static char first=0;
	int max_state_id; //delete me! For debug in 'K' command
	//float CalDist; //distances from current microphone to the source 
	//float CalMeas; //measurements of current microphone
	float CalWaypoint[] = {0.10, 0.20};
	float absFFTout; 	   	   //One value of abs(FFT) for each mic
	float FFTreal,FFTimag;	   //Real and imaginary parts of one FFT value
	float d_meas,alpha_meas;   // for "V"-command
	i=0;
	do 
	{
		par_signal_id[i] = i;
		i++;
	} while (i<NSIG);




	//clock_t tic = 1;
	//clock_t toc = 2;

	//Most of initialization is done in main()
	init_EKF_Source();

	//e_ad_scan_on();
	//e_init_port();    // configure port pins
	//e_start_agendas_processing();
	//e_init_motors();
	//e_init_uart1();   // initialize UART to 115200 Kbaud
	//e_init_micro(); 	

	int use_bt=1;
	selector = getselector(); //SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
	
	if(selector==4) 
	{
		par_AgendaDelayEKF = 1220; //1720;  //Sampling time of Extended Kalman Filter. Units: 0.1ms, i.e. 10'000 = 1s
	} else
	{
		par_AgendaDelayEKF = 620; //620; //220; //1000; //Delay between updates of Extended Kalman Filter and Controller. Units: 0.1ms, i.e. 10'000 = 1s
	}


	if(RCONbits.POR) {	// reset if power on (some problem for few robots)
		RCONbits.POR=0;
		RESET();
	}

	if(use_bt) {
	uart1_send_static_text("\f\a"
			"Greetings, Keeper. \r\n"
			"Using UART1 (Bluetooth). \r\n"
			"WELCOME to the SerCom protocol on e-Puck\r\n"
			"the EPFL education robot type \"H\" for help\r\n");
	} else {
	uart2_send_static_text("\f\a"
			"Greetings, Keeper. \r\n"
			"Using UART2 (cable, NOT SUPPORTED). \r\n"
			"WELCOME to the SerCom protocol on e-Puck\r\n"
			"the EPFL education robot type \"H\" for help\r\n");
	}


	while(1) {
		
		while (e_getchar_uart1(&c)==0)
		{}
		// *****binary mode (big endian)***** //ATr: Not used in Robot Swarm project. See ascii mode below
		if (c<0) { 
			i=0;
			do 
			{	
				switch(-c) 
				{ 
				case 'L': // set LED
					if(use_bt) {
						while (e_getchar_uart1(&c1)==0);
						while (e_getchar_uart1(&c2)==0);
					} else {
						while (e_getchar_uart2(&c1)==0);
						while (e_getchar_uart2(&c2)==0);
					}
					switch(c1) {
						case 8:
							if(use_bt) {
								e_set_body_led(c2);
							}
							break;
						case 9:
							if(use_bt) {
								e_set_front_led(c2);
							}
							break;
						default:
							e_set_led(c1,c2);
							break;
					}
					break;
				default: // silently ignored

					break;
				}
				if(use_bt) {
					while (e_getchar_uart1(&c)==0); // get next command
				} else {
					while (e_getchar_uart2(&c)==0); // get next command
				}
			} while(c);
			if (i!=0){			
				if(use_bt) {
					e_send_uart1_char(buffer,i); // send answer
					while(e_uart1_sending());
				} else {
					e_send_uart2_char(buffer,i); // send answer
					while(e_uart2_sending());
				}			
			}

		// **** ascii mode ****
		} else if (c>0) { // ascii mode
			if(use_bt) {
				while (c=='\n' || c=='\r') e_getchar_uart1(&c);
			} else {
				while (c=='\n' || c=='\r') e_getchar_uart2(&c);
			}
			buffer[0]=c;
			i = 1;
			if(use_bt) {
				do if (e_getchar_uart1(&c)) buffer[i++]=c;
				while (c!='\n' && c!='\r');				
			} else {
				do if (e_getchar_uart2(&c)) buffer[i++]=c;
				while (c!='\n' && c!='\r');			
			}
			buffer[i++]='\0';

			if((buffer[0] != 'B') && (buffer[0] != 'b')) {
				buffer[0]=toupper(buffer[0]); // we accept lowercase letters except for 'b'
			}
			switch (buffer[0]) {

			case 'A':	// Select signal to track
				sscanf(buffer, "A,%d,%d,%d\r", &par_signal_id[0], &par_signal_id[1], &par_playNote_id);
				if ((par_signal_id[0] > 4) | (par_signal_id[1] > 4)) //par_signal_id must be 0..4
				{
					par_signal_id[0] = 0;
					par_signal_id[1] = 0;
				}
				sprintf(buffer,"a, tracking [%f, %f] Hz, playing %f Hz\r\n", par_freqToDetect[par_signal_id[0]], par_freqToDetect[par_signal_id[1]],par_freqToDetect[par_playNote_id]);
				uart1_send_text(buffer);			
				init_EKF_Source(); //reset EKF
				break;
			case 'b':	// battery is ok?
				sprintf(buffer,"b,%d\r\n", BATT_LOW);	// BATT_LOW=1 => battery ok, BATT_LOW=0 => battery<3.4V
				if(use_bt) {
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}				
				break;
			case 'B': // set body led
				sscanf(buffer,"B,%d\r",&action);
				if(use_bt) {
					e_set_body_led(action);
					uart1_send_static_text("b\r\n");
				} else {
					uart2_send_static_text("b\r\n");
				}
				break;
			case 'C': // Calibrate distance measurement with microphones
				//int mic_id = 0;
				sscanf(buffer, "C,%d,%f,%f,%d,%d\r", &action,&CalWaypoint[0],&CalWaypoint[1],&CalNStops,&CalNMeasPerStop);
				//uart1_send_static_text("c\r\n");
				if (CalComplete[par_signal_id[0]]==0)
				{
					sprintf(buffer,"c,%d,%f,%f\r\n",action,CalWaypoint[0],CalWaypoint[1]);	uart1_send_text(buffer);
					uart1_send_static_text("CalData = [\r\n");
				} else
				{
					uart1_send_static_text("ValData = [\r\n");
				}
				i = 0;
				if (action == 1)				
				{
					i = i + CalibrateLineDrive(CalWaypoint[0], CalWaypoint[1]);	
					i = i + CalibrateLineDrive(CalWaypoint[1], CalWaypoint[0]);
					i = i + CalibrateLineDrive(CalWaypoint[0], CalWaypoint[1]);
				} else if(action == 2)
				{
					i = i + CalibrateCircleDrive(CalWaypoint[0], CalWaypoint[0]-CalWaypoint[1]);
					i = i + CalibrateCircleDrive(CalWaypoint[0], CalWaypoint[0]-CalWaypoint[1]);
					i = i + CalibrateCircleDrive(CalWaypoint[0], CalWaypoint[0]-CalWaypoint[1]);	
				} else if(action == 3)
				{
					i = i + CalibrateRotating(CalWaypoint[0]);	
					i = i + CalibrateRotating(CalWaypoint[0]);
					i = i + CalibrateRotating(CalWaypoint[0]);	
				} else if(action == 4)
				{
					i = i + CalibrateRotating(CalWaypoint[0]);
					i = i + CalibrateLineDrive(CalWaypoint[0], CalWaypoint[1]);
					i = i + CalibrateRotating(CalWaypoint[1]);			
				}			
/*
				//Step1 : Rotate around own axis at waypoint0
				i = i + CalibrateRotating(CalWaypoint[0]);
				//Step2 : drive to waypoint1
				i = i + CalibrateLineDrive(CalWaypoint[0], CalWaypoint[1]);
				//Step3 : Rotate around own axis at waypoint1
				i = i + CalibrateRotating(CalWaypoint[1]);
				//Step4 : drive to waypoint2
				i = i + CalibrateLineDrive(CalWaypoint[1], CalWaypoint[2]);	
				//Step5 : drive back to waypoint0
				i = i + CalibrateLineDrive(CalWaypoint[2], CalWaypoint[0]);		
*/		
				sprintf(buffer,"]; i = %d;\r\n",i); uart1_send_text(buffer);
				if (CalComplete[par_signal_id[0]]==0)
				{
					sprintf(buffer,"b1 = [%f, %f, %f]; b0 = [%f, %f, %f];\r\n",CalParamMic0_x1[par_signal_id[0]],CalParamMic1_x1[par_signal_id[0]],CalParamMic2_x1[par_signal_id[0]],CalParamMic0_x0[par_signal_id[0]],CalParamMic1_x0[par_signal_id[0]],CalParamMic2_x0[par_signal_id[0]]); uart1_send_text(buffer);
					sprintf(buffer,"Signal %f Hz calibrated.\r\n",par_freqToDetect[par_signal_id[0]]); uart1_send_text(buffer);
					init_EKF_Source(); //reinitialize matrices for EKF_Source
					CalComplete[par_signal_id[0]] = 1;
				}
/*
				i =0;
				do
				{
					sprintf(buffer,"b1 = [%f, %f, %f]; b0 = [%f, %f, %f];\r\n",CalParamMic0_x1[i],CalParamMic1_x1[i],CalParamMic2_x1[i],CalParamMic0_x0[i],CalParamMic1_x0[i],CalParamMic2_x0[i]); uart1_send_text(buffer);
					i++;
				} while (i<5);
*/
/*
				sprintf(buffer,"Mic 0:\r\n",i); uart1_send_text(buffer);
				sprintf(buffer,"b1 = %f; b2 = %f;\r\n",CalParamMic0_x1[par_signal_id[0]],CalParamMic0_x0[par_signal_id[0]]); uart1_send_text(buffer);
				sprintf(buffer,"P(1,1) = %f; P(2,2) = %f;\r\n",CalParamMic0_P[3],CalParamMic0_P[0]); uart1_send_text(buffer);
				sprintf(buffer,"Mic 1:\r\n",i); uart1_send_text(buffer);
				sprintf(buffer,"b1 = %f; b2 = %f;\r\n",CalParamMic1_x1[par_signal_id[0]],CalParamMic1_x0[par_signal_id[0]]); uart1_send_text(buffer);
				sprintf(buffer,"P(1,1) = %f; P(2,2) = %f;\r\n",CalParamMic1_P[3],CalParamMic1_P[0]); uart1_send_text(buffer);
				sprintf(buffer,"Mic 2:\r\n",i); uart1_send_text(buffer);
				sprintf(buffer,"b1 = %f; b2 = %f;\r\n",CalParamMic2_x1[par_signal_id[0]],CalParamMic2_x0[par_signal_id[0]]); uart1_send_text(buffer);
				sprintf(buffer,"P(1,1) = %f; P(2,2) = %f;\r\n",CalParamMic2_P[3],CalParamMic2_P[0]); uart1_send_text(buffer);
*/

				break;
			case 'D': // set motor speed
				sscanf(buffer, "D,%d,%d\r", &u_1k[0], &u_1k[1]);
				e_set_speed_left(u_1k[0]);
				e_set_speed_right(u_1k[1]);
				if(use_bt) {
					uart1_send_static_text("d\r\n");
				} else {
					uart2_send_static_text("d\r\n");
				}
				iStop = 0;
				break;
			case 'E': // read motor speed
				sprintf(buffer,"e,%d,%d\r\n",speedl,speedr);
				if(use_bt) {
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}
				break;

			case 'F': //Capture and print microphone arrays //and perform frequency analysis
					e_ad_scan_on();
					do
					{
						NOP();
					} while (e_last_mic_scan_id==0); //first wait for the first adc interrupt, which changes e_last_mic_scan_id and sets e_ad_is_array_filled=0
					do
					{
						NOP();
					} while (!e_ad_is_array_filled()); //then wait for e_ad_is_array_filled=1
					e_ad_scan_off();
					e_get_micro_last_values (0, &mic0_seq ,MIC_SAMP_NB);
					e_get_micro_last_values (1, &mic1_seq ,MIC_SAMP_NB);
					e_get_micro_last_values (2, &mic2_seq ,MIC_SAMP_NB);

					sprintf(buffer,"MicData = [\r\n");
					uart1_send_text(buffer);
					i = 0;
					do
					{			
						sprintf(buffer,"%d %d %d\r\n",mic0_seq[i],mic1_seq[i],mic2_seq[i]);
						//sprintf(buffer,"%d\r\n",mic0_seq[i]);
						uart1_send_text(buffer);
					i++;
					} while (i<MIC_SAMP_NB);
					sprintf(buffer,"]';return\r\n");
					uart1_send_text(buffer);	

				break; 

			case 'G': // Manually reset the state estimate
				sscanf(buffer,"G,%f,%f\r",&x_m[0],&x_m[1]);
				sprintf(buffer,"g,%f,%f\r\n",x_m[0],x_m[1]); uart1_send_text(buffer);
				//x_m[0] = SP_d;
				//x_m[1] = SP_alpha;
				//x_m[3] = x_m[0] + SP_d*cos(x_m[2]+SP_alpha);
				//x_m[4] = x_m[1] + SP_d*sin(x_m[2]+SP_alpha);
				// SP_User_x = x_m[3];
				// SP_User_y = x_m[4];
				//uart1_send_static_text("g\r\n");
				// Covariance matrix of the initial estimate 
				//(Reset to diagonal with small uncertainty)
				if (selector ==4)
				{
					e_init_sound();
				}
				if (iRunningEKF == 0)
				{
					e_activate_agenda(CtrlSetSpeed,par_AgendaDelayEKF);
					iRunningEKF = 1;
				}
				//e_activate_agenda(BroadcastSetpoint, 10000);
				//sprintf(buffer,"d_est = %f, alpha_est = %f\r\n",SP_d,SP_alpha); uart1_send_text(buffer);
				//sprintf(buffer,"P = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
				/*sprintf(buffer,"g,%f,%f, result:%d,%d\r\n",SrcPosMeas_d,SrcPosMeas_alpha,u_1k[0],u_1k[1]);
				if(use_bt) {						
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}
				*/
				break;
			case 'H': // ask for help
				if(use_bt) {
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
				} else {
					uart2_send_static_text("\n");
					uart2_send_static_text("\"A\"         Accelerometer\r\n");
					uart2_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
					uart2_send_static_text("\"b\"		  Battery ok?\r\n");
					uart2_send_static_text("\"C\"         Selector position\r\n");
					uart2_send_static_text("\"D,#,#\"     Set motor speed left,right\r\n");
					uart2_send_static_text("\"E\"         Get motor speed left,right\r\n");
					uart2_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
					uart2_send_static_text("\"H\"	      Help\r\n");
					uart2_send_static_text("\"L,#,#\"     Led number,0=off 1=on 2=inverse\r\n");
					uart2_send_static_text("\"P,#,#\"     Set motor position left,right\r\n");
					uart2_send_static_text("\"Q\"         Get motor position left,right\r\n");
					uart2_send_static_text("\"R\"         Reset e-puck\r\n");
					uart2_send_static_text("\"S\"         Stop e-puck and turn off leds\r\n");
					//uart2_send_static_text("\"T,#\"       Play sound 1-5 else stop sound\r\n");
					uart2_send_static_text("\"U\"         Get microphone amplitude\r\n");
					uart2_send_static_text("\"V\"         Version of SerCom\r\n");
				}
				break;

			case 'I': //Update relative position of the source with camera measurement and (optional) with measurements from other robots
				sscanf(buffer,"I,%f,%f,%f,%f,%d,%d,%d,%d,%f,%d,%f,%d,%f\r",&SrcPosMeasCam_d[0],&SrcPosMeasCam_alpha[0],&SrcPosMeasCam_d[1],&SrcPosMeasCam_alpha[1],&RobForm_id,&LeadForm_id,&FolForm_id, &SrcPosMeasComm_new, &SrcPosMeasComm_x[0], &SrcPosMeasComm_y[0], &SrcPosMeasComm_x[1], &SrcPosMeasComm_y[1], &SrcPosMeasComm_x[2], &SrcPosMeasComm_y[2]);
				if (RobForm_id<0 | RobForm_id>par_NFormPos)
				{
					RobForm_id = 0;
					uart1_send_static_text("i,wrong RobForm_id,reset to 0\r\n");
				}
				alpha_meas = GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]]);
				sprintf(buffer,"i,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3],EKF_P[0],EKF_P[5],alpha_meas); uart1_send_text(buffer);
				//sprintf(buffer,"i,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3],EKF_P[0],EKF_P[5]); uart1_send_text(buffer);
				flag_didSendComm++;
				//sprintf(buffer,"i,%f,%f,%f,%f,%d\r\n",SrcPosMeasCam_d[0],SrcPosMeasCam_alpha[0],SrcPosMeasCam_d[1],SrcPosMeasCam_alpha[1],iter_counter); uart1_send_text(buffer);
				// Measurement (d,alpha)=(0,0) is not a measurement
				if ((SrcPosMeasCam_d[0]!=0.0) | (SrcPosMeasCam_alpha[0]!=0.0))
				{
					SrcPosMeasCam_new[0] = 1;
				} else {
					SrcPosMeasCam_new[0] = 0;
				}

				if ((SrcPosMeasCam_d[1]!=0.0) | (SrcPosMeasCam_alpha[1]!=0.0))
				{
					SrcPosMeasCam_new[1] = 1;
				} else {
					SrcPosMeasCam_new[1] = 0;
				}

				if (iRunningEKF == 0)
				{
					e_activate_agenda(CtrlSetSpeed,par_AgendaDelayEKF);
					iRunningEKF = 1;
				}
				break;
			case 'J':  //print states and setpoint
				sprintf(buffer,"Src0 = [%5.3f,%5.3f]; Src1 = [%5.3f,%5.3f]; SP = [%5.3f,%5.3f];\r\n",x_m[0],x_m[1],x_m[2],x_m[3],SP_d,SP_alpha); uart1_send_text(buffer);
				break;
			case 'K':  //Initialize kalman filter to the most likely sector and turn on rejection of unlikely measurements
				//max_state_id = find_max_id(&x_m[0],4);
				//sprintf(buffer,"States = [%5.3f,%5.3f,%5.3f,%5.3f]; max_id = %d;\r\n",x_m[0],x_m[1],x_m[2],x_m[3],max_state_id); uart1_send_text(buffer);
				FindInitSector();
				par_RejectAngles = 1;
				break;
			case 'L': // set led
				sscanf(buffer,"L,%d,%d\r",&LED_nbr,&action);
				e_set_led(LED_nbr,action);
				//	uart1_send_static_text("l\r\n");
				break;

			case 'M': //matrix operations
			// WARNING!!! Sending long messages changes selector value, because there is not enough space in the buffer for the whole message	

				sprintf(buffer,"P = [%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f;",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7]);
				uart1_send_text(buffer);
				sprintf(buffer,"%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f;]\r\n",EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]);
				uart1_send_text(buffer);		
				
				sprintf(buffer,"TMP1 = [%f,%f,%f,%f; %f,%f,%f,%f;",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);
				uart1_send_text(buffer);
			    sprintf(buffer," %f,%f,%f,%f; %f,%f,%f,%f;]\r\n",EKF_TEMP1[8],EKF_TEMP1[9],EKF_TEMP1[10],EKF_TEMP1[11],EKF_TEMP1[12],EKF_TEMP1[13],EKF_TEMP1[14],EKF_TEMP1[15]);
				uart1_send_text(buffer);

				Tr_MatrixMultiply (2,2,2, &EKF_TEMP1[11], &EKF_P[8], EKF_TEMP1);				

				sprintf(buffer,"TMP1[11] = (2x2) P[8]*TMP1[0] = [%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f;",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);
				uart1_send_text(buffer);
			    sprintf(buffer,"%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f;]\r\n",EKF_TEMP1[8],EKF_TEMP1[9],EKF_TEMP1[10],EKF_TEMP1[11],EKF_TEMP1[12],EKF_TEMP1[13],EKF_TEMP1[14],EKF_TEMP1[15]);
				uart1_send_text(buffer);

/*
				Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_P, EKF_TEMP1); // Temp2 = P*Temp1
				sprintf(buffer,"TMP2 = P*TMP1 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8],EKF_TEMP2[9],EKF_TEMP2[10],EKF_TEMP2[11],EKF_TEMP2[12],EKF_TEMP2[13],EKF_TEMP2[14],EKF_TEMP2[15]);
				if(use_bt) {						
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}

				Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P,  EKF_P, EKF_TEMP1); // P = P - Temp1
				sprintf(buffer,"P = P - TMP1 = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f;]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]);
				if(use_bt) {						
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}

				Tr_MatrixSubtractFromId(3,EKF_TEMP2, EKF_TEMP2);
				sprintf(buffer,"TMP2 = I - TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);
				if(use_bt) {						
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}

				Tr_MatrixInverse3x3(EKF_TEMP1, EKF_TEMP2);
				sprintf(buffer,"TMP1 = Inv(EKF_TEMP2) [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);
				if(use_bt) {						
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}
*/
				break;
			case 'N':
				sscanf(buffer,"N,%d\r",&action);
				
				if (action == 1)
				{
					e_activate_agenda(EKF_Source,par_AgendaDelayEKF);
				} else if (action == 2) {
					e_activate_agenda(BroadcastSetpoint, 10000);
				} else if (action == 3) {
					e_activate_agenda(BroadcastSrcPosEst, 10000);				
				} else {				
					e_destroy_agenda(BroadcastSetpoint);
					e_destroy_agenda(BroadcastSrcPosEst);
				}
				
				//EKF_Source();				
				break;
			case 'O':
				sscanf(buffer,"O,%f,%f,%f,%f,%f,%f\r",&SrcPosMeasComm_x[0],&SrcPosMeasComm_y[0],&SrcPosMeasComm_x[1],&SrcPosMeasComm_y[1],&SrcPosMeasComm_x[2],&SrcPosMeasComm_y[2]);
				SrcPosMeasComm_new = 1;
				sprintf(buffer,"o,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",SrcPosMeasComm_x[0],SrcPosMeasComm_y[0],SrcPosMeasComm_x[1],SrcPosMeasComm_y[1],SrcPosMeasComm_x[2],SrcPosMeasComm_y[2]); uart1_send_text(buffer);
				break;	
			case 'P': // set motor position
				sscanf(buffer,"P,%d,%d\r",&positionl,&positionr);
				e_set_steps_left(positionl);
				e_set_steps_right(positionr);
				if(use_bt) {
					uart1_send_static_text("p\r\n");
				} else {
					uart2_send_static_text("p\r\n");
				}
				break;
			case 'Q': // read motor position
				sprintf(buffer,"q,%d,%d\r\n",e_get_steps_left(),e_get_steps_right());
				if(use_bt) {
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}
				break;
			case 'R': // reset
				if(use_bt) {
					uart1_send_static_text("r\r\n");
				} else {
					uart2_send_static_text("r\r\n");
				}
				RESET();
				break;
			case 'S': // stop
				e_set_speed_left(0);
				e_set_speed_right(0);
				u_1k[0]=0;
				u_1k[1]=0;
				iStop = 1;
				if (iRunningEKF == 1)
				{
					e_destroy_agenda(CtrlSetSpeed);
					iRunningEKF = 0;
				}

				if (selector ==4)
				{
					e_close_sound();
				}
				e_destroy_agenda(BroadcastSetpoint);
				if(use_bt) {
					uart1_send_static_text("s\r\n");
				} else {
					uart2_send_static_text("s\r\n");
				}
				break;
			case 'T': // Play sound
				if(use_bt) {
					sscanf(buffer,"T,%d",&action);
					if(first==0){
						e_init_sound();
						first=1;
					}
					e_destroy_agenda(PlaySignal0);
					e_destroy_agenda(PlaySignal0_short);
					e_destroy_agenda(PlaySignal1);
					e_destroy_agenda(PlaySignal2);
					e_destroy_agenda(PlaySignal3);
					e_destroy_agenda(PlaySignal4);
					switch(action)
					{
						case 1: e_play_sound(0,720);break;
						case 2: 
							e_activate_agenda(PlaySignal0_short,10000);
							break;	
						case 3: 
							e_activate_agenda(PlaySignal0,5000);
							break;					
						case 4: 
							e_activate_agenda(PlaySignal1,5000);
							break;
						case 5:
							e_activate_agenda(PlaySignal2,5000);
							break;	
						case 6:
							e_activate_agenda(PlaySignal3,5000);
							break;	
						case 7:
							e_activate_agenda(PlaySignal4,5000);
							break;	
						case 8:
							for (i=0;i<10;i++)
							{
								e_play_sound(0, 3600);
								while (e_dci_unavailable);
								//e_play_sound(3600, 3600);
								//while (e_dci_unavailable);
							}
							break;								
						default:
							e_close_sound();
							first=0;
							break;
					}				
					//sprintf(buffer,"t,%d\r\n",action);	
					uart1_send_static_text("t\r\n");
				} else {
					uart2_send_static_text("t\r\n");
				}
				break;

			case 'U':	
				CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]); //7th frequency is 902Hz 
				//sprintf(buffer,"%f %f %f;\r\n",1.0/(CalParamMic0_x1*MicFreqMeasAmp[0] + CalParamMic0_x0),1.0/(CalParamMic1_x1*MicFreqMeasAmp[1] + CalParamMic1_x0),1.0/(CalParamMic2_x1*MicFreqMeasAmp[2] + CalParamMic2_x0));
				sprintf(buffer,"u0,%f,%f,%f\r\n",MicFreqMeasAmp[0],MicFreqMeasAmp[1],MicFreqMeasAmp[2]);
				uart1_send_text(buffer);
				sprintf(buffer,"u1,%f,%f,%f\r\n",MicFreqMeasAmp[3],MicFreqMeasAmp[4],MicFreqMeasAmp[5]);
				//sprintf(buffer,"u,%f,%f,%f\r\n",MicFreqMeasPhase[0],MicFreqMeasPhase[1],MicFreqMeasPhase[2]);
				uart1_send_text(buffer);
/*

				sprintf(buffer,"1>2 = %d\r\n",1>2); uart1_send_text(buffer); 
				sprintf(buffer,"1<2 = %d\r\n",1<2); uart1_send_text(buffer); 
				sprintf(buffer,"atan2(0,0)<0 = %d\r\n",atan2(0.0,0.0)<0); uart1_send_text(buffer); 
				sprintf(buffer,"atan2(0,0)==0 = %d\r\n",atan2(0.0,0.0)==0); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0,0)>0 = %d\r\n",atan2(0.0,0.0)>0); uart1_send_text(buffer);
 				sprintf(buffer," atan2(0,0) = %d\r\n",	   atan2(0.0,0.0)); uart1_send_text(buffer);
 				sprintf(buffer,"(hex)atan2(0,0) = %x\r\n", atan2(0.0,0.0)); uart1_send_text(buffer);
				sprintf(buffer,"(int)atan2(0,0) = %d\r\n",(int)atan2(0.0,0.0)); uart1_send_text(buffer);
				sprintf(buffer,"(int)atan2(0,0)==-1 = %d\r\n",((int)atan2(0.0,0.0))==-1); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0,0)==-1 = %d\r\n",(atan2(0.0,0.0))==-1); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0,0)==0x7FFF = %d\r\n",(atan2(0.0,0.0))==0x7FFF); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0.1,0.1) = %f\r\n",atan2(0.1,0.1)); uart1_send_text(buffer); 
				sprintf(buffer,"atan2(0.01,0.01) = %f\r\n",atan2(0.1,0.1)); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0.001,0.001) = %f\r\n",atan2(0.1,0.1)); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0.00001,0.00001) = %f\r\n",atan2(0.1,0.1)); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0.000001,0.000001) = %f\r\n",atan2(0.1,0.1)); uart1_send_text(buffer);
 				sprintf(buffer," atan2(0,0) = %f\r\n",atan2(0.0,0.0)); uart1_send_text(buffer);
 				sprintf(buffer," isNaN(atan2(0,0)) = %d\r\n",isNaN(atan2(0.0,0.0))); uart1_send_text(buffer);
				sprintf(buffer,"atan2(0,0)<0 = %d\r\n",atan2(0.0,0.0)<0); uart1_send_text(buffer);
				sprintf(buffer,"!(atan2(0,0)<0) = %d\r\n",!(atan2(0.0,0.0)<0)); uart1_send_text(buffer);
*/
				//sprintf(buffer,"%f\r\n",GetPhaseShiftAngle(MicFreqMeasPhase)); uart1_send_text(buffer); 
				break;

			case 'V':
				d_meas = GetDistMeasurement(par_signal_id[0],&MicFreqMeasAmp[0],x_m[1]);
				alpha_meas = GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]]);
			    sprintf(buffer,"v,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3],d_meas,alpha_meas,MicFreqMeasAmp[0],MicFreqMeasAmp[1],MicFreqMeasAmp[2],MicFreqMeasPhase[0],MicFreqMeasPhase[1],MicFreqMeasPhase[2]);
				//sprintf(buffer,"v,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3]);
				uart1_send_text(buffer);
				flag_didSendComm++;				
				break;

/*
			case 'U':
				sprintf(buffer,"u,%d,%d,%d\r\n",e_get_micro_volume(0),e_get_micro_volume(1),e_get_micro_volume(2));
				uart1_send_text(buffer);
				break;
*/
			/*
			case 'V': // get version information
				if(use_bt) {
					uart1_send_static_text("v,Hacked! Based on Version 1.2.2 August 2008 GCtronic\r\n");
				} else {
					uart2_send_static_text("v,Hacked! Based on Version 1.2.2 August 2008 GCtronic\r\n");
				}
				sprintf(buffer,"HW version: %X\r\n",HWversion);
				if(use_bt) {
					uart1_send_text(buffer);
				} else {
					uart2_send_text(buffer);
				}
				break;
			*/
			case 'W': //Display an 8-bit number with LEDs
				sscanf(buffer,"W,%d,%d",&LED_nbr,&action);
				uart1_send_static_text("w\r\n");
				//led_show_8bit(LED_nbr);
				i=1;

				do
				{
				if (action ==2)
				{
					CaptureFFT(1, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
				} else {
					CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
				}
				EKF_Source();
				if (action == 1)
				{
					Setpoint_Form();
					PID_Drive(SP_d,SP_alpha,u_1k);
				}
				//sprintf(buffer,"%f,%f\r\n",(float)i*PI/16.0 - 3.0*PI, norm_angle((float)i*PI/16.0 - 3.0*PI)); uart1_send_text(buffer);
				led_show_8bit(i);
				i++;				
				} while (i<=LED_nbr);
				break;				
			case 'X': //Set coordinates of a setpoint in "triangle" formation. Origin is on the leader, with x-axis pointing in driving direction.
				sscanf(buffer,"X,%f,%f\r",&SP_User_x,&SP_User_y);
				sprintf(buffer,"x,%f,%f\r\n",SP_User_x,SP_User_y); uart1_send_text(buffer);
				if (iRunningEKF == 0)
				{
					e_activate_agenda(CtrlSetSpeed,par_AgendaDelayEKF);
					iRunningEKF = 1;
				}
				break;

			case 'Y': //Update robots position with camera measurement
				sscanf(buffer,"Y,%f,%f\r",&RobPosMeas_x,&RobPosMeas_y);
				sprintf(buffer,"y,%f,%f\r\n",RobPosMeas_x,RobPosMeas_y); uart1_send_text(buffer);
				RobPosMeas_new = 1;
				break;

			//case '0': //turn ObstacleAvoidance off

			//	break;

			default:
				if(use_bt) {
					uart1_send_static_text("z,Command not found\r\n");
			  	} else {
					uart2_send_static_text("z,Command not found\r\n");
				}
				break;
			}
		}		

	}
}

void Setpoint_Source (void)
{
	const float d_to_source = 0.13; //0.0; //[m] distance to the source, that the robot is trying to keep.
	SP_d = x_m[0] - d_to_source;
	SP_alpha = x_m[1];
}

void Setpoint_User (void)
{
	// Do nothing, SP_d and SP_alpha have already been set by 'G'-command
	SP_d = x_m[0];
	SP_alpha = x_m[1];
}

void Setpoint_Triangle(void)
{
/*
% Assumes that you get SrcPosEst of the Leader and Follower, which both point
% in the same direction, i.e. vector from Follower to Leader defines the
% heading of the formation and Leader's coordinate system

% Current triangle is defined through two side lengths (SrcPosEst(1).d, SrcPosEst(2).d)
% and an angle between them (difference between SrcPosEst(1).alpha and
% SrcPosEst(2).alpha)

% Also see MATLAB simulation

dL = x_m[0];
alphaL = x_m[1];
dF = x_m[2];
alphaF = x_m[3];
*/

// Vector from Leader to Robot's desired formation position, in Leader's coordinate system
//const float Lx_targ = -0.15; 
//const float Ly_targ = -0.2; 

float Lx_targ = SP_User_x;
float Ly_targ = SP_User_y;

/* //Calculating gamma in polar coordinates -> problem with asin. Use carthesian coordinates and atan2 instead
// Calculate distance between Leader and Follower
float rLF = sqrt(x_m[0]*x_m[0] + x_m[2]*x_m[2] - 2.0*x_m[0]*x_m[2]*cos(x_m[1] - x_m[3]));
// Calculate the angle between this robot's coordinate system and Leader's coordinate system
float gamma = x_m[1] + asin(sin(x_m[1] - x_m[3])*x_m[2]/rLF);
*/

// Calculate the angle between this robot's coordinate system and Leader's coordinate system
float gamma = atan2(x_m[0]*sin(x_m[1]) - x_m[2]*sin(x_m[3]), x_m[0]*cos(x_m[1]) - x_m[2]*cos(x_m[3]));

// Vector from Robot to Robot's desired formation position, in Robot's
// coordinate system
float Rx_targ = x_m[0]*cos(x_m[1]) + cos(gamma)*Lx_targ - sin(gamma)*Ly_targ;
float Ry_targ = x_m[0]*sin(x_m[1]) + sin(gamma)*Lx_targ + cos(gamma)*Ly_targ;

//Transform to polar coordinates
SP_d = sqrt(Rx_targ*Rx_targ + Ry_targ*Ry_targ);
SP_alpha = atan2(Ry_targ,Rx_targ);
}

void Setpoint_Form (void)
{
float Fx_targ,Fy_targ,gamma,Rx_targ,Ry_targ;
// Formation position in Leader's coordinate system, depending on this robot's RobForm_id
//Fx_targ = FormPos_x[RobForm_id] - FormPos_x[FolForm_id];
//Fy_targ = FormPos_y[RobForm_id]- FormPos_y[FolForm_id];
Fx_targ = FormPos_x[RobForm_id] - FormPos_x[LeadForm_id];
Fy_targ = FormPos_y[RobForm_id]- FormPos_y[LeadForm_id];
if (RobForm_id == FolForm_id)
{
	SP_d = x_m[0] - sqrt((FormPos_x[LeadForm_id] - FormPos_x[FolForm_id])*(FormPos_x[LeadForm_id] - FormPos_x[FolForm_id]) + (FormPos_y[LeadForm_id] - FormPos_y[FolForm_id])*(FormPos_y[LeadForm_id] - FormPos_y[FolForm_id]));
	SP_alpha = x_m[1];
} else if (RobForm_id == LeadForm_id)
{
	SP_d = x_m[0] - 0.2828; //0.2828m is half the diagonal of a square with a side 0.4m
	SP_alpha = x_m[1];
} else {
	// Calculate the angle between this robot's coordinate system and Leader's coordinate system
	gamma = atan2(x_m[0]*sin(x_m[1]) - x_m[2]*sin(x_m[3]), x_m[0]*cos(x_m[1]) - x_m[2]*cos(x_m[3])) - atan2(FormPos_y[LeadForm_id] - FormPos_y[FolForm_id], FormPos_x[LeadForm_id] - FormPos_x[FolForm_id]) ;
	
	// Vector from Robot to Robot's desired formation position, in Robot's
	// coordinate system
	//Rx_targ = x_m[2]*cos(x_m[3])  + cos(gamma)*Fx_targ - sin(gamma)*Fy_targ;
	//Ry_targ = x_m[2]*sin(x_m[3])  + sin(gamma)*Fx_targ + cos(gamma)*Fy_targ;
	Rx_targ = x_m[0]*cos(x_m[1])  + cos(gamma)*Fx_targ - sin(gamma)*Fy_targ;
 	Ry_targ = x_m[0]*sin(x_m[1])  + sin(gamma)*Fx_targ + cos(gamma)*Fy_targ;
	//Transform to polar coordinates
	SP_d = sqrt(Rx_targ*Rx_targ + Ry_targ*Ry_targ);
	SP_alpha = atan2(Ry_targ,Rx_targ);
}

}


void PID_Drive(float d_est,float alpha_est,int u[2])
{
	float u_PID_d, u_PID_d_sat, u_PID_a,u_mean,u_diff,err_dist,err_alpha;
	static float I_err_dist, I_err_alpha, err_antiWindUp;
	static int SP_reached, DrivingBackwards;
	//See Simulator files (Matlab) for justification of these controller settings

	const float Kp_d = 5.0;
	const float Ki_d = 1.5;//6.6; // //0.0;
	const float Kp_a = 0.4; //0.4;
    const float Ki_a = 0.0; //0.6;
	const float u_diff_max = 0.75;
	const float u_mean_max = 0.95;
	const float u_PID_0 = 0.4775;
	const float u_diff_0 = 0.2854;
	const float Ts = 0.15;
	const float I_decay = 0.8; //0.9; //0.95;//0.8; 	
	const float L_antiwindup = 0.0;
	const float u_meanSlow_MaxAmp = 0.0; //0.0; //1.0 //[-] Maximal additional amplification of u_diff as input to a gaussian, when close to the target (u_PID_d small)
	const float u_meanSlow_d_0p10Amp = 0.45; //[m] Distance of 10% amplification, i.e. above this distance the effect of this feature is neglectable
	const float u_diffSlow_d_0p63Amp = 0.02; //0.02; //[m] Distance when err_alpha is multiplied by 0.6321, i.e. above this distance the effect of this feature gets smaller

	const float pos_tol =  0.01; //0.001;  //0.10  //[m] position tolerance. (setpoint considered reached if distance smaller than pos_tol)
	const float pos_hyst = 0.01; //0.01;//0.05; //[m] hysteresis on "setpoint reached" check
	const float driveBack_hyst = 0.1; //[rad] hysteresis on switch between driving forward and backwards

	err_dist = d_est;
	err_alpha = norm_angle(alpha_est); // error to minimize
	e_set_body_led(err_dist<0.07);
	//hysteresis on reaching setpoint
	if (fabs(err_dist)< pos_tol)
	{
		SP_reached = 1;
	} else if (fabs(err_dist) > (pos_tol+pos_hyst))
	{
		SP_reached = 0;
	}

	//if (SP_reached == 0)
	//{

		if ((fabs(err_alpha) > (PI/2.0 + driveBack_hyst)) & DrivingBackwards ==0 ) // drive backwards if angle larger than (PI/2 + hysteresis)
		{
			DrivingBackwards = 1;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		} else if ((fabs(err_alpha) < (PI/2.0 - driveBack_hyst)) & DrivingBackwards ==1 ) {
			DrivingBackwards = 0;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		}
	
		if (DrivingBackwards == 1)
		{
	    	err_alpha = norm_angle(alpha_est + PI);
	    	err_dist = -err_dist;
		}

		I_err_alpha = I_err_alpha*I_decay + err_alpha*Ts; //Discrete time integrator. I = I*decay + err*Ts
		I_err_dist  = I_err_dist*I_decay  + Ki_d*(err_dist - L_antiwindup*err_antiWindUp)*Ts;
		u_PID_a = Kp_a*err_alpha + Ki_a*I_err_alpha;
		u_PID_d = Kp_d*err_dist  + I_err_dist;
		u_diff = u_diff_max*atan(u_PID_a/u_PID_0)/(PI/2.0)*(1.0 - exp(-fabs(err_dist)/u_diffSlow_d_0p63Amp)); 
		// saturate u_PID_d
		if (u_PID_d > 1.0)
		{	
			u_PID_d_sat = 1.0;
		} else if (u_PID_d < -1.0) 
		{
			u_PID_d_sat = -1.0;
		} else 
		{
			u_PID_d_sat = u_PID_d;
		}
		u_PID_d_sat = u_PID_d_sat*exp(-powf(u_diff*(1.0 + u_meanSlow_MaxAmp/(9.0*fabs(u_PID_d)/u_meanSlow_d_0p10Amp + 1.0))/u_diff_0,2.0)); //When u_PID_d is small, u_diff gets multiplied by up to 5 -> low mean speed, when large angle error, but small distance error
		err_antiWindUp = u_PID_d - u_PID_d_sat;
        u_mean = u_PID_d_sat*u_mean_max; 

		//sprintf(buffer,"d:%5.2f,Id:%5.2f,u:%4.3f\r\n",err_dist,I_err_dist,u_mean); 	uart1_send_text(buffer);

	if (SP_reached == 1)
	{
		u_mean = 0.0;
	}

		u[0] = (int)(1000*(u_mean-u_diff));
		u[1] = (int)(1000*(u_mean+u_diff));
//} else {
//		u[0] = 0;
//		u[1] = 0;
//	}

return;
}

void PID_Drive_Lead(float d_est,float alpha_est,int u[2])
{
// Same as usual, but with limited max speed
	float u_PID_d, u_PID_d_sat, u_PID_a,u_mean,u_diff,err_dist,err_alpha;
	static float I_err_dist, I_err_alpha, err_antiWindUp;
	static int SP_reached, DrivingBackwards;
	//See Simulator files (Matlab) for justification of these controller settings

	const float Kp_d = 5.0;
	const float Ki_d = 6.6; //1.5; //0.0;
	const float Kp_a = 0.4; //0.4;
    const float Ki_a = 0.0; //0.6;
	const float u_diff_max = 0.75;
	const float u_mean_max = 0.50; //0.4;
	const float u_PID_0 = 0.4775;
	const float u_diff_0 = 0.2854;
	const float Ts = 0.15;
	const float I_decay = 0.8; //0.95; //0.9;	
	const float L_antiwindup = 0.0;
	const float u_meanSlow_MaxAmp = 0.0; //2.0; //1.0 //[-] Maximal additional amplification of u_diff as input to a gaussian, when close to the target (u_PID_d small)
	const float u_meanSlow_d_0p10Amp = 0.45; //[m] Distance of 10% amplification, i.e. above this distance the effect of this feature is neglectable
	const float u_diffSlow_d_0p63Amp = 0.02; //[m] Distance when err_alpha is multiplied by 0.6321, i.e. above this distance the effect of this feature gets smaller

	const float pos_tol =  0.02; //0.001;  //0.10  //[m] position tolerance. (setpoint considered reached if distance smaller than pos_tol)
	const float pos_hyst = 0.0; //0.01;//0.05; //[m] hysteresis on "setpoint reached" check
	const float driveBack_hyst = 0.1; //[rad] hysteresis on switch between driving forward and backwards

	err_dist = d_est;
	err_alpha = norm_angle(alpha_est); // error to minimize
	e_set_body_led(err_dist<0.07);
	//hysteresis on reaching setpoint
	if (fabs(err_dist)< pos_tol)
	{
		SP_reached = 1;
	} else if (fabs(err_dist) > (pos_tol+pos_hyst))
	{
		SP_reached = 0;
	}

	//if (SP_reached == 0)
	//{

		if ((fabs(err_alpha) > (PI/2.0 + driveBack_hyst)) & DrivingBackwards ==0 ) // drive backwards if angle larger than (PI/2 + hysteresis)
		{
			DrivingBackwards = 1;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		} else if ((fabs(err_alpha) < (PI/2.0 - driveBack_hyst)) & DrivingBackwards ==1 ) {
			DrivingBackwards = 0;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		}
	
		if (DrivingBackwards == 1)
		{
	    	err_alpha = norm_angle(alpha_est + PI);
	    	err_dist = -err_dist;
		}

		I_err_alpha = I_err_alpha*I_decay + err_alpha*Ts; //Discrete time integrator. I = I*decay + err*Ts
		I_err_dist  = I_err_dist*I_decay  + Ki_d*(err_dist - L_antiwindup*err_antiWindUp)*Ts;
		u_PID_a = Kp_a*err_alpha + Ki_a*I_err_alpha;
		u_PID_d = Kp_d*err_dist  + I_err_dist;
		u_diff = u_diff_max*atan(u_PID_a/u_PID_0)/(PI/2.0)*(1.0 - exp(-fabs(err_dist)/u_diffSlow_d_0p63Amp)); 
		// saturate u_PID_d
		if (u_PID_d > 1.0)
		{	
			u_PID_d_sat = 1.0;
		} else if (u_PID_d < -1.0) 
		{
			u_PID_d_sat = -1.0;
		} else 
		{
			u_PID_d_sat = u_PID_d;
		}
		u_PID_d_sat = u_PID_d_sat*exp(-powf(u_diff*(1.0 + u_meanSlow_MaxAmp/(9.0*fabs(u_PID_d)/u_meanSlow_d_0p10Amp + 1.0))/u_diff_0,2.0)); //When u_PID_d is small, u_diff gets multiplied by up to 5 -> low mean speed, when large angle error, but small distance error
		err_antiWindUp = u_PID_d - u_PID_d_sat;
        u_mean = u_PID_d_sat*u_mean_max; 

		//sprintf(buffer,"d:%5.2f,Id:%5.2f,u:%4.3f\r\n",err_dist,I_err_dist,u_mean); 	uart1_send_text(buffer);

	if (SP_reached == 1)
	{
		u_mean = 0.0;
	}

		u[0] = (int)(1000*(u_mean-u_diff));
		u[1] = (int)(1000*(u_mean+u_diff));
//} else {
//		u[0] = 0;
//		u[1] = 0;
//	}

return;
}

void PID_Drive_Basic(float d_est,float alpha_est,int u[2])
{
	float u_PID_a,u_mean,u_diff,err_dist,err_alpha;
	static float I_err_alpha;
	static int SP_reached, DrivingBackwards;
	//See Simulator files (Matlab) for justification of these controller settings


	const float Kp_a = 0.4; //0.4;
    const float Ki_a = 0.0; //0.6;
	const float u_diff_max = 0.75;
	const float u_mean_max = 0.95;
	const float u_PID_0 = 0.4775;
	const float u_diff_0 = 0.2854;
	const float d_slowdown = 0.2; //0.2; //0.01; //0.15 //[m] mean speed drops when distance to setpoint is less than this value. May not be 0.
	const float Ts = 0.15;
	const float I_decay = 0.8; //0.95; //0.9;	
	const float pos_tol =  0.02; //0.001;  //0.10  //[m] position tolerance. (setpoint considered reached if distance smaller than pos_tol)
	const float pos_hyst = 0.02; //0.01;//0.05; //[m] hysteresis on "setpoint reached" check
	const float driveBack_hyst = 0.1; //[rad] hysteresis on switch between driving forward and backwards

	err_dist = d_est;
	err_alpha = norm_angle(alpha_est); // error to minimize
	e_set_body_led(err_dist<0.05);
	//hysteresis on reaching setpoint
	if (fabs(err_dist)< pos_tol)
	{
		SP_reached = 1;
	} else if (fabs(err_dist) > (pos_tol+pos_hyst))
	{
		SP_reached = 0;
	}

	//if (SP_reached == 0)
	//{

		if ((fabs(err_alpha) > (PI/2.0 + driveBack_hyst)) & DrivingBackwards ==0 ) // drive backwards if angle larger than (PI/2 + hysteresis)
		{
			DrivingBackwards = 1;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		} else if ((fabs(err_alpha) < (PI/2.0 - driveBack_hyst)) & DrivingBackwards ==1 ) {
			DrivingBackwards = 0;
			// Have to flip integrators when changing direction of movement (??)
			//I_err_dist = - I_err_dist;
			I_err_alpha = - I_err_alpha;
		}
	
		if (DrivingBackwards == 1)
		{
	    	err_alpha = norm_angle(alpha_est + PI);
	    	err_dist = -err_dist;
		}

		I_err_alpha = I_err_alpha*I_decay + err_alpha*Ts; //Discrete time integrator. I = I*decay + err*Ts
		u_PID_a = Kp_a*err_alpha + Ki_a*I_err_alpha;
		u_diff = u_diff_max*atan(u_PID_a/u_PID_0)/(PI/2.0); 
        u_mean = (err_dist/fabs(err_dist))*u_mean_max*exp(-powf(u_diff/u_diff_0,2.0)); //*atan(err_dist*(5.0/d_slowdown))/(PI/2.0)

		//sprintf(buffer,"d:%5.2f,Id:%5.2f,u:%4.3f\r\n",err_dist,I_err_dist,u_mean); 	uart1_send_text(buffer);

	if (SP_reached == 1)
	{
		u_mean = 0.0;
	}

		u[0] = (int)(1000*(u_mean-u_diff));
		u[1] = (int)(1000*(u_mean+u_diff));
//} else {
//		u[0] = 0;
//		u[1] = 0;
//	}

return;
}

void BroadcastVolume(void)
{
	sprintf(buffer,"f,%d,%d,%d\r\n",e_get_micro_volume(0),e_get_micro_volume(1),e_get_micro_volume(2));
	uart1_send_text(buffer);
}


void PlaySignal(void)
{
	e_play_sound(par_playNote_id*3600,3600);  // 7200 samples = 1s
}

void PlaySignal0(void)
{
	e_play_sound(0,3600);  // 7200 samples = 1s
}

void PlaySignal0_short(void)
{
	e_play_sound(0,720);  // 7200 samples = 1s
}

void PlaySignal1(void)
{
	e_play_sound(3600,3600);  // 7200 samples = 1s
}
void PlaySignal2(void)
{
	e_play_sound(7200,3600); // 7200 samples = 1s
}

void PlaySignal3(void)
{
	e_play_sound(10800,3600);  // 7200 samples = 1s
}

void PlaySignal4(void)
{
	e_play_sound(14400,3600);  // 7200 samples = 1s
}


void BroadcastSetpoint(void)
{
	sprintf(buffer,"SP: d=%f,a=%f\r\n",SP_d,SP_alpha);
	uart1_send_text(buffer);
}

void BroadcastSrcPosEst(void)
{
	sprintf(buffer,"Src1: d=%f, a=%f; Src2: d=%f,a=%f\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);
	uart1_send_text(buffer);
}

void CtrlSetSpeed(void)
{
	float rnd;
	// selector 0 = run source position estimation, but stand still or move according to manually set motor speeds ("d" command)
	// selector 1 = drive towards the manually set setpoint ("g" or "x" command) and ignore source position estimation
	// selector 2 = run source position estimation, but drive towards the manually set setpoint
	// selector 3 = Triangle formation
	// selector 4 = Playing sound while not measuring
	// selector 5 (or default) = estimate sound sourse position and drive towards it. Current estimate can be overwritten by "g" command	
	switch (selector)
	{
		case 0:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			EKF_Source();
			break;
		case 1:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);			
			SrcPosMeas_new[0] = 0;
			SrcPosMeas_new[1] = 0;
			EKF_Source();
			Setpoint_User();
			PID_Drive(SP_d,SP_alpha,u_1k);
			break;
		case 2:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			SrcPosMeas_new[0] = 0;
			SrcPosMeas_new[1] = 0;
			EKF_Source();
			Setpoint_Source();
			PID_Drive(SP_d,SP_alpha,u_1k);
			break;
		case 3:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			EKF_Source();
			Setpoint_Triangle();
			PID_Drive(SP_d,SP_alpha,u_1k);
			break;
		case 4:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			//e_play_sound(par_playNote_id*3600,1024);

/*
			if (par_signal_id[0] == 4)
			{
				e_play_sound(0*3600,1024); //540 works for par_AgendaDelayEKF=1000 // 7200 samples = 1s;
			} else if (par_signal_id[0] == 3)
			{
				e_play_sound(4*3600,1024);  // 7200 samples = 1s;
			} else if (par_signal_id[0] == 2)
			{
				e_play_sound(3*3600,1024);  // 7200 samples = 1s;
			} else if (par_signal_id[0] == 1)
			{
				e_play_sound(2*3600,1024);  // 7200 samples = 1s;
			} else //par_signal_id[0] = 0
			{
				e_play_sound(1*3600,1024);  // 7200 samples = 1s;
			}
*/				
			if (SrcPosMeas_new[0] == 0)
			{
				rnd = fabsf(fmodf(log(rand()),1.0));
				soundDelay = 1000*floor(par_soundDelayMax*1000.0*rnd);
				//sprintf(buffer,"rnd = %f; delay = %ld\r\n",rnd,soundDelay); uart1_send_text(buffer);						
				wait(soundDelay);
				flag_didSoundDelay = 1;
			}
			EKF_Source();
			Setpoint_Source();
			PID_Drive(SP_d,SP_alpha,u_1k);
			break;
		case 5: // Formation; Similar to 3, but with one of the predefined setpoints and no mic update
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			SrcPosMeas_new[0] = 0;
			SrcPosMeas_new[1] = 0;
			EKF_Source();
			Setpoint_Form();
			if (RobForm_id == LeadForm_id)
			{
				PID_Drive_Lead(SP_d,SP_alpha,u_1k);
			} else if (RobForm_id == 3 & LeadForm_id == 9) { //HACK!!
				PID_Drive_Lead(SP_d,SP_alpha,u_1k);
			} else {
				PID_Drive(SP_d,SP_alpha,u_1k);
			}
			break;
		default:
			CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
			EKF_Source();
			Setpoint_Source();
			PID_Drive_Basic(SP_d,SP_alpha,u_1k);			
			break;
	}
	//sprintf(buffer,"SP = (%.3f,%.3f); u = (%d,%d); \r\n",SP_d,SP_alpha,u_1k[0],u_1k[1]); uart1_send_text(buffer);
	e_set_speed_left(u_1k[0]);
	e_set_speed_right(u_1k[1]);

	iter_counter++;
	if ( iter_counter%100 == 0)
	{
		//uart1_send_static_text("it100\r\n");
		sprintf(buffer,"it = %d; me0 = %d; me1 = %d\r\n",iter_counter,meas_counter[0],meas_counter[1]); uart1_send_text(buffer);		
	}
	if (selector ==4)
	{
		//e_play_sound(par_playNote_id*3600,1024);
		e_play_sound(par_playNote_id*3600,720);
	}
	//float alpha_meas = GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]]);
	//sprintf(buffer,"i,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3],EKF_P[0],EKF_P[5],alpha_meas); uart1_send_text(buffer);
	//flag_didSendComm++;
	//sprintf(buffer,"i,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\r\n",SP_d,SP_alpha,x_m[0],x_m[1],x_m[2],x_m[3],EKF_P[0],EKF_P[5]); uart1_send_text(buffer);
}

int CaptureFFT(unsigned int NSignals, unsigned int* signal_id, float* amp_at_freq, float* phase_at_freq)
{				
float par_SigStrThr = 5.0;       //Threshold on signal strength (weighted sum of frequencies of interest). Above this value the signal is considered to be present in the measurement.
float par_minAmpForPhase = 0.5; //0.2 // Minimal amplitude needed at frequency of interest, to be able to extract phase information.
								//(when amp is low, Re and Im parts are close to zero and because of numerical erorrs: phase = atan2(0,0) = NaN)

	//uart1_send_static_text("CaptureFFT - start\r\n");
				float fftReal, fftImag;
				float signalStr[NSignals];
				int i, iSig;
				int check_cnt = 0;
				iSig = 0;
				do
				{
					if	(signal_id[iSig] <= 4) //signal id must be one of: [0,1,2,3,4]
					{
						check_cnt++;
					}
					iSig++;
				} while (iSig<NSignals);
				if	(check_cnt == NSignals) 
				{
					e_last_mic_scan_id = 0;
					e_ad_scan_on();
					// Wait until first AD interrupt, which sets "e_ad_is_array_filled" to 0 and increases e_last_mic_scan_id (see e_ad_conv.c)
					do
					{
						NOP();	
					} while (e_last_mic_scan_id == 0);
					// Wait until array is filled, then stop AD converter and read arrays one by one
					do
					{
						NOP();	
					} while (!e_ad_is_array_filled());
					e_ad_scan_off();

					e_get_micro_last_values (0, &mic0_seq ,MIC_SAMP_NB);
					e_get_micro_last_values (1, &mic1_seq ,MIC_SAMP_NB);
					e_get_micro_last_values (2, &mic2_seq ,MIC_SAMP_NB);
/*
					sprintf(buffer,"MicData = [\r\n");
					uart1_send_text(buffer);
					i = 0;
					do
					{			
						sprintf(buffer,"%d %d %d\r\n",mic0_seq[i],mic1_seq[i],mic2_seq[i]);
						uart1_send_text(buffer);
					i++;
					} while (i<MIC_SAMP_NB);
					sprintf(buffer,"]';return\r\n");
					uart1_send_text(buffer);	
*/
					// We put the mean to zero
					e_subtract_mean(mic0_seq, FFT_BLOCK_LENGTH, LOG2_BLOCK_LENGTH);
					e_subtract_mean(mic1_seq, FFT_BLOCK_LENGTH, LOG2_BLOCK_LENGTH);
					e_subtract_mean(mic2_seq, FFT_BLOCK_LENGTH, LOG2_BLOCK_LENGTH);
					// Scale data to utilize maximal precision
					i = 0;
					do
					{			
						mic0_seq[i] = mic0_seq[i]*FFT_Prescaler;
						mic1_seq[i] = mic1_seq[i]*FFT_Prescaler;
						mic2_seq[i] = mic2_seq[i]*FFT_Prescaler;
						i++;
					} while (i<MIC_SAMP_NB);
					// Initialize result variables for every signal
					iSig = 0;
					do 
					{
						amp_at_freq[(3*iSig + 0)] = 0.0;
						amp_at_freq[(3*iSig + 1)] = 0.0;
						amp_at_freq[(3*iSig + 2)] = 0.0;			
						signalStr[iSig] = 0.0;
						iSig++;
					} while (iSig<NSignals);

					// MIC 0 FFT
					//-----------------------------------------------------
		 			// We copy the array of micro #0 to the FFT array
		 			e_fast_copy(mic0_seq, (int*)sigCmpx, FFT_BLOCK_LENGTH);
		 			// Now we are doing the FFT
		 			e_doFFT_asm(sigCmpx);

					iSig = 0;
					do
					{
						// Detecting Signal iSig
						i = 0;
						do
						{	
							if (par_freqToDetect_id[signal_id[iSig]]*(11-2*i) < 128)		
							{		
								fftReal = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].real)*(float)(65536/FFT_Prescaler);
								fftImag = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].imag)*(float)(65536/FFT_Prescaler);
								amp_at_freq[(3*iSig + 0)] = amp_at_freq[(3*iSig + 0)] + filters_short[(signal_id[iSig]*6 + (5-i))]*sqrt(fftReal*fftReal + fftImag*fftImag);
							}
							i++;
						} while (i<6);
	
						if ((fftReal*fftReal + fftImag*fftImag) > (par_minAmpForPhase*par_minAmpForPhase))
						{
							phase_at_freq[(3*iSig + 0)] = atan2(fftImag,fftReal);
						} else 
						{
							phase_at_freq[(3*iSig + 0)] = INVALID_PHASE;
						}
						iSig++;
					} while (iSig < NSignals);

					// MIC 1 FFT
					//-----------------------------------------------------
		 			// We copy the array of micro #1 to the FFT array
		 			e_fast_copy(mic1_seq, (int*)sigCmpx, FFT_BLOCK_LENGTH);
		 			// Now we are doing the FFT
		 			e_doFFT_asm(sigCmpx);

					iSig = 0;
					do
					{
						// Detecting Signal iSig
						i = 0;
						do
						{	
							if (par_freqToDetect_id[signal_id[iSig]]*(11-2*i) < 128)		
							{		
								fftReal = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].real)*(float)(65536/FFT_Prescaler);
								fftImag = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].imag)*(float)(65536/FFT_Prescaler);
								amp_at_freq[(3*iSig + 1)] = amp_at_freq[(3*iSig + 1)] + filters_short[(signal_id[iSig]*6 + (5-i))]*sqrt(fftReal*fftReal + fftImag*fftImag);
							}
							i++;
						} while (i<6);
	
						if ((fftReal*fftReal + fftImag*fftImag) > (par_minAmpForPhase*par_minAmpForPhase))
						{
							phase_at_freq[(3*iSig + 1)] = atan2(fftImag,fftReal);
						} else 
						{
							phase_at_freq[(3*iSig + 1)] = INVALID_PHASE;
						}
						iSig++;
					} while (iSig < NSignals);

					// MIC 2 FFT
					//-----------------------------------------------------
		 			// We copy the array of micro #2 to the FFT array
		 			e_fast_copy(mic2_seq, (int*)sigCmpx, FFT_BLOCK_LENGTH);
		 			// Now we are doing the FFT
		 			e_doFFT_asm(sigCmpx);

					iSig = 0;
					do
					{
						// Detecting Signal iSig
						i = 0;
						do
						{	
							if (par_freqToDetect_id[signal_id[iSig]]*(11-2*i) < 128)		
							{		
								fftReal = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].real)*(float)(65536/FFT_Prescaler);
								fftImag = Fract2Float(sigCmpx[par_freqToDetect_id[signal_id[iSig]]*(11-2*i)].imag)*(float)(65536/FFT_Prescaler);
								amp_at_freq[(3*iSig + 2)] = amp_at_freq[(3*iSig + 2)] + filters_short[(signal_id[iSig]*6 + (5-i))]*sqrt(fftReal*fftReal + fftImag*fftImag);
							}
							i++;
						} while (i<6);
	
						if ((fftReal*fftReal + fftImag*fftImag) > (par_minAmpForPhase*par_minAmpForPhase))
						{
							phase_at_freq[(3*iSig + 2)] = atan2(fftImag,fftReal);
						} else 
						{
							phase_at_freq[(3*iSig + 2)] = INVALID_PHASE;
						}
						iSig++;
					} while (iSig < NSignals);

					iSig = 0;
					do
					{
						signalStr[iSig] = amp_at_freq[(3*iSig + 0)]+amp_at_freq[(3*iSig + 1)]+amp_at_freq[(3*iSig + 2)];
						//sprintf(buffer,"str = %f\r\n",signalStr); uart1_send_text(buffer);
						flag_didFFT++; //This flag is evaluated in EKF_Source() function
						if ((signalStr[iSig] > par_SigStrThr) & (phase_at_freq[3*iSig+0]!=INVALID_PHASE) & (phase_at_freq[3*iSig+1]!=INVALID_PHASE) & (phase_at_freq[3*iSig+2]!=INVALID_PHASE))
						{						
							SrcPosMeas_new[iSig] = 1;
							meas_counter[iSig]++;
							//uart1_send_static_text("new\r\n");
							//sprintf(buffer,"new%d\r\n",iSig); uart1_send_text(buffer);							
						} else {
							SrcPosMeas_new[iSig] = 0;
						}
						iSig++;
					} while (iSig<NSignals);
					//sprintf(buffer,"phase_at_freq=[%f, %f, %f, %f, %f, %f]\r\n",phase_at_freq[0],phase_at_freq[1],phase_at_freq[2],phase_at_freq[3],phase_at_freq[4],phase_at_freq[5]); uart1_send_text(buffer);
					return 0;

				} else { // invalid signal_id
					amp_at_freq[0] = 0.0;
					amp_at_freq[1] = 0.0;
					amp_at_freq[2] = 0.0;
					amp_at_freq[3] = 0.0;
					amp_at_freq[4] = 0.0;
					amp_at_freq[5] = 0.0;
					phase_at_freq[0] = 0.0;
					phase_at_freq[1] = 0.0;
					phase_at_freq[2] = 0.0;
					phase_at_freq[3] = 0.0;
					phase_at_freq[4] = 0.0;
					phase_at_freq[5] = 0.0;
					return 2;
				}
}

void init_EKF_Source (void)
{
// Fill all matrices with array indices. Unnecessary, but helpful for debuging
//sprintf(buffer,"P = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]); uart1_send_text(buffer);

int i = 0;

//Reset counters
iter_counter = 0;

i=0;
do
{
	meas_counter[i] = 0;
	i++;
} while (i<NSIG);

i=0;
do
{
	if (i<MSIZE_N*MSIZE_N*NSTORESTEPS)
	{
		EKF_P[i] = 0.0;
		EKF_F[i] = 0.0;	// [NxN] 2x2 Jacobian of process modell with respect to states (time variable)
	}
	if (i<MSIZE_P*MSIZE_P)
	{
		EKF_Q[i] = 0.0;   // [PxP] 4x4 Covariance matrix of process noise; diag(left wheel noise ,right wheel noise)
	}
	if (i<MSIZE_N*MSIZE_P)
	{
		EKF_L[i] = 0.0;	// [NxP] 2x4 Jacobian of process modell with respect to noise (time variable)
	}
	if (i<MSIZE_T*MSIZE_T)
	{
		EKF_R0[i] = 0.0;   // [TxT] 2x2 Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
	}
	if (i<MSIZE_R*MSIZE_N)
	{
		EKF_H0[i] = 0.0;   // [RxN] 2x2 Jacobian of measurement modell with respect to states (time variable)
	}
	if (i<MSIZE_T*MSIZE_T)
	{
		EKF_R1[i] = 0.0;   // [TxT] 2x2 Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
	}
	if (i<MSIZE_R*MSIZE_N)
	{
		EKF_H1[i] = 0.0;   // [RxN] 2x2 Jacobian of measurement modell with respect to states (time variable)
	}
	if (i<MSIZE_T*MSIZE_T)
	{
		EKF_R2[i] = 0.0;   // [TxT] 2x2 Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
	}
	if (i<MSIZE_R*MSIZE_N)
	{
		EKF_H2[i] = 0.0;   // [RxN] 2x2 Jacobian of measurement modell with respect to states (time variable)
	}
	if (i<MSIZE_T*MSIZE_T)
	{
		EKF_R3[i] = 0.0;   // [TxT] 2x2 Covariance matrix of measurement noise; 
	}
	if (i<MSIZE_R4*MSIZE_R4)
	{
		EKF_R4[i] = 0.0;   // [R4xR4] 1x1 Covariance matrix of measurement noise;
	}
	if (i<MSIZE_R*MSIZE_N)
	{
		EKF_H3[i] = 0.0;   // [RxN] 2x2 Jacobian of measurement modell with respect to states (time variable)
	}
	if (i<MSIZE_R4*MSIZE_N)
	{
		EKF_H4[i] = 0.0;   // [R4xN] 1x2 Jacobian of measurement modell with respect to states (time variable)
	}
	if (i<MSIZE_N*MSIZE_MAX)
	{
		EKF_K[i] = 0.0;	// [NxR] 4x2 Kalman gain matrix
	}
	if (i<MSIZE_N*MSIZE_N)
	{
		EKF_D[i]   = 0.0;  // [NxN] 4x4 Delay correction matrix for camera measurements
	}	

	EKF_TEMP1[i] = i;  // temporary storage for intermediate values in matrix calculations
	EKF_TEMP2[i] = i;  // temporary storage for intermediate values in matrix calculations
	EKF_TEMP3[i] = i;  // temporary storage for intermediate values in matrix calculations
	i++;
} while (i<MSIZE_MAX*MSIZE_MAX);

uart1_send_static_text("Init matrix complete\r\n");
//sprintf(buffer,"P = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]); uart1_send_text(buffer);

// Initialization
for (i=0; i<NSTORESTEPS; i++)
{
	x_m[0 + MSIZE_N*(i)] = 0.15;  //[m] distance to source 0
	x_m[1 + MSIZE_N*(i)] = 0.0;   //[rad] angle to source 0
	x_m[2 + MSIZE_N*(i)] = 0.15;  //[m] distance to source 1
	x_m[3 + MSIZE_N*(i)] = PI/2.0; //[rad] angle to source 1
}
// Covariance matrix of the initial estimate 
//(should be high at the start)
EKF_P[0] = 5.0;
EKF_P[5] = 5.0;
EKF_P[10] = 5.0;
EKF_P[15] = 5.0;
//sprintf(buffer,"P = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]); uart1_send_text(buffer);

//Jacobian of process model w.r.t. states
EKF_F[0] = 1.0;
EKF_F[10] = 1.0;

// Jacobian of process model w.r.t. noise
EKF_L[0] = 1.0;
EKF_L[5] = 1.0;
EKF_L[10] = 1.0;
EKF_L[15] = 1.0;

// Covariance matrix of process noise
EKF_Q[0] = par_sigma_Add_d*par_sigma_Add_d;
EKF_Q[5] = par_sigma_Add_alpha*par_sigma_Add_alpha;
EKF_Q[10] = par_sigma_Add_d*par_sigma_Add_d;
EKF_Q[15] = par_sigma_Add_alpha*par_sigma_Add_alpha;

//Measurement Update 0: distance and angle to source1 from FFT
//-----------------------------------------------------------------------
// Jacobian of measurement model w.r.t. states
if(selector==5)
{
	EKF_H0[0] = 0.0; //1.0; //If this value is 0, then distance measurement from microphones will be ignored
	EKF_H0[5] = 1.0;
} else {
	EKF_H0[0] = 1.0; //1.0; //If this value is 0, then distance measurement from microphones will be ignored
	EKF_H0[5] = 1.0;
}


// Covariance matrix of measurement noise
EKF_R0[0] = par_sigma_Mic_d*par_sigma_Mic_d;
EKF_R0[3] = par_sigma_Mic_alpha*par_sigma_Mic_alpha;

//Measurement Update 1: distance and angle to source2 from FFT
//-----------------------------------------------------------------------
// Jacobian of measurement model w.r.t. states

if(selector==5)
{
	EKF_H1[2] = 0.0; //1.0; //If this value is 0, then distance measurement from microphones will be ignored
	EKF_H1[7] = 1.0;
} else {
	EKF_H1[2] = 1.0; //1.0; //If this value is 0, then distance measurement from microphones will be ignored
	EKF_H1[7] = 1.0;
}


// Covariance matrix of measurement noise
EKF_R1[0] = par_sigma_Mic_d*par_sigma_Mic_d;
EKF_R1[3] = par_sigma_Mic_alpha*par_sigma_Mic_alpha;

//Measurement Update 2: distance and angle to source1 from Camera
//-----------------------------------------------------------------------
// Jacobian of measurement model w.r.t. states
EKF_H2[0] = 1.0;
EKF_H2[5] = 1.0;

// Covariance matrix of measurement noise
EKF_R2[0] = par_sigma_Cam_d*par_sigma_Cam_d;
EKF_R2[3] = par_sigma_Cam_alpha*par_sigma_Cam_alpha;

//Measurement Update 2: distance and angle to source2 from Camera
//-----------------------------------------------------------------------
// Jacobian of measurement model w.r.t. states
EKF_H3[2] = 1.0;
EKF_H3[7] = 1.0;

// Covariance matrix of measurement noise
EKF_R3[0] = par_sigma_Cam_d*par_sigma_Cam_d;
EKF_R3[3] = par_sigma_Cam_alpha*par_sigma_Cam_alpha;

//Measurement Update 4: angle from other robot in formation
EKF_R4[0] = par_sigma_Mic_d*par_sigma_Mic_d;
EKF_R4[3] = par_sigma_Mic_d*par_sigma_Mic_d;
//EKF_R4[4] = par_sigma_Mic_alpha*par_sigma_Mic_alpha;
//EKF_R4[8] = par_sigma_Mic_alpha*par_sigma_Mic_alpha;

// Delay correction matrix for camera measurements
EKF_D[0] = 1.0;
EKF_D[5] = 1.0;
EKF_D[10] = 1.0;
EKF_D[15] = 1.0;

//For Calibration filter
// 2x2 covariance matrix of calibration filter states for mic0
CalParamMic0_P[0] = 1.0;
CalParamMic0_P[1] = 0.0;
CalParamMic0_P[2] = 0.0;
CalParamMic0_P[3] = 1.0;	
// 2x2 covariance matrix of calibration filter states for mic1
CalParamMic1_P[0] = 1.0;
CalParamMic1_P[1] = 0.0;
CalParamMic1_P[2] = 0.0;
CalParamMic1_P[3] = 1.0;
// 2x2 covariance matrix of calibration filter states for mic2
CalParamMic2_P[0] = 1.0;
CalParamMic2_P[1] = 0.0;
CalParamMic2_P[2] = 0.0;
CalParamMic2_P[3] = 1.0;

}

void EKF_Source(void)
{

//Step 0: Initialisation
/********************************************/
float x_p[MSIZE_N]; // States after prediction step
float y[MSIZE_R];   // Errors between prediction and measurement
float z[MSIZE_R];	// Measurements
//Transform decimal integers into floats
//float Ts = (((float)par_AgendaDelayEKF)/10000.0) + 0.015 + 0.035*(float)flag_didFFT + 0.032*(float)flag_didMeasUp; //Sampling time of Extended Kalman Filter [s];
float Ts = (((float)par_AgendaDelayEKF)/10000.0) + 0.089 + (float)(soundDelay/1000000)*(float)flag_didSoundDelay  + 0.005*(float)flag_didCommUp; //+ 0.057*(float)flag_didSendComm; //Sampling time of Extended Kalman Filter [s]
//float Ts = (((float)par_AgendaDelayEKF)/10000.0) + 0.043 + 0.035*(float)flag_didFFT + 1.0*(float)flag_didSoundDelay; //Sampling time of Extended Kalman Filter [s]

// 0.078s with 2 meas updates (rel. coord)
// 0.086s with 4 meas updates (rel. coord)

flag_didSoundDelay = 0; //this flag is set to 1 in CtrlSetSpeed() function
flag_didFFT = 0; //this flag is set to 1 in CaptureFFT() function
flag_didCommUp =0; //this flag is set later in this function, if there is a communication update
flag_didSendComm =0; //this flag is set in "I" and "V" commands, when regular updates are sent to PC
float u_norm[2];
u_norm[0] = (float)u_1k[0]/1000.0; //left wheel input [0..1]
u_norm[1] = (float)u_1k[1]/1000.0; //right wheel input [0..1]

//Precompute some reoccuring values
float v_avg, omega, sin_alpha1, cos_alpha1,sin_alpha2, cos_alpha2,denom1, denom2;//,SrcPosEst_d,SrcPosEst_alpha;
v_avg = par_v_max/2.0*(u_norm[0]+u_norm[1]); 	  // translation speed of the robot [m/s]
omega = par_v_max/par_b_wheels*(u_norm[1]-u_norm[0]); // rotation speed about the robot's center [rad/s]
sin_alpha1 = sinf(x_m[1]);
cos_alpha1 = cosf(x_m[1]);
sin_alpha2 = sinf(x_m[3]);
cos_alpha2 = cosf(x_m[3]);

// Calculate/check denominators to avoid singularity at dist=0.
if (fabs(x_m[0]) < par_EKF_min_d)
{
	denom1 = par_EKF_min_d;
} else 
{
	denom1 = x_m[0];
}

if (fabs(x_m[2]) < par_EKF_min_d)
{
	denom2 = par_EKF_min_d;
} else 
{
	denom2 = x_m[2];
}

// Store old states
Tr_MatrixTimeShift(MSIZE_N, 1, NSTORESTEPS, x_m, x_m);
Tr_MatrixTimeShift(MSIZE_N, MSIZE_N, NSTORESTEPS, EKF_P, EKF_P);
Tr_MatrixTimeShift(MSIZE_N, MSIZE_N, NSTORESTEPS, EKF_F, EKF_F);

//Step 1: Prediction
/********************************************/
x_p[0] = x_m[0] - v_avg*cos_alpha1*Ts;
x_p[1] = x_m[1] + v_avg/denom1*sin_alpha1*Ts - omega*Ts;
x_p[2] = x_m[2] - v_avg*cos_alpha2*Ts;
x_p[3] = x_m[3] + v_avg/denom2*sin_alpha2*Ts - omega*Ts;
//sprintf(buffer,"x_p = [%f,%f,%f,%f;]\r\n",x_p[0],x_p[1],x_p[2],x_p[3]); uart1_send_text(buffer);

// Only variable terms of F are calculated, the rest of F is an identity matrix and has been declared in init_EKF_source() function
EKF_F[1] = v_avg*sin_alpha1*Ts;
EKF_F[4] = -v_avg/denom1/denom1*sin_alpha1*Ts;
EKF_F[5] =  1.0+v_avg/denom1*cos_alpha1*Ts;
EKF_F[11] = v_avg*sin_alpha2*Ts;
EKF_F[14] = -v_avg/denom2/denom2*sin_alpha2*Ts;
EKF_F[15] =  1.0+v_avg/denom2*cos_alpha2*Ts;

// EKF_L = eye(4) and is defined in init_EKF_Source() function

//Calculating P_p = F*P_m_old*F' + L*Q*L'; 
//sprintf(buffer,"P = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f;]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]); uart1_send_text(buffer);
//sprintf(buffer,"F = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f;]\r\n",EKF_F[0],EKF_F[1],EKF_F[2],EKF_F[3],EKF_F[4],EKF_F[5],EKF_F[6],EKF_F[7],EKF_F[8],EKF_F[9],EKF_F[10],EKF_F[11],EKF_F[12],EKF_F[13],EKF_F[14],EKF_F[15]); uart1_send_text(buffer);
//sprintf(buffer,"L = [%f,%f,%f,%f,%f,%f; %f,%f,%f,%f,%f,%f; %f,%f,%f,%f,%f,%f; %f,%f,%f,%f,%f,%f;]\r\n",EKF_L[0],EKF_L[1],EKF_L[2],EKF_L[3],EKF_L[4],EKF_L[5],EKF_L[6],EKF_L[7],EKF_L[8],EKF_L[9],EKF_L[10],EKF_L[11],EKF_L[12],EKF_L[13],EKF_L[14],EKF_L[15],EKF_L[16],EKF_L[17],EKF_L[18],EKF_L[19],EKF_L[20],EKF_L[21],EKF_L[22],EKF_L[23]); uart1_send_text(buffer);
//sprintf(buffer,"Q = diag([%f,%f,%f,%f,%f,%f])\r\n",EKF_Q[0],EKF_Q[7],EKF_Q[14],EKF_Q[21],EKF_Q[28],EKF_Q[35]); uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_N, EKF_TEMP1, EKF_F); // Temp1 = F'
//sprintf(buffer,"TMP1 = F' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_P, EKF_TEMP1); // Temp2 = P_p*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_P, EKF_F, EKF_TEMP2); // P_p = F*Temp2
//sprintf(buffer,"P = F*TMP1 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_P, EKF_TEMP1, EKF_L); // Temp1 = L'
//sprintf(buffer,"TMP1 = L' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_P,MSIZE_P,MSIZE_N, EKF_TEMP2, EKF_Q, EKF_TEMP1); // Temp2 = Q*Temp1
//sprintf(buffer,"TMP2 = Q*TMP1 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_P,MSIZE_N, EKF_TEMP1, EKF_L, EKF_TEMP2); // Temp1 = L*Temp2
//sprintf(buffer,"TMP1 = L*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_N,MSIZE_N,EKF_P, EKF_P, EKF_TEMP1);
//sprintf(buffer,"P = P+TMP1 = [%f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f; %f,%f,%f,%f;]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3],EKF_P[4],EKF_P[5],EKF_P[6],EKF_P[7],EKF_P[8],EKF_P[9],EKF_P[10],EKF_P[11],EKF_P[12],EKF_P[13],EKF_P[14],EKF_P[15]); uart1_send_text(buffer);

/*
//Limit P_p to some reasonable values (avoid P_p = [inf,inf;inf;inf])
if (EKF_P[0] > 1.0) //St.Dev of 1m
{
	uart1_send_static_text("EKF: P[0] saturated\r\n");
	EKF_P[0] = 1.0; //Saturate Variance of distance to 1 m^2
	EKF_P[1] = 0.0; //Destroy Covariance
	EKF_P[2] = 0.0; //Destroy Covariance
}
if (EKF_P[3] > 10.0) //St.Dev of PI
{
	uart1_send_static_text("EKF: P[3] saturated\r\n");
	EKF_P[3] = 10.0; //Saturate Variance of angle to 10 rad^2 (ca. PI^2)
	EKF_P[1] = 0.0; //Destroy Covariance
	EKF_P[2] = 0.0; //Destroy Covariance
}
*/

//Step 2: Correction by Measurement
/********************************************/
//x_m = x_p if no updates follow
x_m[0] = x_p[0];
x_m[1] = x_p[1];
x_m[2] = x_p[2];
x_m[3] = x_p[3];
z[0] = 0.0;
z[1] = 0.0;
flag_didMeasUp = 0; //This flag will be evaluated at the next call of EKF_Source function

//Measurement Update 0
//---------------------------------------------------------------------------------------
//x_m from previous update (if any) is input to this update
x_p[0] = x_m[0];
x_p[1] = x_m[1];
x_p[2] = x_m[2];
x_p[3] = x_m[3];

//if (SrcPosMeas_new[0] == 1)
//{

//if (isNaN(MicFreqMeasAmp[0])) uart1_send_static_text("MicFreqMeasAmp[0] = NaN\r\n"); 
//if (isNaN(MicFreqMeasAmp[1])) uart1_send_static_text("MicFreqMeasAmp[1] = NaN\r\n");
//if (isNaN(MicFreqMeasAmp[2])) uart1_send_static_text("MicFreqMeasAmp[2] = NaN\r\n");
z[0] = GetDistMeasurement(par_signal_id[0],&MicFreqMeasAmp[0],x_p[1]);
//if (isNaN(z[0])) uart1_send_static_text("z[0] = NaN\r\n"); 
z[1] = GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]]);
//if (isNaN(z[1])) uart1_send_static_text("z[1] = NaN\r\n"); 
//sprintf(buffer,"new z0 = [%f; %f;]\r\n",z[0],z[1]); uart1_send_text(buffer);
if ((par_CapDist==1) & z[0]>par_CapDistThres)
{
	z[0]=par_CapDistValue;
} 

if ((par_RejectAngles==1) & (fabs(norm_angle(z[1]-x_p[1]))>par_RejectAngleThres))
{
	SrcPosMeas_new[0] = 0;
	//EKF_H0[5] = 0.0;
} else
{
	//EKF_H0[5] = 1.0;
}
/*
if (SrcPosMeas_new[0] == 1)
{
	sprintf(buffer,"new z0 = [%f; %f;]\r\n",z[0],norm_angle(z[1])); uart1_send_text(buffer);
}
*/
//sprintf(buffer,"%f %f;\r\n",z[0],norm_angle(z[1])); uart1_send_text(buffer);

// Error
y[0] = z[0] - x_p[0];
y[1] = norm_angle(z[1]  - x_p[1]);
//sprintf(buffer,"y = [%f; %f;]\r\n",y[0],y[1]); uart1_send_text(buffer);

// H has been declared in init_EKF_source() function
// H[0] = 1.0;

/*
int i;
for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
{
	if (isNaN(EKF_P[i]))
	{
		sprintf(buffer,"P[%d] = NaN\r\n",i);
		uart1_send_text(buffer);
	}
}
*/

//Calculating K = P_p*H'*inv(H*P_p*H' + R);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H0); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_TEMP2, EKF_P, EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R,MSIZE_N,MSIZE_R, EKF_TEMP1, EKF_H0, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_R,MSIZE_R,EKF_TEMP2, EKF_TEMP1, EKF_R0); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
//Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
if(Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2) != 0)   //; //Temp3 = inv(Temp2);
{
	sprintf(buffer,"TMP2 = [%f,%f; %f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}
//sprintf(buffer,"TMP3 = inv(TMP2) =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H0); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_R, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, EKF_P, EKF_TEMP2); // K = P*Temp2
//sprintf(buffer,"K = P*TMP2 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5],EKF_K[6],EKF_K[7]);uart1_send_text(buffer);

//Calculating x_m = x_p + K*y
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f; %f; %f; %f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
if  (SrcPosMeas_new[0] == 1)
{
	Tr_MatrixAdd (MSIZE_N,1,x_m, x_p, EKF_TEMP1); // x_m = x_p + Temp1;
	//sprintf(buffer,"x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);uart1_send_text(buffer);
} else
{
	Tr_MatrixAdd (MSIZE_N,1,EKF_TEMP2, x_p, EKF_TEMP1); // Temp2 = x_p + Temp1;
	//sprintf(buffer,"FAKE x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}

//Calculating P_m = P_p - K*H*P_p;
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H0); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_TEMP1, EKF_P); // Temp2 = Temp1*P
//sprintf(buffer,"TMP1 = TMP2*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);

if  (SrcPosMeas_new[0] == 1)
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P, EKF_P, EKF_TEMP2); // P = P - Temp2
	//sprintf(buffer,"P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
	SrcPosMeas_new[0] = 0;
} else
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_TEMP1 ,EKF_P, EKF_TEMP2); // Temp1 = P - Temp2
	//sprintf(buffer,"FAKE P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
}

/* This covariance update rule is optimal for any K. Shorter version (above) is only valid when using MMSE optimal gain K
//Calculating P_m = (I - K*H)*P_p*(I - K*H)' + K*R*K';
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H0); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixSubtractFromId (MSIZE_N, EKF_TEMP2, EKF_TEMP1); //Temp2 = (I - Temp1)
//sprintf(buffer,"TMP2 = I - TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_N, EKF_TEMP3, EKF_TEMP2); // Temp3 = Temp2'
//sprintf(buffer,"TMP3 = TMP2' =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP1, EKF_P, EKF_TEMP3); // Temp1 = P*Temp3
//sprintf(buffer,"TMP1 = P*TMP3 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP3, EKF_TEMP2, EKF_TEMP1); // Temp3 = Temp2*Temp1
//sprintf(buffer,"TMP3 = TMP2*TMP1 = [%f,%f; %f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3]); uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_R, EKF_TEMP1, EKF_K); // Temp1 = K'
//sprintf(buffer,"TMP1 = K' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R,MSIZE_R,MSIZE_N, EKF_TEMP2, EKF_R0, EKF_TEMP1); // Temp2 = R*Temp1
//sprintf(buffer,"TMP2 = R*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_TEMP2); // Temp1 = K*Temp2
//sprintf(buffer,"TMP1 = K*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);

if  (SrcPosMeas_new[0] == 1)
{
	Tr_MatrixAdd (MSIZE_N,MSIZE_N,EKF_P, EKF_TEMP3, EKF_TEMP1); // P = Temp3 + Temp1
	//sprintf(buffer,"P = TMP3+TMP1 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
	SrcPosMeas_new[0] = 0;
} else
{
	Tr_MatrixAdd (MSIZE_N,MSIZE_N,EKF_TEMP2, EKF_TEMP3, EKF_TEMP1); // Temp2 = Temp3 + Temp1
	//sprintf(buffer,"FAKE P = TMP3+TMP1 = [%f,%f; %f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]); uart1_send_text(buffer);
}
*/

//SrcPosMeas_new[0] = 0;
//flag_didMeasUp++; //This flag will be evaluated at the next call of EKF_Source function
//}

//Measurement Update 1
//---------------------------------------------------------------------------------------
//x_m from previous update (if any) is input to this update
x_p[0] = x_m[0];
x_p[1] = x_m[1];
x_p[2] = x_m[2];
x_p[3] = x_m[3];

//if (SrcPosMeas_new[1] == 1)
//{

//if (isNaN(x_p[2])) uart1_send_static_text("x_p[2] = NaN\r\n");
//if (isNaN(MicFreqMeasAmp[0])) uart1_send_static_text("MicFreqMeasAmp[0] = NaN\r\n"); 
//if (isNaN(MicFreqMeasAmp[1])) uart1_send_static_text("MicFreqMeasAmp[1] = NaN\r\n");
//if (isNaN(MicFreqMeasAmp[2])) uart1_send_static_text("MicFreqMeasAmp[2] = NaN\r\n");
z[0] = GetDistMeasurement(par_signal_id[1],&MicFreqMeasAmp[3],x_p[3]);
//if (isNaN(z[0])) uart1_send_static_text("z[0] = NaN\r\n"); 
z[1] = GetPhaseShiftAngleMat(&MicFreqMeasPhase[3],par_freqToDetect[par_signal_id[1]]);
//if (isNaN(z[1])) uart1_send_static_text("z[1] = NaN\r\n"); 

//sprintf(buffer,"new z1 = [%f; %f;]\r\n",z[0],z[1]); uart1_send_text(buffer);
//sprintf(buffer,"%f %f;\r\n",z[0],norm_angle(z[1])); uart1_send_text(buffer);

if ((par_CapDist==1) & z[0]>par_CapDistThres)
{
	z[0]=par_CapDistValue;
} 
if ((par_RejectAngles==1) & (fabs(norm_angle(z[1]-x_p[3]))>par_RejectAngleThres))
{
	SrcPosMeas_new[1] = 0;
	//EKF_H1[7] = 0.0;
} else
{
	//EKF_H1[7] = 1.0;
}

// Error
y[0] = z[0] - x_p[2];
y[1] = norm_angle(z[1]  - x_p[3]);
//sprintf(buffer,"y = [%f; %f;]\r\n",y[0],y[1]); uart1_send_text(buffer);

// H has been declared in init_EKF_source() function
// H[0] = 1.0;

/*
int i;
for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
{
	if (isNaN(EKF_P[i]))
	{
		sprintf(buffer,"P[%d] = NaN\r\n",i);
		uart1_send_text(buffer);
	}
}
*/

//Calculating K = P_p*H'*inv(H*P_p*H' + R);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H1); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_TEMP2, EKF_P, EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R,MSIZE_N,MSIZE_R, EKF_TEMP1, EKF_H1, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_R,MSIZE_R,EKF_TEMP2, EKF_TEMP1, EKF_R1); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
//Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
if(Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2) != 0)   //; //Temp3 = inv(Temp2);
{
	sprintf(buffer,"TMP2 = [%f,%f; %f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}
//sprintf(buffer,"TMP3 = inv(TMP2) =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H1); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_R, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, EKF_P, EKF_TEMP2); // K = P*Temp2
//sprintf(buffer,"K = P*TMP2 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5],EKF_K[6],EKF_K[7]);uart1_send_text(buffer);

//Calculating x_m = x_p + K*y
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f; %f; %f; %f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
if  (SrcPosMeas_new[1] == 1)
{
	Tr_MatrixAdd (MSIZE_N,1,x_m, x_p, EKF_TEMP1); // x_m = x_p + Temp1;
	//sprintf(buffer,"x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);uart1_send_text(buffer);
} else
{
	Tr_MatrixAdd (MSIZE_N,1,EKF_TEMP2, x_p, EKF_TEMP1); // Temp2 = x_p + Temp1;
	//sprintf(buffer,"FAKE x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}

//Calculating P_m = P_p - K*H*P_p;
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H1); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_TEMP1, EKF_P); // Temp2 = Temp1*P
//sprintf(buffer,"TMP1 = TMP2*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);

if  (SrcPosMeas_new[1] == 1)
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P, EKF_P, EKF_TEMP2); // P = P - Temp2
	//sprintf(buffer,"P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);	
    SrcPosMeas_new[1] = 0;
} else
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_TEMP1 ,EKF_P, EKF_TEMP2); // Temp1 = P - Temp2
	//sprintf(buffer,"FAKE P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
}

//SrcPosMeas_new[1] = 0;
//flag_didMeasUp++; //This flag will be evaluated at the next call of EKF_Source function
//}

//Measurement Update 2: (d,alpha) from camera
//---------------------------------------------------------------------------------------
//x_m from previous update (if any) is input to this update
x_p[0] = x_m[0];
x_p[1] = x_m[1];
x_p[2] = x_m[2];
x_p[3] = x_m[3];

//if (SrcPosMeas_new[0] == 1)
//{

z[0] = SrcPosMeasCam_d[0];
z[1] = SrcPosMeasCam_alpha[0];

//sprintf(buffer,"%f %f;\r\n",z[0],norm_angle(z[1])); uart1_send_text(buffer);

// Error
if (NSTORESTEPS == 1) {
	y[0] = z[0] - x_p[0];
	y[1] = norm_angle(z[1]  - x_p[1]);
} else {
	y[0] = z[0] - x_m[0 + MSIZE_N*(NSTORESTEPS-1)];
	y[1] = norm_angle(z[1]  - x_m[1 + MSIZE_N*(NSTORESTEPS-1)]);
}
//sprintf(buffer,"y = [%f; %f;]\r\n",y[0],y[1]); uart1_send_text(buffer);

// H has been declared in init_EKF_source() function
// H[0] = 1.0;

/*
int i;
for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
{
	if (isNaN(EKF_P[i]))
	{
		sprintf(buffer,"P[%d] = NaN\r\n",i);
		uart1_send_text(buffer);
	}
}
*/

//Calculating D = F_{k-1}*...*F_{k-N};
int i;
if (NSTORESTEPS == 1) { // Set to identity 
	for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
	{
		EKF_D[i]   = 0.0;  
	}	
	EKF_D[0] = 1.0;
	EKF_D[5] = 1.0;
	EKF_D[10] = 1.0;
	EKF_D[15] = 1.0;
} else if (NSTORESTEPS == 2) { // Copy F_{k-1}
	for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
	{
		EKF_D[i] = EKF_F[MSIZE_N*MSIZE_N*1 + i];  
	}
} else if (NSTORESTEPS == 3) {
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_D, &EKF_F[MSIZE_N*MSIZE_N*1], &EKF_F[MSIZE_N*MSIZE_N*2]); // D = F_{k-1}*F_{k-2}
} else if (NSTORESTEPS == 4) {
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP1, &EKF_F[MSIZE_N*MSIZE_N*2], &EKF_F[MSIZE_N*MSIZE_N*3]); // Temp1 = F_{k-2}*F_{k-3}
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_D, &EKF_F[MSIZE_N*MSIZE_N*1], EKF_TEMP1); // D = F_{k-1}*Temp1
} else {
	sprintf(buffer,"ERROR! Unsupported delay size (%d steps)\r\n",NSTORESTEPS);
	uart1_send_text(buffer);
}		



//Calculating K = D*P_p_old*H'*inv(H*P_p_old*H' + R);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H2); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_TEMP2, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R,MSIZE_N,MSIZE_R, EKF_TEMP1, EKF_H2, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_R,MSIZE_R,EKF_TEMP2, EKF_TEMP1, EKF_R2); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
//Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
if(Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2) != 0)   //; //Temp3 = inv(Temp2);
{
	sprintf(buffer,"TMP2 = [%f,%f; %f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}
//sprintf(buffer,"TMP3 = inv(TMP2) =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H2); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_R, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP2); // Temp1 = P*Temp2
//sprintf(buffer,"TMP1 = P*TMP2 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, EKF_D, EKF_TEMP1); // K = D*Temp1
//sprintf(buffer,"K = D*TMP1 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5],EKF_K[6],EKF_K[7]);uart1_send_text(buffer);

//Calculating x_m = x_p + K*y
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f; %f; %f; %f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
if  (SrcPosMeasCam_new[0] == 1)
{
	Tr_MatrixAdd (MSIZE_N,1,x_m, x_p, EKF_TEMP1); // x_m = x_p + Temp1;
	//sprintf(buffer,"x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);uart1_send_text(buffer);
} else
{
	Tr_MatrixAdd (MSIZE_N,1,EKF_TEMP2, x_p, EKF_TEMP1); // Temp2 = x_p + Temp1;
	//sprintf(buffer,"FAKE x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}

//Calculating P_m = P_p - K*H*P_p_old*D';
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H2); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP3, EKF_TEMP1, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)]); // Temp3 = Temp1*P
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_N, EKF_TEMP1, EKF_D); // Temp1 = D'
//sprintf(buffer,"TMP1 = D' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_TEMP3, EKF_TEMP1); // Temp2 = Temp3*Temp1
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);

if  (SrcPosMeasCam_new[0] == 1)
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P, EKF_P, EKF_TEMP2); // P = P - Temp2
	//sprintf(buffer,"P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);	
	SrcPosMeasCam_new[0] = 0;;
} else
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_TEMP1 ,EKF_P, EKF_TEMP2); // Temp1 = P - Temp2
	//sprintf(buffer,"FAKE P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
}

//SrcPosMeas_new[0] = 0;
flag_didMeasUp++; //This flag will be evaluated at the next call of EKF_Source function
//}

//Measurement Update 3: (d,alpha) of source 2 from camera
//---------------------------------------------------------------------------------------
//x_m from previous update (if any) is input to this update
x_p[0] = x_m[0];
x_p[1] = x_m[1];
x_p[2] = x_m[2];
x_p[3] = x_m[3];

//if (SrcPosMeas_new[0] == 1)
//{

z[0] = SrcPosMeasCam_d[1];
z[1] = SrcPosMeasCam_alpha[1];

//sprintf(buffer,"%f %f;\r\n",z[0],norm_angle(z[1])); uart1_send_text(buffer);
// Error
if (NSTORESTEPS == 1) {
	y[0] = z[0] - x_p[2];
	y[1] = norm_angle(z[1]  - x_p[3]);
} else {
	y[0] = z[0] - x_m[2 + MSIZE_N*(NSTORESTEPS-1)];
	y[1] = norm_angle(z[1]  - x_m[3 + MSIZE_N*(NSTORESTEPS-1)]);
}
//sprintf(buffer,"y = [%f; %f;]\r\n",y[0],y[1]); uart1_send_text(buffer);

// H has been declared in init_EKF_source() function
// H[0] = 1.0;

/*
int i;
for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
{
	if (isNaN(EKF_P[i]))
	{
		sprintf(buffer,"P[%d] = NaN\r\n",i);
		uart1_send_text(buffer);
	}
}
*/

//Calculating D = F_{k-1}*...*F_{k-N};
/* No need - same as for previous update
if (NSTORESTEPS == 1) { // Set to identity 
	for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
	{
		EKF_D[i]   = 0.0;  
	}	
	EKF_D[0] = 1.0;
	EKF_D[5] = 1.0;
	EKF_D[10] = 1.0;
	EKF_D[15] = 1.0;
} else if (NSTORESTEPS == 2) { // Copy F_{k-1}
	for (i = 0; i<(MSIZE_N*MSIZE_N); i++)
	{
		EKF_D[i] = EKF_F[MSIZE_N*MSIZE_N*1 + i];  
	}
} else if (NSTORESTEPS == 3) {
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_D, &EKF_F[MSIZE_N*MSIZE_N*1], &EKF_F[MSIZE_N*MSIZE_N*2]); // D = F_{k-1}*F_{k-2}
} else if (NSTORESTEPS == 4) { //Hardcoded delay of 3, because otherwise a big mess
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP1, &EKF_F[MSIZE_N*MSIZE_N*2], &EKF_F[MSIZE_N*MSIZE_N*3]); // Temp1 = F_{k-2}*F_{k-3}
	Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_D, &EKF_F[MSIZE_N*MSIZE_N*1], EKF_TEMP1); // D = F_{k-1}*Temp1
} else {
	sprintf(buffer,"ERROR! Unsupported delay size (%d steps)\r\n",NSTORESTEPS);
	uart1_send_text(buffer);
}		
*/


//Calculating K = D*P_p_old*H'*inv(H*P_p_old*H' + R);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H3); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_TEMP2, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R,MSIZE_N,MSIZE_R, EKF_TEMP1, EKF_H3, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_R,MSIZE_R,EKF_TEMP2, EKF_TEMP1, EKF_R3); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
//Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
if(Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2) != 0)   //; //Temp3 = inv(Temp2);
{
	sprintf(buffer,"TMP2 = [%f,%f; %f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}
//sprintf(buffer,"TMP3 = inv(TMP2) =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_H3); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_R, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP2); // Temp1 = P*Temp2
//sprintf(buffer,"TMP1 = P*TMP2 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R, EKF_K, EKF_D, EKF_TEMP1); // K = D*Temp1
//sprintf(buffer,"K = D*TMP1 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5],EKF_K[6],EKF_K[7]);uart1_send_text(buffer);

//Calculating x_m = x_p + K*y
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f; %f; %f; %f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
if  (SrcPosMeasCam_new[1] == 1)
{
	Tr_MatrixAdd (MSIZE_N,1,x_m, x_p, EKF_TEMP1); // x_m = x_p + Temp1;
	//sprintf(buffer,"x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);uart1_send_text(buffer);
} else
{
	Tr_MatrixAdd (MSIZE_N,1,EKF_TEMP2, x_p, EKF_TEMP1); // Temp2 = x_p + Temp1;
	//sprintf(buffer,"FAKE x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
}

//Calculating P_m = P_p - K*H*P_p_old*D';
Tr_MatrixMultiply (MSIZE_N,MSIZE_R,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H3); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP3, EKF_TEMP1, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)]); // Temp3 = Temp1*P
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_N, EKF_TEMP1, EKF_D); // Temp1 = D'
//sprintf(buffer,"TMP1 = D' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_TEMP3, EKF_TEMP1); // Temp2 = Temp3*Temp1
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);

if  (SrcPosMeasCam_new[1] == 1)
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P, EKF_P, EKF_TEMP2); // P = P - Temp2
	//sprintf(buffer,"P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);	
	SrcPosMeasCam_new[1] = 0;;
} else
{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_TEMP1 ,EKF_P, EKF_TEMP2); // Temp1 = P - Temp2
	//sprintf(buffer,"FAKE P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
}

//SrcPosMeas_new[0] = 0;
flag_didMeasUp++; //This flag will be evaluated at the next call of EKF_Source function
//}

//Measurement Update 4: (phi) of source 1 (Evader) from the other robot in the formation. Valid only for the leader. Source 2 is the Assistant.
//---------------------------------------------------------------------------------------
float phi_rob,sin_phi_rob,cos_phi_rob,x_meas,y_meas,h_phi,h_d_sq;
x_p[0] = x_m[0];
x_p[1] = x_m[1];
x_p[2] = x_m[2];
x_p[3] = x_m[3];
if (SrcPosMeasComm_new == 1)
{
for (i=0; i<3; i++)
{
//x_m from previous update (if any) is input to this update
z[0] = SrcPosMeasComm_x[i];
z[1] = SrcPosMeasComm_y[i];
//Expected measurement
phi_rob = norm_angle(x_m[1 + MSIZE_N*(NSTORESTEPS-1)] + (PI - x_m[3 + MSIZE_N*(NSTORESTEPS-1)])); //angle relative to the heading of the formation
sin_phi_rob = sin(phi_rob);
cos_phi_rob = cos(phi_rob);
//x_meas = FormPos_x[SrcPosMeasComm_new[i]] - FormPos_x[RobForm_id]; //coordinates of this robot
//y_meas = FormPos_y[SrcPosMeasComm_new[i]] - FormPos_y[RobForm_id]; //in the formation coordinate frame
//h_phi = atan2(x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob - y_meas, x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob - x_meas);
//h_d_sq = (x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob - x_meas)*(x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob - x_meas) + (x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob - y_meas)*(x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob - y_meas); //square of the expected distance between the communicating robot and the source
//sprintf(buffer,"[h_phi,h_d_sq] = [%.4f %.4f]; \r\n",h_phi,h_d_sq);uart1_send_text(buffer);
// Error
y[0] = z[0]  - x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob;
y[1] = z[1]  - x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob;

EKF_H4[0] =  cos_phi_rob;
EKF_H4[1] = -x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob;
EKF_H4[4] =  sin_phi_rob;
EKF_H4[5] =  x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob;
//derivatives d h_phi/d x and d h_phi/ d y
//EKF_H4[i] =  -(sin_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - y_meas)/h_d_sq;
//EKF_H4[3+i] = (cos_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - x_meas)/h_d_sq;

//derivatives d h_phi/d d and d h_phi/ d alpha
//EKF_H4[i*4 + 0] = (-(sin_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - y_meas)*cos_phi_rob + (cos_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - x_meas)*sin_phi_rob)/h_d_sq;
//EKF_H4[i*4 + 1] = (-(sin_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - y_meas)*(-x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*sin_phi_rob) + (cos_phi_rob*x_m[0 + MSIZE_N*(NSTORESTEPS-1)] - x_meas)*(x_m[0 + MSIZE_N*(NSTORESTEPS-1)]*cos_phi_rob))/h_d_sq;

//sprintf(buffer,"y = [%f; %f;]\r\n",y[0],y[1]); uart1_send_text(buffer);
//sprintf(buffer,"H4 = [%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f]\r\n",EKF_H4[0],EKF_H4[1],EKF_H4[2],EKF_H4[3],EKF_H4[4],EKF_H4[5],EKF_H4[6],EKF_H4[7]);uart1_send_text(buffer);

//EKF_H4[2] = 0.0;
//EKF_H4[3] = 0.0;

//Calculating K = D*P_p_old*H'*inv(H*P_p_old*H' + R);
Tr_MatrixTranspose (MSIZE_R4,MSIZE_N, EKF_TEMP1, EKF_H4); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%.3f,%.3f; %.3f,%.3f; %.3f,%.3f; %.3f,%.3f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R4, EKF_TEMP2, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f; %.3f,%.3f,%.3f,%.3f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8],EKF_TEMP2[9],EKF_TEMP2[10],EKF_TEMP2[11]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_R4,MSIZE_N,MSIZE_R4, EKF_TEMP1, EKF_H4, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%.3f,%.3f; %.3f,%.3f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
Tr_MatrixAdd (MSIZE_R4,MSIZE_R4,EKF_TEMP2, EKF_TEMP1, EKF_R4); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
//EKF_TEMP3[0] = 1.0/EKF_TEMP2[0];
Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
//Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
//sprintf(buffer,"TMP3 = inv(TMP2) = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_R4, MSIZE_N, EKF_TEMP1, EKF_H4); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_R4,MSIZE_R4, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R4, EKF_K, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)], EKF_TEMP2); // Temp1 = P*Temp2
//sprintf(buffer,"TMP1 = P*TMP2 =[%f,%f; %f,%f; %f,%f; %f,%f;]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_R4, EKF_K, EKF_D, EKF_TEMP1); // K = D*Temp1
//sprintf(buffer,"K = D*TMP1 =[%.3f,%.3f; %.3f,%.3f; %.3f,%.3f; %.3f,%.3f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5],EKF_K[6],EKF_K[7]);uart1_send_text(buffer);

//Calculating x_m = x_p + K*y
Tr_MatrixMultiply (MSIZE_N,MSIZE_R4,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f; %f; %f; %f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3]);uart1_send_text(buffer);
//if  (SrcPosMeasCam_new[1] == 1)
//{
	Tr_MatrixAdd (MSIZE_N,1,x_m, x_p, EKF_TEMP1); // x_m = x_p + Temp1;
//sprintf(buffer,"x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",x_m[0],x_m[1],x_m[2],x_m[3]);uart1_send_text(buffer);
//} else
//{
//	Tr_MatrixAdd (MSIZE_N,1,EKF_TEMP2, x_p, EKF_TEMP1); // Temp2 = x_p + Temp1;
	//sprintf(buffer,"FAKE x_m = x_p + TMP1 = [%f; %f; %f; %f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3]);uart1_send_text(buffer);
//}

//Calculating P_m = P_p - K*H*P_p_old*D';
Tr_MatrixMultiply (MSIZE_N,MSIZE_R4,MSIZE_N, EKF_TEMP1, EKF_K, EKF_H4); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP3, EKF_TEMP1, &EKF_P[MSIZE_N*MSIZE_N*(NSTORESTEPS-1)]); // Temp3 = Temp1*P
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (MSIZE_N,MSIZE_N, EKF_TEMP1, EKF_D); // Temp1 = D'
//sprintf(buffer,"TMP1 = D' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (MSIZE_N,MSIZE_N,MSIZE_N, EKF_TEMP2, EKF_TEMP3, EKF_TEMP1); // Temp2 = Temp3*Temp1
//sprintf(buffer,"TMP3 = TMP1*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);

//if  (SrcPosMeasCam_new[1] == 1)
//{
	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_P, EKF_P, EKF_TEMP2); // P = P - Temp2
	//sprintf(buffer,"P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);	

//} else
//{
//	Tr_MatrixSubtract (MSIZE_N,MSIZE_N, EKF_TEMP1 ,EKF_P, EKF_TEMP2); // Temp1 = P - Temp2
//	//sprintf(buffer,"FAKE P = P - TMP2 = [%f,%f; %f,%f]\r\n",EKF_P[0],EKF_P[1],EKF_P[2],EKF_P[3]); uart1_send_text(buffer);
//}

//SrcPosMeas_new[0] = 0;
flag_didCommUp++; //This flag will be evaluated at the next call of EKF_Source function
//}

}
SrcPosMeasComm_new = 0;
}

//sprintf(buffer,"%5.2f,%5.2f,%5.3f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.3f;\r\n",x_m[0],x_m[1],x_m[2],x_m[3],x_m[4],x_m[5],x_m[6],z[0],z[1]); uart1_send_text(buffer);

// Normalize angles and display on LEDs
x_m[1] = norm_angle(x_m[1]);
x_m[3] = norm_angle(x_m[3]);

led_show_angle_blink(x_m[3]);
led_show_angle(x_m[1]);

//sprintf(buffer,"x(k)=[%4.2f,%4.2f,%4.2f,%4.2f];\r\n",x_m[0+MSIZE_N*0],x_m[1+MSIZE_N*0],x_m[2+MSIZE_N*0],x_m[3+MSIZE_N*0]); uart1_send_text(buffer);
//sprintf(buffer,"x(k-1)=[%4.2f,%4.2f,%4.2f,%4.2f];\r\n",x_m[0+MSIZE_N*1],x_m[1+MSIZE_N*1],x_m[2+MSIZE_N*1],x_m[3+MSIZE_N*1]); uart1_send_text(buffer);
//sprintf(buffer,"x(k-2)=[%4.2f,%4.2f,%4.2f,%4.2f];\r\n",x_m[0+MSIZE_N*2],x_m[1+MSIZE_N*2],x_m[2+MSIZE_N*2],x_m[3+MSIZE_N*2]); uart1_send_text(buffer);
//sprintf(buffer,"x(k-3)=[%4.2f,%4.2f,%4.2f,%4.2f];\r\n",x_m[0+MSIZE_N*3],x_m[1+MSIZE_N*3],x_m[2+MSIZE_N*3],x_m[3+MSIZE_N*3]); uart1_send_text(buffer);

//sprintf(buffer,"d_est = %f, alpha_est = %f\r\n",SrcPosEst_d,SrcPosEst_alpha); uart1_send_text(buffer);
//sprintf(buffer,"%4.2f,%4.2f,%4.2f,%4.2f\r\n",x_m[0],x_m[1],x_m[2],x_m[3]); uart1_send_text(buffer);
}

void EKF_Calibration(int mic_id, unsigned int signal_id, float distMicToSpk, float amp)
{

float x_m[2], x_m_old[2], P_m[4];

if (mic_id == 0)
{
	x_m_old[0] = CalParamMic0_x0[signal_id];
	x_m_old[1] = CalParamMic0_x1[signal_id];
	P_m[0] = CalParamMic0_P[0];
	P_m[1] = CalParamMic0_P[1];
	P_m[2] = CalParamMic0_P[2];
	P_m[3] = CalParamMic0_P[3];
} else if (mic_id == 1)
{
	x_m_old[0] = CalParamMic1_x0[signal_id];
	x_m_old[1] = CalParamMic1_x1[signal_id];
	P_m[0] = CalParamMic1_P[0];
	P_m[1] = CalParamMic1_P[1];
	P_m[2] = CalParamMic1_P[2];
	P_m[3] = CalParamMic1_P[3];
} else if (mic_id == 2)
{
	x_m_old[0] = CalParamMic2_x0[signal_id];
	x_m_old[1] = CalParamMic2_x1[signal_id];
	P_m[0] = CalParamMic2_P[0];
	P_m[1] = CalParamMic2_P[1];
	P_m[2] = CalParamMic2_P[2];
	P_m[3] = CalParamMic2_P[3];
} else
{
	//Wrong mic id
}

//Step 1: Prediction

// No prediction update - we are estimating a constant

//Step 2: Correction by Measurement

//Inverse of distance is the measurement, amplitude is time variable model parameter
float z = 1.0/distMicToSpk; // Measurement

// Measurement model :
// h_k(x,w) = x1*amp + x0;

// Error: y = z - h_k(x_m);
float y = z - (x_m_old[1]*amp + x_m_old[0]);

// 1x2 Jacobian of measurement model w.r.t. states
Cal_H[0] = 1.0;
Cal_H[1] = amp;
// 1x1 Covariance matrix of measurement noise
Cal_R[0] = par_sigma_Calib*par_sigma_Calib;

//Calculating K = P_p*H'*inv(H*P_p*H' + R);
Tr_MatrixTranspose (1,2, EKF_TEMP1, Cal_H); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (2,2,1, EKF_TEMP2, P_m, EKF_TEMP1); // Temp2 = P*Temp1
//sprintf(buffer,"TMP2 = P*TMP1 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (1,2,1, EKF_TEMP1, Cal_H, EKF_TEMP2); // Temp1 = H*Temp2
//sprintf(buffer,"TMP1 = H*TMP2 = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixAdd (1,1,EKF_TEMP2, EKF_TEMP1, Cal_R); // Temp2 = Temp1 + R
//sprintf(buffer,"TMP2 = TMP1 + R =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
/*
	Tr_MatrixInverse3x3(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
	Tr_MatrixInverse2x2(EKF_TEMP3, EKF_TEMP2); //Temp3 = inv(Temp2);
*/
EKF_TEMP3[0] = 1.0/EKF_TEMP2[0]; //Temp3 = inv(Temp2);
//sprintf(buffer,"TMP3 = inv(TMP2) =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP3[0],EKF_TEMP3[1],EKF_TEMP3[2],EKF_TEMP3[3],EKF_TEMP3[4],EKF_TEMP3[5],EKF_TEMP3[6],EKF_TEMP3[7],EKF_TEMP3[8]);uart1_send_text(buffer);
Tr_MatrixTranspose (1,2, EKF_TEMP1, Cal_H); // Temp1 = H'
//sprintf(buffer,"TMP1 = H' = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (2,1,1, EKF_TEMP2, EKF_TEMP1, EKF_TEMP3); // Temp2 = Temp1*Temp3
//sprintf(buffer,"TMP2 = TMP1*TMP3 =[%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP2[0],EKF_TEMP2[1],EKF_TEMP2[2],EKF_TEMP2[3],EKF_TEMP2[4],EKF_TEMP2[5],EKF_TEMP2[6],EKF_TEMP2[7],EKF_TEMP2[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (2,2,1, EKF_K, P_m, EKF_TEMP2); // K = P*Temp2
//sprintf(buffer,"K = P*TMP2 =[%f,%f,%f; %f %f,%f;]\r\n",EKF_K[0],EKF_K[1],EKF_K[2],EKF_K[3],EKF_K[4],EKF_K[5]);uart1_send_text(buffer);

//Calculating x_m = x_m_old + K*y
//Tr_MatrixMultiply (2,1,1, EKF_TEMP1, EKF_K, y); // Temp1 = K*y
//sprintf(buffer,"TMP1 = K*y = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);

x_m[0] = x_m_old[0] + EKF_K[0]*y;
x_m[1] = x_m_old[1] + EKF_K[1]*y;

//Calculating P_m = P_p - K*H*P_p;
Tr_MatrixMultiply (2,1,2, EKF_TEMP1, EKF_K, Cal_H); // Temp1 = K*H
//sprintf(buffer,"TMP1 = K*H = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixMultiply (2,2,2, EKF_TEMP2, EKF_TEMP1, P_m); // Temp2 = Temp1*P
//sprintf(buffer,"TMP1 = TMP2*P = [%f,%f,%f; %f %f,%f; %f,%f,%f]\r\n",EKF_TEMP1[0],EKF_TEMP1[1],EKF_TEMP1[2],EKF_TEMP1[3],EKF_TEMP1[4],EKF_TEMP1[5],EKF_TEMP1[6],EKF_TEMP1[7],EKF_TEMP1[8]);uart1_send_text(buffer);
Tr_MatrixSubtract (2,2,P_m, P_m, EKF_TEMP2); // P = P - Temp2
//sprintf(buffer,"P = P+TMP1 = [%f,%f; %f,%f]\r\n",P_m[0],P_m[1],P_m[2],P_m[3]); uart1_send_text(buffer);

if (mic_id == 0)
{
	CalParamMic0_x0[signal_id] = x_m[0];
	CalParamMic0_x1[signal_id] = x_m[1];
	CalParamMic0_P[0] = P_m[0];
	CalParamMic0_P[1] = P_m[1];
	CalParamMic0_P[2] = P_m[2];
	CalParamMic0_P[3] = P_m[3];
} else if (mic_id == 1)
{
	CalParamMic1_x0[signal_id] = x_m[0];
	CalParamMic1_x1[signal_id] = x_m[1];
	CalParamMic1_P[0] = P_m[0];
	CalParamMic1_P[1] = P_m[1];
	CalParamMic1_P[2] = P_m[2];
	CalParamMic1_P[3] = P_m[3];
} else if (mic_id == 2)
{
	CalParamMic2_x0[signal_id] = x_m[0];
	CalParamMic2_x1[signal_id] = x_m[1];
	CalParamMic2_P[0] = P_m[0];
	CalParamMic2_P[1] = P_m[1];
	CalParamMic2_P[2] = P_m[2];
	CalParamMic2_P[3] = P_m[3];
} else
{
	//Wrong mic id
}

//sprintf(buffer,"Mic%d: x1 = %f, x0 = %f\r\n",mic_id,x_m[1],x_m[0]); uart1_send_text(buffer);
}

float GetPhaseShiftAngleMat(float* phase, float frequency)
{
if (phase[0]==INVALID_PHASE | phase[1]==INVALID_PHASE | phase[2]==INVALID_PHASE)
{
	return INVALID_PHASE;
}

//sprintf(buffer,"phase=[%f; %f; %f];\r\n", phase[0],phase[1],phase[2]); uart1_send_text(buffer);

if (isNaN(phase[0])) uart1_send_static_text("phase[0] = NaN\r\n");
if (isNaN(phase[1])) uart1_send_static_text("phase[1] = NaN\r\n");
if (isNaN(phase[2])) uart1_send_static_text("phase[2] = NaN\r\n");

float phaseShift20 = norm_angle(-(phase[0]-phase[2])); // [rad] Phase difference between Mic0 and Mic2; minus sign because phase is mirrored (as a result of taking fft of microphone measurement with oldest sample first)
float phaseShift21 = norm_angle(-(phase[1]-phase[2])); // [rad] Phase difference between Mic1 and Mic2;
//sprintf(buffer,"PS20=%f,PS21=%f\r\n",phaseShift20,phaseShift21); uart1_send_text(buffer);
float d[2];
d[0] = phaseShift20/(2.0*PI)*par_speedSound/frequency; //[m] distance traveled by wave front between hitting Mic0 and Mic2 (if negative -> Mic2 was hit first)
d[1] = phaseShift21/(2.0*PI)*par_speedSound/frequency; //[m] distance traveled by wave front between hitting Mic1 and Mic2 (if negative -> Mic2 was hit first)

float A_inv[] = {
   16.3934,   16.3934,
  -16.1290,   16.1290};

float n[2];

Tr_MatrixMultiply (2,2,1, n, A_inv, d); // n = A_inv*d;
//sprintf(buffer,"Phase=[%.3f,%.3f,%.3f]; n=[%.3f,%.3f];\r\n",phase[0],phase[1],phase[2],n[0],n[1]); uart1_send_text(buffer);
float res = atan2(n[1],n[0]);

if (isNaN(res))
{
	sprintf(buffer,"Invalid Phase! Phase=[%.3f,%.3f,%.3f]; n=[%.3f,%.3f];\r\n",phase[0],phase[1],phase[2],n[0],n[1]); uart1_send_text(buffer);
	return INVALID_PHASE;
} else {
	return res;
}
}

float GetDistMeasurement(unsigned int signal_id, float* amp, float angle)
{
// Given:
// - angle towards the target
// - three amplitudes
// Find best estimate of distance to the target

//if (isNaN(amp[0])) uart1_send_static_text("amp[0] = NaN\r\n");
//if (isNaN(amp[1])) uart1_send_static_text("amp[1] = NaN\r\n");
//if (isNaN(amp[2])) uart1_send_static_text("amp[2] = NaN\r\n");

// distance estimate for each microphone based on calibration parameters
float d_mic0 = 1.0/(CalParamMic0_x1[signal_id]*amp[0] + CalParamMic0_x0[signal_id]);
float d_mic1 = 1.0/(CalParamMic1_x1[signal_id]*amp[1] + CalParamMic1_x0[signal_id]);
float d_mic2 = 1.0/(CalParamMic2_x1[signal_id]*amp[2] + CalParamMic2_x0[signal_id]);

//if (isNaN(d_mic0)) uart1_send_static_text("d_mic0 = NaN\r\n");
//if (isNaN(d_mic1)) uart1_send_static_text("d_mic1 = NaN\r\n");
//if (isNaN(d_mic2)) uart1_send_static_text("d_mic2 = NaN\r\n");

//In theory, d_mic may not be smaller than sin(par_micPosA - angle)*par_micPosR, but the following is a more strict constraint (with sin=1)
if (d_mic0 < par_micPosR0) d_mic0=par_micPosR0; 
if (d_mic1 < par_micPosR1) d_mic1=par_micPosR1;
if (d_mic2 < par_micPosR2) d_mic2=par_micPosR2;

/*
float d0 = cosf(par_micPosA0 - angle)*par_micPosR0 + d_mic0*cosf(asinf(sinf(par_micPosA0 - angle)*par_micPosR0/d_mic0));
float d1 = cosf(par_micPosA1 - angle)*par_micPosR1 + d_mic1*cosf(asinf(sinf(par_micPosA1 - angle)*par_micPosR1/d_mic1));
float d2 = cosf(par_micPosA2 - angle)*par_micPosR2 + d_mic2*cosf(asinf(sinf(par_micPosA2 - angle)*par_micPosR2/d_mic2));
*/

float d0 = cosf(par_micPosA0 - angle)*par_micPosR0 + sqrt(d_mic0*d_mic0 - sinf(par_micPosA0 - angle)*sinf(par_micPosA0 - angle)*par_micPosR0*par_micPosR0);
float d1 = cosf(par_micPosA1 - angle)*par_micPosR1 + sqrt(d_mic1*d_mic1 - sinf(par_micPosA1 - angle)*sinf(par_micPosA1 - angle)*par_micPosR1*par_micPosR1);
float d2 = cosf(par_micPosA2 - angle)*par_micPosR2 + sqrt(d_mic2*d_mic2 - sinf(par_micPosA2 - angle)*sinf(par_micPosA2 - angle)*par_micPosR2*par_micPosR2);

//if (isNaN(d0)) uart1_send_static_text("d0 = NaN\r\n");
//if (isNaN(d1)) uart1_send_static_text("d1 = NaN\r\n");
//if (isNaN(d2)) uart1_send_static_text("d2 = NaN\r\n");

float res = (d0+d1+d2)/3.0;
return res;				                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
}

int CalibrateLineDrive(float distRobToSpkStart, float distRobToSpkFinish)
	{
	int i = 0;
	int mic_id = 0;
	int stop_id = 0;
	float CalDistTraveled,CalMeas,CalAngle,CalDist,distRobToSpk;
	e_set_steps_left(0); 		//reset step counter
	if (CalNStops == 0)						
		{
		if (distRobToSpkStart > distRobToSpkFinish)		
		{
			e_set_speed_left(CalMotorSpeed);				//driving towards the source
			e_set_speed_right(CalMotorSpeed);
		} else 
		{
			e_set_speed_left(-CalMotorSpeed);				//driving away from the source
			e_set_speed_right(-CalMotorSpeed);
		}
	}					
	do
	{
		CalDistTraveled = (float)e_get_steps_left()/(float)NStepsPerMeter;
		if (fabs(CalDistTraveled) >= fabs(distRobToSpkFinish-distRobToSpkStart))
		{ 
			break;					
		}
		//sprintf(buffer,"filled=%d;\r\n",e_ad_is_array_filled()); uart1_send_text(buffer);
		//sprintf(buffer,"ad_id=%d;\r\n",e_last_mic_scan_id); uart1_send_text(buffer);		
		CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]); //7th frequency is 902Hz
		CalAngle = 0.0;
		distRobToSpk = distRobToSpkStart-CalDistTraveled;
		if (CalComplete[par_signal_id[0]] == 0)
		{
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				//CalMeas = (float) e_get_micro_volume(mic_id);
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));
				EKF_Calibration(mic_id, par_signal_id[0], CalDist, CalMeas); //
				sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
		} else
		{
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));
				//sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
			sprintf(buffer,"%.4f,%.4f;\r\n",CalDist,GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]])); uart1_send_text(buffer);
			//sprintf(buffer,"%f,%f,%f,%f;\r\n",distRobToSpk,norm_angle(CalAngle),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[0]),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[1])); uart1_send_text(buffer);
			//sprintf(buffer,"3,%f,%f;\r\n",distRobToSpk,GetDistMeasurement(MicFreqMeasAmp, CalAngle)); uart1_send_text(buffer);
			//sprintf(buffer,"4,%f,%f;\r\n",norm_angle(CalAngle),GetPhaseShiftAngle(MicFreqMeasAmp)); uart1_send_text(buffer);
					}
		i++;

		if ((CalNStops != 0) & (i%CalNMeasPerStop == 0))
		{
			stop_id++;
			if (distRobToSpkStart > distRobToSpkFinish)		
			{
				e_set_speed_left(CalMotorSpeed);				//driving towards the source
				e_set_speed_right(CalMotorSpeed);
			} else 
			{
				e_set_speed_left(-CalMotorSpeed);				//driving away from the source
				e_set_speed_right(-CalMotorSpeed);
			}
			do
			{
				CalDistTraveled = (float)e_get_steps_left()/(float)NStepsPerMeter;
			} while (fabs(CalDistTraveled) < ((float)stop_id/(float)CalNStops)*fabs(distRobToSpkFinish-distRobToSpkStart));
			e_set_speed_left(0);
			e_set_speed_right(0);
			wait(500000);	
		} else if (CalNStops == 0)
		{
			//wait(100000);		
		}

	} while (1);
	e_set_speed_left(0);
	e_set_speed_right(0);		
	return i;
}

int CalibrateCircleDrive (float distRobToSpkStart, float diameterCircle)
{
// Robot is supposed to start at alpha=+pi/2 (i.e. source to the left from the robot) and at distance "distRobToSpkStart" from the source
// if diameter is positive, then the robot will drive on a circle turning left (i.e. towards the source, or even driving around the source if diameterCircle>distRobToSpkStart)
// if diameter is negative, then the robot will drive on a circle turning right (i.e. away from the source)
	int i = 0; //counter for measurement steps only
	int mic_id = 0;
	int stop_id = 0;
	float CalCircCovered,CalMeas,CalAngle,CalDist,distRobToSpk, x,y;
	e_set_steps_left(0); 				//reset step counter
	if (CalNStops == 0)
	{
		e_set_speed_left(CalMotorSpeed*(1.0-(par_b_wheels/diameterCircle)));				//set wheel speed to (500,-500)
		e_set_speed_right(CalMotorSpeed*(1.0+(par_b_wheels/diameterCircle)));
	}
	do
	{
		
		CalCircCovered = (float)e_get_steps_left()/(float)NStepsPerMeter/((diameterCircle-par_b_wheels)/2.0);
		if (fabs(CalCircCovered) >= 2.0*PI)
		{ 
			break;					
		}
		CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]); //7th frequency is 902Hz
		x = distRobToSpkStart + diameterCircle*(cos(CalCircCovered) - 1.0)/2.0;
		y = diameterCircle*sin(CalCircCovered)/2.0;
		CalAngle = atan2(y,x) + PI/2.0 - CalCircCovered;
		distRobToSpk= sqrt(x*x + y*y);
		if (CalComplete[par_signal_id[0]] == 0)
		{
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				//CalMeas = (float) e_get_micro_volume(mic_id);
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));					
				EKF_Calibration(mic_id, par_signal_id[0], CalDist, CalMeas);
				sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
		} else
		{
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));
				sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
			//sprintf(buffer,"%f,%f,%f,%f;\r\n",distRobToSpk,norm_angle(CalAngle),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[0]),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[1])); uart1_send_text(buffer);		
			//sprintf(buffer,"3,%f,%f;\r\n",distRobToSpk,GetDistMeasurement(MicFreqMeasAmp, CalAngle)); uart1_send_text(buffer);
			//sprintf(buffer,"4,%f,%f;\r\n",norm_angle(CalAngle),GetPhaseShiftAngle(MicFreqMeasAmp)); uart1_send_text(buffer);
			}
		i++;
		if ((CalNStops != 0) & (i%CalNMeasPerStop == 0))
		{
			stop_id++;
			e_set_speed_left(CalMotorSpeed*(1.0-(par_b_wheels/diameterCircle)));				//set wheel speed to (500,-500)
			e_set_speed_right(CalMotorSpeed*(1.0+(par_b_wheels/diameterCircle)));
			do
			{
				CalCircCovered = (float)e_get_steps_left()/(float)NStepsPerMeter/((diameterCircle-par_b_wheels)/2.0);
			} while (fabs(CalCircCovered) < ((float)stop_id/(float)CalNStops)*2.0*PI);
			e_set_speed_left(0);
			e_set_speed_right(0);
			wait(500000);	
		} else if (CalNStops == 0)
		{
			//wait(100000);		
		}
	} while (1);
	e_set_speed_left(0);
	e_set_speed_right(0);	
	return i;
}

int CalibrateRotating (float distRobToSpk)
{
//Step1 : Rotate around own axis at waypoint0
	int i = 0;
	int mic_id = 0;
	int nSteps=0;
	int stop_id = 0;
	float CalDistTraveled,CalMeas,CalAngle,CalDist;
	e_set_steps_left(0); 							//reset step counter
	if (CalNStops == 0)
	{
		e_set_speed_left(CalMotorSpeed);				//set wheel speed to (500,-500)
		e_set_speed_right(-CalMotorSpeed);
	}
	do
	{		
		nSteps = e_get_steps_left();
		if (nSteps >= NStepsPerRobRot)
		{ 
			break;					
		}
		CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
		CalAngle = 2.0*PI*(float)nSteps/(float)NStepsPerRobRot;
		if (CalComplete[par_signal_id[0]] == 0)
		{
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				//CalMeas = (float) e_get_micro_volume(mic_id);
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));					
				EKF_Calibration(mic_id, par_signal_id[0], CalDist, CalMeas);
				sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
		} else
		{
/*
			for (mic_id = 0; mic_id<3; mic_id++)
			{
				CalMeas= MicFreqMeasAmp[mic_id];
				CalDist= sqrt(powf(distRobToSpk-(par_micPosX[mic_id]*cosf(CalAngle) + par_micPosY[mic_id]*sinf(CalAngle)),2) + powf(-par_micPosX[mic_id]*sinf(CalAngle) + par_micPosY[mic_id]*cosf(CalAngle),2));
				sprintf(buffer,"%d,%f,%f;\r\n",mic_id,CalDist,CalMeas); uart1_send_text(buffer);
			}
*/
			
			sprintf(buffer,"%.4f,%.4f;\r\n",norm_angle(CalAngle),GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]])); uart1_send_text(buffer);
			//sprintf(buffer,"3,%f,%f;\r\n",distRobToSpk,GetDistMeasurement(MicFreqMeasAmp, CalAngle)); uart1_send_text(buffer);
			//sprintf(buffer,"4,%f,%f;\r\n",norm_angle(CalAngle),GetPhaseShiftAngle(MicFreqMeasPhase)); uart1_send_text(buffer);
			//sprintf(buffer,"%f,%f,%f,%f;\r\n",distRobToSpk,norm_angle(CalAngle),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[0]),norm_angle(MicFreqMeasPhase[2]-MicFreqMeasPhase[1])); uart1_send_text(buffer);
		}
		i++;
		if ((CalNStops != 0) & (i%CalNMeasPerStop == 0))
		{
			stop_id++;
			e_set_speed_left(CalMotorSpeed);				//set wheel speed to (500,-500)
			e_set_speed_right(-CalMotorSpeed);
			do
			{
				nSteps = e_get_steps_left();
			} while (nSteps < ((float)stop_id/(float)CalNStops)*NStepsPerRobRot);
			e_set_speed_left(0);
			e_set_speed_right(0);	
			wait(500000);
		} else if (CalNStops == 0)
		{
			//wait(100000);		
		}
	} while (1);
	e_set_speed_left(0);
	e_set_speed_right(0);	
	return i;
}

void FindInitSector (void)
{
int i, a0, a1; 
int MLSec0_id,MLSec1_id; //id of the sector with most measurements for signals 0 and 1
int SecCnt0[] = {0,0,0,0,0,0,0,0}; //Counters for measurements of signal 0 in one of the 8 sectors
int SecCnt1[] = {0,0,0,0,0,0,0,0}; //Counters for measurements of signal 1 in one of the 8 sectors
int NSamples;

NSamples = 200;
	for (i=0; i<NSamples; i++)
	{
		CaptureFFT(NSIG, &par_signal_id[0] ,&MicFreqMeasAmp[0],&MicFreqMeasPhase[0]);
		e_led_clear();

		if (SrcPosMeas_new[0] ==1)				
		{
			a0 = norm_angle(GetPhaseShiftAngleMat(&MicFreqMeasPhase[0],par_freqToDetect[par_signal_id[0]]));
			if (a0<-2.7489) //-(7/8)*PI
			{
				SecCnt0[4] = SecCnt0[4] + 1;
				e_set_led(4, 1);
			} else if (a0<-1.9635) //-(5/8)*PI
			{
				SecCnt0[3] = SecCnt0[3] + 1;
				e_set_led(3, 1);
			} else if (a0<-1.1781) //-(3/8)*PI
			{
				SecCnt0[2] = SecCnt0[2] + 1;
				e_set_led(2, 1);
			} else if (a0<-0.3927) //-(1/8)*PI
			{
				SecCnt0[1] = SecCnt0[1] + 1;
				e_set_led(1, 1);
			} else if (a0<0.3927) //+(1/8)*PI
			{
				SecCnt0[0] = SecCnt0[0] + 1;
				e_set_led(0, 1);
			} else if (a0<1.1781) //+(3/8)*PI
			{
				SecCnt0[7] = SecCnt0[7] + 1;
				e_set_led(7, 1);
			} else if (a0<1.9635) //+(5/8)*PI
			{
				SecCnt0[6] = SecCnt0[6] + 1;
				e_set_led(6, 1);
			} else if (a0<2.7489) //+(7/8)*PI
			{
				SecCnt0[5] = SecCnt0[5] + 1;
				e_set_led(5, 1);
			} else
			{
				SecCnt0[4] = SecCnt0[4] + 1;
				e_set_led(4, 1);
			}
		}

		if (SrcPosMeas_new[1] ==1)				
		{
			a1 = GetPhaseShiftAngleMat(&MicFreqMeasPhase[3],par_freqToDetect[par_signal_id[1]]);
			if (a0<-2.7489) //-(7/8)*PI
			{
				SecCnt1[4] = SecCnt1[4] + 1;
				//e_set_led(4, 2);
			} else if (a0<-1.9635) //-(5/8)*PI
			{
				SecCnt1[3] = SecCnt1[3] + 1;
				//e_set_led(3, 2);
			} else if (a0<-1.1781) //-(3/8)*PI
			{
				SecCnt1[2] = SecCnt1[2] + 1;
				//e_set_led(2, 2);
			} else if (a0<-0.3927) //-(1/8)*PI
			{
				SecCnt1[1] = SecCnt1[1] + 1;
				//e_set_led(1, 2);
			} else if (a0<0.3927) //+(1/8)*PI
			{
				SecCnt1[0] = SecCnt1[0] + 1;
				//e_set_led(0, 2);
			} else if (a0<1.1781) //+(3/8)*PI
			{
				SecCnt1[7] = SecCnt1[7] + 1;
				//e_set_led(7, 2);
			} else if (a0<1.9635) //+(5/8)*PI
			{
				SecCnt1[6] = SecCnt1[6] + 1;
				//e_set_led(6, 2);
			} else if (a0<2.7489) //+(7/8)*PI
			{
				SecCnt1[5] = SecCnt1[5] + 1;
				//e_set_led(5, 2);
			} else
			{
				SecCnt1[4] = SecCnt1[4] + 1;
				//e_set_led(4, 2);
			}
		}
	}
	MLSec0_id = find_max_id (&SecCnt0[0], 8);
	MLSec1_id = find_max_id (&SecCnt1[0], 8);
	if (meas_counter[0] > 20)
	{
		x_m[1] =  norm_angle((float)MLSec0_id*(PI/4.0));
		sprintf(buffer,"Signal 0 angle initialized: %d out of %d (%.2f%%) lie in sector %d\r\n",SecCnt0[MLSec0_id],meas_counter[0],(100.0*(float)SecCnt0[MLSec0_id]/(float)meas_counter[0]),MLSec0_id); uart1_send_text(buffer);
	} else {
		sprintf(buffer,"Signal 0 angle initialization failed: got less than 20 measurements (%d)\r\n",meas_counter[0]); uart1_send_text(buffer);
	}
	if (meas_counter[1] > 20)
	{
		x_m[3] =  norm_angle((float)MLSec1_id*(PI/4.0));
		sprintf(buffer,"Signal 1 angle initialized: %d out of %d (%.2f%%) lie in sector %d\r\n",SecCnt1[MLSec1_id],meas_counter[1],(100.0*(float)SecCnt1[MLSec1_id]/(float)meas_counter[1]),MLSec1_id); uart1_send_text(buffer);
	} else {
		sprintf(buffer,"Signal 1 angle initialization failed: got less than 20 measurements (%d)\r\n",meas_counter[1]); uart1_send_text(buffer);
	}
	meas_counter[0] = 0;
	meas_counter[1] = 0;
}

