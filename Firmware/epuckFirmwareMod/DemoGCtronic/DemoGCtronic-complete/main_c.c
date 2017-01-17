#include "p30f6014A.h"

#include "stdio.h"
#include "string.h"
#include "math.h"
#include <time.h>

#include <motor_led/e_init_port.h>
#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_motors.h>

#include <motor_led/advance_one_timer/e_agenda.h>
#include <codec/e_sound.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>


#include "e_led.h" //modified to include led_show_8bit(int) function
#include "uart/e_uart_char.h" //modified to allow variable baudrate for uart2


/* selector on normal extension*/

#define SELECTOR0 _RG6
#define SELECTOR1 _RG7
#define SELECTOR2 _RG8
#define SELECTOR3 _RG9


//#define MIC_SAMP_NB 100

#include "memory.h"
char buffer[BUFFER_SIZE];
int selector;
char c;

#include "utility.h"
#include "runcontroldrive.h"


//#define PI 3.14159265358979

#define uart2_send_static_text(msg) { e_send_uart2_char(msg,sizeof(msg)-1); while(e_uart2_sending()); }
#define uart1_send_static_text(msg) { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); }

int main() {

	//system initialization 
	e_init_port();    // configure port pins
	e_start_agendas_processing();
	e_init_motors();
	e_init_uart1();   // initialize UART to 115200 Kbaud
	e_init_uart2(BAUD115200);   // initialize UART to 115200 Kbaud
	e_init_ad_scan(MICRO_ONLY); // initialize AD converter to read microphones at 33kHz -> can't use Acc or Prox
	e_ad_scan_off(); //Turn scan off until we need it
	//Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR=0;
		__asm__ volatile ("reset");
	}

	// Decide upon program
	selector=getselector();
	sprintf(buffer, "Selector pos %d\r\n", selector);
	e_send_uart1_char(buffer, strlen(buffer));
	while(e_uart1_sending());


	//if (selector==0) {
		run_controldrive();
	//} else {
	//	run_breitenberg_shocker();
	//}

	while(1);
	return 0;
}

