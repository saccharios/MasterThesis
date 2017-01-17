
#ifndef _DEVICE
#define _DEVICE

void run_device(void);


void PID_Drive(void);
void CtrlSetSpeed(void);

void update_pos(void);
float intervall_angle(float alpha);

void navigation(void);
//void navigationAFAP(void);
void Circle_Drive(void);
float normalize_angle(float alpha);


#endif
