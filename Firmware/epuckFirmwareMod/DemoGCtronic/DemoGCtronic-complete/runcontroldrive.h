//#include "init_EKF.h" //For MSIZE_MAX

#ifndef _CONTROLDRIVE
#define _CONTROLDRIVE

void run_controldrive(void);
void Setpoint_Source (void);
void Setpoint_User (void);
void Setpoint_Triangle(void);
void Setpoint_Form (void);
void PID_Drive(float,float,int[2]);
void PID_Drive_Lead(float,float,int[2]);
void PID_Drive_Basic(float,float,int[2]);
void init_EKF_Source (void);
void EKF_Source(void);
void EKF_Calibration(int, unsigned int, float, float);
float GetDistMeasurement( unsigned int, float*, float);
//float GetPhaseShiftAngle(float*, float);
float GetPhaseShiftAngleMat(float*, float);
int CalibrateRotating (float);
int CalibrateLineDrive(float, float);
int CalibrateCircleDrive (float, float);
void FindInitSector (void);
int CaptureFFT(unsigned int, unsigned int*, float*, float*);
void BroadcastVolume(void);
void BroadcastSetpoint(void);
void BroadcastSrcPosEst(void);
void CtrlSetSpeed(void);
void PlaySignal(void);
void PlaySignal0(void);
void PlaySignal0_short(void);
void PlaySignal1(void);
void PlaySignal2(void);
void PlaySignal3(void);
void PlaySignal4(void);
/*
void Tr_MatrixMultiply(unsigned int, unsigned int, unsigned int, float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixAdd(unsigned int, unsigned int, float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixTranspose(unsigned int, unsigned int, float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixSubtractFromId (unsigned int, float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixInverse3x3(float[MSIZE_MAX*MSIZE_MAX], float[MSIZE_MAX*MSIZE_MAX]);
*/
#endif
