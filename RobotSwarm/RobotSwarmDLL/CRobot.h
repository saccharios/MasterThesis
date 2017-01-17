#ifndef __CROBOT_H__
#define __CROBOT_H__

#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <conio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <CameraConfig.h>
#include <CRoboBlob.h>

using namespace std;

/*--THIS LIST IS DEPRECATED-------

"A"         Accelerometer
"B,#"       Body led 0=off 1=on 2=inverse
"C"         Selector position
"D,#,#"     Set motor speed left,right
"E"         Get motor speed left,right
"F,#"       Front led 0=off 1=on 2=inverse
"G"         IR receiver
"H"	     Help
"I"         Get camera parameter
"J,#,#,#,#,#,#" Set camera parameter mode,width,heigth,zoom(1,4 or 8),x1,y1
"K"         Calibrate proximity sensors
"L,#,#"     Led number,0=off 1=on 2=inverse
"N"         Proximity
"O"         Light sensors
"P,#,#"     Set motor position left,right
"Q"         Get motor position left,right
"R"         Reset e-puck
"S"         Stop e-puck and turn off leds
"T,#"       Play sound 1-5 else stop sound
"U"         Get microphone amplitude
"V"         Version of SerCom
"W"         Write I2C (mod,reg,val)
"Y"         Read I2C val=(mod,reg)
*/


const int MAXIMAL_SPEED = 2000;
const int MAX_INNER_WHEEL_SPEED = 500;





class CRobot
{

protected:
	string command;
	int iLeftSpeed;
	int iRightSpeed;
	int iLeftMotorPos;
	int iRightMotorPos;
	int iBodyLedState;
	int iFrontLedState;
	int iLEDnumber;
	int iLEDstate;
	int maxSpeed; //1000 correspond to 12.88cm/s
	int iSpeedMean;
	int iOmega;


	float speedFactor;
	DWORD timestampLastSetpoint;
	DWORD timestampLastClick;
	int priority;
	
	int iComPort;
	HANDLE hCom;
	DWORD dwBytesWrote;
	DWORD dwBytesRead;
	DCB dcb;
	BOOL bSuccess;
	COMMTIMEOUTS timeouts;
	char RoboBuffer[100];
	

	virtual void SetCom();
	string CutStringRoboBuf(string str = ""); //Provide smart output

	vector<float> vDebugValues;


public:
		
	CRobot();
	~CRobot();
	virtual bool Connect();
	virtual bool Disconnect();

	virtual void SetComPort(int ComPort);
	virtual int GetComPort();

	int GetVecIndex(vector<CRobot*> vRoboVect, int ComPort);

	virtual int Type(); //epuck = 0, khepera = 1, elisa = 2

	virtual void SetMotorSpeed(int iLeft, int iRight); //in Robot speed
	virtual void TurnRight();
	virtual void TurnLeft();
	virtual void SetPlaySound(int sound_id);
	virtual void SetSetpointDA(float SPd, float SPalpha);
	virtual void SetSetpointXY(float SPx, float SPy);
	virtual void SendRobPosUpdateXY(float RobPos_x, float RobPos_y);
	virtual void SendRobPosUpdateDA(float Rob1Pos_d, float Rob1Pos_alpha, float Rob2Pos_d, float Rob2Pos_alpha, int form_id);
	virtual vector<float> GetRobPosUpdateResponse();
	virtual vector<float> ReadLastRobPosUpdateResponse();
	void GetMotorSpeed();
	void SetTimestampLastSetpoint(DWORD t);
	DWORD GetTimestampLastSetpoint(void);
	void SetTimestampLastClick(DWORD t);
	DWORD GetTimestampLastClick(void);
	int SpeedConverter(float Speed); //converts real speed into "robot speed"
	int OmegaConverter(float Omega);

	void SetMotorPosition(string sCommand, int iLeft, int iRight);
	void GetMotorPosition(string sCommand);

	void SetPriority(int iPriority);
	int GetPriority();
	
	void SetBodyLED(int iState);
	void SetFrontLED(int iState);
	void SetLED(int iLED_Number, int iState);
	void GetLightSensors();

	void GetSelectorPosition();
	virtual void Calibrate();
	virtual vector<int> GetProximitySensorData();
	virtual void Accelerometer();
	void GetIR_Receiver();
	vector<int> GetMicroAmplitude();
	vector<float> GetDebugValuesRobot();
	void GetSercomVersion();
	bool GetBatteryLevel();

	void SetIC2();
	void GetIC2();
	

	virtual void Stop();
	virtual void Reset();
	void SteerManual(char RoboBuff[]);
	virtual void RandomWalk(bool bCollisionAvoidance);
	
	virtual void ObstacleAvoidanceOn();
	virtual void ObstacleAvoidanceOff();
	
	void SendCommand(string sCommand);
	void GetSpeedAndOmega(int* speed, int* omega);
	void GetInputsEKF(float* speed, float* omega);

	// User 2 specific functions for algorithms
	void SetSamplingTime(float Ts);
	void SendNavigateCommandN(CvMat* currentState, CvMat* endState);
	void SendNavigateCommandNAFAP(CvMat* currentState, CvMat* endState);
	void WaitUntilArrived(DWORD waitTimeapprox);





};

#endif // __CROBOT_H__

