#include "CRobot.h"
#include <string> 
#include <sstream>
#include <ctime>

using namespace std;



CRobot::CRobot(){

dwBytesWrote = 0;
dwBytesRead;
iLeftSpeed = 0;
iRightSpeed = 0;

maxSpeed = 1000; //48000 correspond to 33.3cm/s
speedFactor = maxSpeed/12.88f;
DWORD timestampLastSetpoint = 0;
DWORD timestampLastClick = 0;

}

CRobot::~CRobot(){


}

bool CRobot::Connect()
{

 string sComPort = "\\\\.\\COM";

 stringstream ss;
   ss << iComPort;
 string sStr = ss.str();

 printf("Connecting...\n");


 sComPort.append(sStr);
 cout <<  sComPort<< endl;

					  hCom = CreateFile( sComPort.c_str(),
                      GENERIC_READ | GENERIC_WRITE,
                      0,      //  must be opened with exclusive-access
                      NULL,   //  default security attributes
                      OPEN_EXISTING, //  must use OPEN_EXISTING
                      0,      //  not overlapped I/O
                      NULL ); //  hTemplate must be NULL for comm devices
        
                   if (hCom == INVALID_HANDLE_VALUE) 
                   {
                       //  Handle the error.
                       printf ("CreateFile failed with error %d.\n", GetLastError());
					   return false;
				   }
                   else
				   {
					   //printf("Connection started\n"); 
					   cout << "Connected to ComPort: " << iComPort << endl;
					   SetCom();

					   if (!SetCommMask(hCom, EV_RXCHAR | EV_RXFLAG) ) 
					   {
						   // Handle the error. 
						   printf("SetCommMask failed with error %d.\n", GetLastError());
						   return false;
					   }

					   return true;
					   //epuck.SteerManual(hCom,dwBytesWrote,dwBytesRead, RoboBuff);

				   }
}

bool CRobot::Disconnect()
{

	bSuccess = CloseHandle(hCom);
	if (bSuccess==1)
    {
		//printf("Disconnected \n");
		cout << "Disconnected from ComPort: " << iComPort << endl;
		return true;           
    }
    else
    {
        printf("Disconnection unsuccessful!\n");
		return false;
    }
}

void CRobot::SetComPort(int ComPort)
{
	iComPort = ComPort;
}

int CRobot::GetVecIndex(vector<CRobot*> vRoboVect, int ComPort)
{


	for(size_t i = 0; i < vRoboVect.size(); i++)
	{
		if(vRoboVect[i]->GetComPort() == ComPort)
		{
			return i;
		}
	}

	//cout << "Nothing is connected to this COM port or no algorithm has been chosen!" << endl;
	return -1;
}

void CRobot::SetCom()
{

				   dcb.BaudRate = 9600;
				   //dcb.BaudRate = iBaudrate;
                   dcb.ByteSize = 8;
                   dcb.Parity = NOPARITY;
                   dcb.StopBits = ONESTOPBIT;

                   dcb.fDtrControl = DTR_CONTROL_DISABLE;
                   dcb.fInX = FALSE;
                    
                   bSuccess = SetCommState(hCom, &dcb);
                   
                  /*  bytesize = 8,
            baudrate = 115200,
            timeout = 1*/
                  /* timeouts.ReadIntervalTimeout=100;
                   timeouts.ReadTotalTimeoutConstant=100;
                   timeouts.ReadTotalTimeoutMultiplier=20;
                   timeouts.WriteTotalTimeoutConstant=20;
                   timeouts.WriteTotalTimeoutMultiplier=20;*/

				     /*timeouts.ReadIntervalTimeout         = 15;
  timeouts.ReadTotalTimeoutMultiplier  = 1;
  timeouts.ReadTotalTimeoutConstant    = 250;
  timeouts.WriteTotalTimeoutMultiplier = 1;
  timeouts.WriteTotalTimeoutConstant   = 250;*/

				   /* timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;//MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;//32000;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 100;*/
  
				   /*timeouts.ReadIntervalTimeout         = 250;//MAXDWORD;
				   timeouts.ReadTotalTimeoutMultiplier  = 200;
				   timeouts.ReadTotalTimeoutConstant    = 80;
                   timeouts.WriteTotalTimeoutMultiplier = 200;
                   timeouts.WriteTotalTimeoutConstant   = 250;*/
				   timeouts.ReadIntervalTimeout         = 40;//MAXDWORD;
				   timeouts.ReadTotalTimeoutMultiplier  = 0;
				   timeouts.ReadTotalTimeoutConstant    = 0;
                   timeouts.WriteTotalTimeoutMultiplier = 0;
                   timeouts.WriteTotalTimeoutConstant   = 0;
				   /*timeouts.ReadIntervalTimeout=200;
                   timeouts.ReadTotalTimeoutConstant=200;
                   timeouts.ReadTotalTimeoutMultiplier=40;
                   timeouts.WriteTotalTimeoutConstant=40;
                   timeouts.WriteTotalTimeoutMultiplier=40;*/
                   
                   bSuccess=SetCommTimeouts(hCom, &timeouts);       

}

int CRobot::Type()
{
	return 1;
}

string CRobot::CutStringRoboBuf(string str)
{
	string sRoboBuf;
	sRoboBuf = RoboBuffer;

	 if(RoboBuffer[0] == 'z')
	{ 
		sRoboBuf = sRoboBuf.substr(20);	
	}
	
	 /*int i = sRoboBuf.find_last_of("\n");
	 sRoboBuf = sRoboBuf.substr(0,i);*/

	 if(str.compare("") != 0)
	 {
		 int j = 0;
		 j = sRoboBuf.find(str);
		 sRoboBuf = sRoboBuf.substr(j+str.length(),sRoboBuf.length() - (2 + 1)); //first 2 entries are: "n," ; 2 because of '\n'; 1 because strings starts with index 0
	 }
	 //cout << sRoboBuf << endl;

	 return sRoboBuf;
	 /*strcpy(RoboBuffer,sRoboBuf.c_str());
	 RoboBuffer[i+1] = '\0';*/
}

void CRobot::SetPlaySound(int sound_id){ //See "t" command of ePuck to learn about different sound IDs
	
	//memset(RoboBuffer, 0, sizeof RoboBuffer);
	
	stringstream sTemp;
	string sCommand;

	sTemp << "T" << "," << sound_id << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	//bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//cout << "RoboBuffer: " << RoboBuffer << endl;



}

void CRobot::SetSetpointDA(float SPd, float SPalpha)
{
	stringstream sTemp;
	string sCommand;	

	sTemp << "G" << "," << SPd << "," << SPalpha << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	memset(RoboBuffer, 0, sizeof RoboBuffer);
}

void CRobot::SetSetpointXY(float SPx, float SPy)
{
	stringstream sTemp;
	string sCommand;	

	sTemp << "X" << "," << SPx << "," << SPy << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	memset(RoboBuffer, 0, sizeof RoboBuffer);
}

void CRobot::SendRobPosUpdateXY(float RobPos_x, float RobPos_y)
{
	stringstream sTemp;
	string sCommand;	

	sTemp << "Y" << "," << RobPos_x << "," << RobPos_y << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	memset(RoboBuffer, 0, sizeof RoboBuffer);
}

void CRobot::SendRobPosUpdateDA(float Rob1Pos_d, float Rob1Pos_alpha, float Rob2Pos_d, float Rob2Pos_alpha, int form_id)
{
	// Send relative position to another (two) robot(s); 
	// If (d,alpha)=(0,0), then it's not a measurement, i.e. 
	// SendRobPosUpdateDA(Rob1_d, Rob1_alpha, 0, 0) is interpreted as only a measurement of robot 1, but not robot 2
	stringstream sTemp;
	string sCommand;	

	sTemp << "I" << "," << Rob1Pos_d << "," << Rob1Pos_alpha << "," << Rob2Pos_d << "," << Rob2Pos_alpha << "," << form_id << "\n";
	cout << "Sending: R1 = (" << Rob1Pos_d << "," << Rob1Pos_alpha << "); R2 = (" << Rob2Pos_d << "," << Rob2Pos_alpha << "); " << endl;
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
}

vector<float> CRobot::GetRobPosUpdateResponse()
{
	// Get response to "i" command; This function only works after CRobot::SendRobPosUpdateDA()
	vDebugValues.clear();
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	string sDebVal = CutStringRoboBuf("i,");
	int j = 0;
	int k = 0;

	while(j != -1)
	{
		j = sDebVal.find(",");
		vDebugValues.push_back( (float) atof(sDebVal.substr(0,j).c_str()));
		k++;
		sDebVal = sDebVal.substr(j + 1,sDebVal.length() - (j + 1));
	}

	//memset(RoboBuffer, 0, sizeof RoboBuffer);
	return vDebugValues;
}

vector<float> CRobot::ReadLastRobPosUpdateResponse()
{
	// Read last stored response to "i" command;

	return vDebugValues;
}

void CRobot::SetTimestampLastSetpoint(DWORD t)
{
	this->timestampLastSetpoint = t;
}

DWORD CRobot::GetTimestampLastSetpoint(void)
{
	return this->timestampLastSetpoint;
}

void CRobot::SetTimestampLastClick(DWORD t)
{
	this->timestampLastClick = t;
}

DWORD CRobot::GetTimestampLastClick(void)
{
	return this->timestampLastClick;
}

void CRobot::TurnRight()
{
	SetMotorSpeed(SpeedConverter(12.88f),SpeedConverter(1.0f));
}

void CRobot::TurnLeft()
{
	SetMotorSpeed(SpeedConverter(1.0f),SpeedConverter(12.88f));
}

int CRobot::OmegaConverter(float Omega)
{
	int iConvertedOmega = (int) (speedFactor*Omega);
	if(iConvertedOmega > 2*maxSpeed)
	{
		iConvertedOmega = 2*maxSpeed;
	}else if(iConvertedOmega < -2*maxSpeed)
	{
		iConvertedOmega = -2*maxSpeed;
	}
	return iConvertedOmega;
}

int CRobot::SpeedConverter(float Speed)
{
	int iConvertedSpeed = (int)(speedFactor*Speed);

	if(iConvertedSpeed > maxSpeed)
	{
		iConvertedSpeed = maxSpeed;
	}
	else if(iConvertedSpeed < -maxSpeed)
	{
		iConvertedSpeed = -maxSpeed;
	}

	return iConvertedSpeed;

}

void CRobot::SetMotorSpeed(int iLeft, int iRight)// in robot speed (0 - 1000)
{ 

	iSpeedMean = (int) (iLeft + iRight) /2;
	iOmega =  (int) ( (iRight - iLeft) / WHEEL_DISTANCE_EPUCK) ;
	//memset(RoboBuffer, 0, sizeof RoboBuffer);
	
	stringstream sTemp;
	string sCommand;
	iLeftSpeed = iLeft;
	iRightSpeed = iRight;
	

	sTemp << "D" << "," << iLeftSpeed << "," << iRightSpeed << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	//wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()), &dwBytesWrote, NULL);
	//memset(RoboBuffer, 0, sizeof RoboBuffer);
	//bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//cout << "RoboBuffer: " << RoboBuffer << endl;



}

void CRobot::GetMotorSpeed(){

	memset(RoboBuffer, 0, sizeof RoboBuffer);

	//sComd.append("\n");
	string sCommand = "E\n";
	DWORD startTick = GetTickCount();

	
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);

	DWORD endTick = GetTickCount();
	DWORD diffTick = endTick - startTick;

	cout << "MotorSpeedTime: " << diffTick << endl;
	CutStringRoboBuf();

}

void CRobot::SetMotorPosition(string sCommand, int iLeft, int iRight) //Command "P,#,#" 
{
	//memset(RoboBuffer, 0, sizeof RoboBuffer);
	
	stringstream sTemp;
	
	iLeftMotorPos = iLeft;
	iRightMotorPos = iRight;

	sTemp << sCommand << "," << iLeft << "," << iRight << "\n";
	sCommand = sTemp.str();
	
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
}

void CRobot::GetMotorPosition(string sCommand) //Command "Q"
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);

	stringstream sTemp;

	sTemp << sCommand << "\n";
	sCommand = sTemp.str();

	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	cout << "MotorPosition: ";
	CutStringRoboBuf();
}

void CRobot::SetPriority(int iPriority)
{
	priority = iPriority;
}

int CRobot::GetPriority()
{
	return priority;
}

void CRobot::SetBodyLED(int iState) //Body led 0=off 1=on 2=inverse
{
	stringstream sTemp;
	string sCommand = "B";
	
	iBodyLedState = iState;

	sTemp << sCommand << "," << iBodyLedState << "\n";
	sCommand = sTemp.str();
	
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
}

void CRobot::SetFrontLED(int iState)
{
	stringstream sTemp;
	string sCommand = "F";
	
	iFrontLedState = iState;

	sTemp << sCommand << "," << iFrontLedState << "\n";
	sCommand = sTemp.str();
	
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
}

void CRobot::SetLED(int iLED_Number, int iState)
{
	stringstream sTemp;
	string sCommand = "L";
	
	iLEDnumber = iLED_Number;
	iLEDstate = iState;

	sTemp << sCommand << "," << iLEDnumber << "," << iLEDstate << "\n";
	sCommand = sTemp.str();
	
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
}

vector<int> CRobot::GetProximitySensorData()
{
	vector<int> vProximityData;
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	PurgeComm(hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);
	DWORD startTick = GetTickCount();
	bSuccess = WriteFile(hCom, "N\n", strlen("N\n"),&dwBytesWrote, NULL);	
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	DWORD endTick = GetTickCount();
	DWORD diffTick = endTick - startTick;

	cout << "SensorDiffTick: " << diffTick << endl;

	string sProxData = CutStringRoboBuf("n,");
	int j = 0;
	int k = 0;

	while(j != -1)
	{
		j = sProxData.find(",");
		vProximityData.push_back(atoi(sProxData.substr(0,j).c_str()));
		k++;
		sProxData = sProxData.substr(j + 1,sProxData.length() - (j + 1));
	}

	//bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);

	return vProximityData;

}

void CRobot::Accelerometer()
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "A\n", strlen("A\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	cout << "Accelerometer: ";
	CutStringRoboBuf();
}

void CRobot::GetIR_Receiver()
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "G\n", strlen("G\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	cout << "IR Receiver: ";
	CutStringRoboBuf();

}

vector<int> CRobot::GetMicroAmplitude()
{
	vector<int> vMicroAmplitude;
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "U\n", strlen("U\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//CutStringRoboBuf();

	string sMicroAmp = CutStringRoboBuf("u,");
	int j = 0;
	int k = 0;

	while(j != -1)
	{
		j = sMicroAmp.find(",");
		vMicroAmplitude.push_back(atoi(sMicroAmp.substr(0,j).c_str()));
		k++;
		sMicroAmp = sMicroAmp.substr(j + 1,sMicroAmp.length() - (j + 1));
	}
	return vMicroAmplitude;
}

vector<float> CRobot::GetDebugValuesRobot()
{
	vector<float> vDebugValues;
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "V\n", strlen("V\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//CutStringRoboBuf();

	string sDebVal = CutStringRoboBuf("v,");
	int j = 0;
	int k = 0;

	while(j != -1)
	{
		j = sDebVal.find(",");
		vDebugValues.push_back( (float) atof(sDebVal.substr(0,j).c_str()));
		k++;
		sDebVal = sDebVal.substr(j + 1,sDebVal.length() - (j + 1));
	}
	return vDebugValues;
}

void CRobot::GetSercomVersion()
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "V\n", strlen("V\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	cout << "Sercom Version: ";
	CutStringRoboBuf();
}

bool CRobot::GetBatteryLevel(void)
{//Author: Stefan Frei, 2013
	//returns true when battery level is high enough
	memset(RoboBuffer, 0, sizeof RoboBuffer);

	WriteFile(hCom, "b\n", strlen("b\n"),&dwBytesWrote, NULL);
	Sleep(100);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	string buf(RoboBuffer);

	if( buf.compare(0,3,"b,0",3) == 0) //when read 0, try again
	{
		//Sleep(100);
		//memset(RoboBuffer, 0, sizeof RoboBuffer);
		//WriteFile(hCom, "b\n", strlen("b\n"),&dwBytesWrote, NULL);
		//ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);

		//string buf(RoboBuffer);
		//cout << "2nd check: " << RoboBuffer << endl;
		//if( buf.compare(0,3,"b,0",3) == 0) // really no battery
		//{	
		//	return false;
		//}else{
		//	return true;
		//}
		return false;

	}	else // read 1
	{
		return true;
	}
	
}

void CRobot::GetIC2()
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	WriteFile(hCom, "Y\n", strlen("Y\n"),&dwBytesWrote, NULL);
	ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	cout << "IC2: ";
	CutStringRoboBuf();
}

void CRobot::SteerManual(char RoboBuff[])
{

	char Key=0;
	 //int iSpeedLeft = 0;
	 //int iSpeedRight = 0;


	 while(Key!=27)
	 {
			if(_kbhit()) {

				int iTempSpeed;
				Key = _getch();
                       
                       switch (Key) 
                       {
                       case 'w':
						   if(iLeftSpeed < MAXIMAL_SPEED && iRightSpeed < MAXIMAL_SPEED)
						   {
						    iLeftSpeed = iLeftSpeed + 10;
							iRightSpeed = iRightSpeed + 10;
						   }
							
							SetMotorSpeed(iLeftSpeed,iRightSpeed);
							
                            cout << "Forward: " << iLeftSpeed << ", " << iRightSpeed << endl; 
                            break;
                       
					   case 's':
						   if(iLeftSpeed > -MAXIMAL_SPEED && iRightSpeed > -MAXIMAL_SPEED)
							{
                            iLeftSpeed = iLeftSpeed - 10;
							iRightSpeed = iRightSpeed - 10;
						   }
							SetMotorSpeed(iLeftSpeed,iRightSpeed);
							
							cout << "Backwards: " << iLeftSpeed << ", " << iRightSpeed << endl; 
                            break;
                       
					   case 'a':
						   iTempSpeed = iLeftSpeed/2;
						    if(iTempSpeed > MAX_INNER_WHEEL_SPEED)
							{
								iTempSpeed = MAX_INNER_WHEEL_SPEED;
							}
						    SetMotorSpeed(iTempSpeed,iRightSpeed);
							cout << "Left: " << iTempSpeed << ", " << iRightSpeed << endl; 
                            break;
                       
					   case 'd':
						   iTempSpeed = iRightSpeed/2;
						    if(iTempSpeed > MAX_INNER_WHEEL_SPEED)
							{
								iTempSpeed = MAX_INNER_WHEEL_SPEED;
							}
                            SetMotorSpeed(iLeftSpeed,iTempSpeed);
							cout << "Right: " << iLeftSpeed << ", " << iTempSpeed << endl; 
                            break; 
					   case 'e':
						   GetMotorSpeed();
                                                                                                                                                 
                       default:  
                            break;     
                       }
                                          
					}   
	 
	 }       

}

void CRobot::RandomWalk(bool bCollisionAvoidance)
{
	SetMotorSpeed(SpeedConverter(12.88f),SpeedConverter(12.88f));
}

void CRobot::Stop()
{
	WriteFile(hCom, "S\n", strlen("S\n"),&dwBytesWrote, NULL);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//CutStringRoboBuf();
}

void CRobot::Reset()
{
	WriteFile(hCom, "R\n", strlen("R\n"),&dwBytesWrote, NULL);
	PurgeComm(hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

int CRobot::GetComPort()
{
	return iComPort;
}

void CRobot::ObstacleAvoidanceOn()
{

}

void CRobot::ObstacleAvoidanceOff()
{

}

void CRobot::SendCommand(string sCommand)
{
	stringstream sTemp;
	memset(RoboBuffer, 0, sizeof RoboBuffer);

	sTemp << sCommand << "\n";
	sCommand = sTemp.str();
	
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	CutStringRoboBuf();
	
}

void CRobot::Calibrate()
{

}

void CRobot::GetSpeedAndOmega(int* speed, int* omega)
{// Author: Stefan Frei, 2013
	*speed = iSpeedMean;
	*omega = iOmega;
}

void CRobot::SetSamplingTime(float Ts)
{// Author: Stefan Frei, 2013
	stringstream sTemp;
	string sCommand = "O";

	sTemp << sCommand << "," << Ts << "\n";
	sCommand = sTemp.str();
	
	//cout << "Send command: " << sCommand;
	WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()), &dwBytesWrote, NULL);
	//ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//cout << "read file: " << RoboBuffer << endl;
	Sleep(150);
}

void CRobot::SendNavigateCommandN(CvMat* currentState, CvMat* endState)
{// Author: Stefan Frei, 2013
	// in table coordinates
	stringstream sTemp;
	string sCommand = "N";

	currentState->data.fl[2] = fmod(currentState->data.fl[2], 2*PI);
	if (currentState->data.fl[2] < 0) {currentState->data.fl[2] += 2*PI;}
	endState->data.fl[2] = fmod(endState->data.fl[2], 2*PI);
	if (endState->data.fl[2] < 0) {endState->data.fl[2] += 2*PI;}



	sTemp << sCommand << "," << currentState->data.fl[0] << "," << currentState->data.fl[1] << "," << currentState->data.fl[2] << "," << endState->data.fl[0] << "," << endState->data.fl[1] << "," << endState->data.fl[2] << "\n";
	sCommand = sTemp.str();
	cout <<"Send command: " << sCommand;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	//ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//cout << "read file: " << RoboBuffer << endl;

	ofstream file;
	file.open("NavigateCommands.txt",ios::app|ios::out);		//open a file
	file << setprecision(6);
	file << currentState->data.fl[0] <<"\t" << currentState->data.fl[1] <<"\t" << currentState->data.fl[2] <<"\t" <<  endState->data.fl[0] <<"\t" << endState->data.fl[1] <<"\t" << endState->data.fl[2];
	file << endl;

	file.close();	
}

void CRobot::SendNavigateCommandNAFAP(CvMat* currentState, CvMat* endState)
{// Author: Stefan Frei, 2013
	// in table coordinates
	stringstream sTemp;
	string sCommand = "K";

	currentState->data.fl[2] = fmod(currentState->data.fl[2], 2*PI);
	if (currentState->data.fl[2] < 0) {currentState->data.fl[2] += 2*PI;}
	endState->data.fl[2] = fmod(endState->data.fl[2], 2*PI);
	if (endState->data.fl[2] < 0) {endState->data.fl[2] += 2*PI;}


	sTemp << sCommand << "," << currentState->data.fl[0] << "," << currentState->data.fl[1] << "," << currentState->data.fl[2] << "," << endState->data.fl[0] << "," << endState->data.fl[1] << "," << endState->data.fl[2] << "\n";
	sCommand = sTemp.str();
	cout <<"Send command: " << sCommand;

	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	//ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	//cout << "read file: " << RoboBuffer << endl;

}

void CRobot::WaitUntilArrived(DWORD waitTimeapprox)
{// Author: Stefan Frei, 2013
	
	//DWORD dwEvtMask;
	//int t = clock();
	//cout << "waitTimeapprox: " << waitTimeapprox +1000 << endl;
	Sleep( waitTimeapprox + 1000); // add 1000 ms for safety,

	//Waiting until flag from robot has arrived does work, unless you start and stop, then it breaks down
	//Problem: Cannot write and read at the same time! (epuck sends '1', and pc sends 'N,....')

	//WaitCommEvent(hCom, &dwEvtMask, NULL );
	//if(clock()-t < waitTimeapprox)
	//{
	//	cout << "Wait again" << endl;
	//	WaitCommEvent(hCom, &dwEvtMask, NULL );
	//}

	//do //problem when alread '1' in buffer from previous command
	//{
	//	WaitCommEvent(hCom, &dwEvtMask, NULL );
	//}while(dwEvtMask != 1 );

	//cout << " elapsed: " << clock()-t << endl; // " "<< dwEvtMask <<endl;


}

void CRobot::GetInputsEKF(float* speed, float* omega)
{//Author: Stefan Frei, 2013
	//gets inputs from epucks for EKF, in EKF-coordinates

	cout  << iLeftSpeed  << " " << iRightSpeed << endl;

	*speed = (iLeftSpeed + iRightSpeed)/2/speedFactor;
	*omega = ( iRightSpeed -iLeftSpeed )/WHEEL_DISTANCE_EPUCK/speedFactor;

}

