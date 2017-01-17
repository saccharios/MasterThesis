#include "CKhepera.h"
#include <iostream>
#include <sstream>

using namespace std;

//int matrix_prox[2][8] =	{{20,10,5,0,0,-5,-10,-20},{-20,-10,-5,0,0,5,20,}}; //{{32,16,8,0,0,-8,-16,-32},{-32,-16,-8,0,0,8,16,32}}; //{{8,4,2,0,0,-4,-8,-16},{-16,-8,-4,0,0,2,4,8}};
int matrix_prox[2][9] = {{ 2, -2, -4, -12,  5,  2,  2, 2, 4},{ 2,  2,  2, 5, -12, -4, -2, 2, 4}}; 

CKhepera::CKhepera()
{
	maxSpeed = 48000; //48000 correspond to 33.3cm/s
	speedFactor = maxSpeed/33.3f;
	for(int i = 0; i < 9; i++)
	{
		vProximity.push_back(0);
	}

}

CKhepera::~CKhepera()
{

}

void CKhepera::SetCom()
{

	dcb.BaudRate = 115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fInX = FALSE;
                    
    bSuccess = SetCommState(hCom, &dcb);
                   
                   
    timeouts.ReadIntervalTimeout=150;
    timeouts.ReadTotalTimeoutConstant=150;
    timeouts.ReadTotalTimeoutMultiplier=30;
    timeouts.WriteTotalTimeoutConstant=30;
    timeouts.WriteTotalTimeoutMultiplier=30;
	
	/*timeouts.ReadIntervalTimeout=40;
    timeouts.ReadTotalTimeoutConstant=1;
    timeouts.ReadTotalTimeoutMultiplier=1;
    timeouts.WriteTotalTimeoutConstant=1;
    timeouts.WriteTotalTimeoutMultiplier=1;*/

	bSuccess=SetCommTimeouts(hCom, &timeouts); 

}

void CKhepera::CollisionAvoidance()
{
	int potential[2];
	float speed[2];

	//vProximityData.clear();
	//GetProximitySensorData();
	int j = 4;
	//int matrix_prox[2][8] =	{{20,10,5,0,0,-5,-10,-20},{-20,-10,-5,0,0,5,20,}}; //{{32,16,8,0,0,-8,-16,-32},{-32,-16,-8,0,0,8,16,32}}; //{{8,4,2,0,0,-4,-8,-16},{-16,-8,-4,0,0,2,4,8}};
	//Prox.clear();

	vector<int> Prox;
	vector<int> vSpeed;

	Prox.push_back(vProximity[4]);
	Prox.push_back(vProximity[5]);
	Prox.push_back(vProximity[6]);
	Prox.push_back(vProximity[7]);
	Prox.push_back(vProximity[0]);
	Prox.push_back(vProximity[1]);
	Prox.push_back(vProximity[2]);
	Prox.push_back(vProximity[3]);

	int pot1, pot2;

	for (int m = 0; m < 2; m++)
	{
		potential[m] = 0;
		for (int s = 0; s < 9; s++)
		{
			/*if(j == 8)
			{
				j = 0;
			}*/
			potential[m] += (matrix_prox[m][s]*vProximity[s]); // get values from proximity sensors
			//j++;
		}
		if(m == 0)
		{
					pot1 = potential[0];
		}
		else
		{
					pot2 = potential[1];
		}

		//cout << "Potential: " << potential[m] << endl; 
        speed[m] = (float) (3*potential[m] + 30000);
		//cout << "speed: " << speed[m] << endl; 

		
		
		
	}

	if(abs(pot1) < 1000 && abs(pot2) < 1000)
		{
			speed[0] = 30000;
			speed[1] = 30000;

		}
		

		if(abs(potential[0]) > 2500)
		{
			speed[0] = speed[0]/2;
		}
		else if(abs(potential[1]) > 2500)
		{
			speed[1] = speed[1]/2;

		}

	/*if(Prox[0] + Prox[7] > 400)
	{
		speed[1] = speed[1]/2;
		speed[0] = speed[0]/2;

	}*/


	if((speed[1] < 50 && speed[1] > -50)
		&& (speed[0] < 50 && speed[0] > -50)) {
		speed[1] = speed[1] * 20;
		speed[0] = speed[0] * 20;
	}
	
	/*	if(abs(potential[0]) > 3000)
	{
		speed[0] = 0;
	}
	else if(abs(potential[1]) > 3000)
	{
		speed[1] = 0;
	}*/

	if (speed[1] > 24000)
		speed[1] = 24000;
	else if (speed[1] < -24000 )
		speed[1] = -24000;

	if (speed[0] > 24000)
		speed[0] = 24000;
	else if (speed[0] < -24000 )
		speed[0] = -24000;

	
	if(speed[1] > 1.5*speed[0]) 
	{
		speed[1] = speed[1]/2;
		speed[0] = speed[0]/2;
		//SetMotorSpeed("D",speed[1],speed[0]);
		//Sleep(1000);
	}
	else if(1.5 *speed[1] < speed[0])
	{
		speed[1] = speed[1]/2;
		speed[0] = speed[0]/2;
		//SetMotorSpeed("D",speed[1],speed[0]);
		//Sleep(1000);
	}

	//SetMotorSpeed("D",(int)speed[1],(int)speed[0]);
	
	//cout << endl;
	iLeftSpeed =  (int) (speed[1] / speedFactor); 
	iRightSpeed = (int) (speed[0] / speedFactor);

}

void CKhepera::SetMotorSpeed(int iLeft, int iRight)
	{
	stringstream sTemp;
	string sCommand;

	iLeftSpeed = SpeedConverter( (float) iLeft);
	iRightSpeed = SpeedConverter( (float) iRight);

	sTemp  << "D" << ",l" << iLeftSpeed << ",l" << iRightSpeed << "\n";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	//DWORD startTick = GetTickCount();
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	//DWORD endTick = GetTickCount();
	//DWORD diffTick = endTick - startTick;
	//cout << "diffTick: " << diffTick << endl;
	//cout << "bSuccess: " << bSuccess << endl;
	//ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	memset(RoboBuffer, 0, sizeof RoboBuffer);

	//might need some string cutting ...

	}

void CKhepera::Stop()
	{
		SetMotorSpeed(0,0);
	}

void CKhepera::Reset()
	{
		WriteFile(hCom, "M\n", strlen("M\n"),&dwBytesWrote, NULL);
		
	}

void CKhepera::RandomWalk(bool bCollisionAvoidance)
{
	if (bCollisionAvoidance)
	{
		CollisionAvoidance();
		//GetMotorSpeed();
		SetMotorSpeed(iLeftSpeed,iRightSpeed);
	}
	else
	{
		//GetMotorSpeed();
		SetMotorSpeed(13,13);
	}

}


void CKhepera::SetTargetProfile(int iLeft, int iRight)
{
	//SetMotorSpeed("F",iLeft,iRight);

	stringstream sTemp;
	string sCommand;
	iLeftSpeed = iLeft;
	iRightSpeed = iRight;

	sTemp << "F" << "," << iLeftSpeed << "," << iRightSpeed << "\n";
	//sTemp << "\0";
	sCommand = sTemp.str();
	//cout << sCommand;
	wstring w;
	bSuccess = WriteFile(hCom, sCommand.c_str(), strlen(sCommand.c_str()),&dwBytesWrote, NULL);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
}

void CKhepera::GetUS_PeriferalMeasure(int i)
{
	stringstream sTemp;
	string sCommand;

	sTemp << "G," << i;
	sCommand = sTemp.str();
	SendCommand(sCommand);
}

void CKhepera::GetBatteryState(int iMeasurement)
{
	stringstream sTemp;
	string sCommand;

	sTemp << "V," << iMeasurement;

	cout << "BatteryState: ";
	sCommand = sTemp.str();
	SendCommand(sCommand);
}

int CKhepera::Type()
{
	return 0;
}