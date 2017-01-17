#include "CEpuck.h"

void CEpuck::Calibrate()
{
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	bSuccess = WriteFile(hCom, "K\n", strlen("K\n"),&dwBytesWrote, NULL);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	
     
	string sRoboBuf;
	sRoboBuf = RoboBuffer;
	CutStringRoboBuf();

	Sleep(5000); //Wait for calibration response
	
	memset(RoboBuffer, 0, sizeof RoboBuffer);
	bSuccess = ReadFile(hCom, RoboBuffer, 100, &dwBytesRead, NULL);
	sRoboBuf = RoboBuffer;
	CutStringRoboBuf();
}

void CEpuck::ObstacleAvoidanceOn()
{
	WriteFile(hCom, "X\n", strlen("X\n"),&dwBytesWrote, NULL);
	
}

void CEpuck::ObstacleAvoidanceOff()
{
	WriteFile(hCom, "0\n", strlen("0\n"),&dwBytesWrote, NULL);
}