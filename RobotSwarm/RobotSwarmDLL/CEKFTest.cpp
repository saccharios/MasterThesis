#include "CEKFTest.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;


CEKFTest::CEKFTest()
{
	AlgorithmSamplingTime = 2.5f; // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 4.0f; 
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms


}

CEKFTest::~CEKFTest()
{
}

void CEKFTest::StartEKFTest()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
	hThread = CreateThread(NULL, 0, StaticEKFTestStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CEKFTest::StaticEKFTestStart(void* Param)
{// Author: Stefan Frei, 2013
	CEKFTest* This = (CEKFTest*) Param;
	return This->EKFTestRun();
}

DWORD CEKFTest::EKFTestRun()
{//Author: Stefan Frei 2013

	DWORD Tlag = 0;
	CRobot* pTHERobot = ivRoboVect[0];
	CRoboBlob* pTHEBlob = ivRoboBlobs[0];

		pTHERobot->SetMotorSpeed(1000, -1000);
		Sleep(Tlag);
		pTHEBlob->SetInput( 0.0f, -4.8604f);
		Sleep(20000-Tlag);

		pTHERobot->SetMotorSpeed(0, 0);
		Sleep(Tlag);
		pTHEBlob->SetInput( 0.0f, 0.0f);
		Sleep(2000-Tlag);


		pTHERobot->SetMotorSpeed(1000, -1000);
		Sleep(Tlag);
		pTHEBlob->SetInput( 0.0f, -4.8604f);
		Sleep(2000-Tlag);

		pTHERobot->SetMotorSpeed(0, 0);
		Sleep(Tlag);
		pTHEBlob->SetInput( 0.0f, 0.0f);
		Sleep(2000-Tlag);

		pTHERobot->SetMotorSpeed(1000, 1000);
		Sleep(Tlag);
		pTHEBlob->SetInput( 12.88f, 0.0f);
		Sleep(2000-Tlag);

		pTHERobot->SetMotorSpeed(0, 0);
		Sleep(Tlag);
		pTHEBlob->SetInput( 0.0f, 0.0f);
		Sleep(20000-Tlag);



	pTHERobot->Stop();
	ExitThread( ThreadID );
	return 0;
}