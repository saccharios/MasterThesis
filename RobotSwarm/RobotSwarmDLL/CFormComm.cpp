#include "CFormComm.h"

//using namespace boost::numeric;
using namespace std;
static DWORD fcThreadId = NULL;

CFormComm::CFormComm(void)
{
	ThStopFlag = false;
	ThreadID = NULL;

	int i,j;

	for (i =0; i<7; i++)
	{
		for (j =0; j<7; j++)
		{
			Rob2Rob_d[i][j] = -100.0;
			Rob2Rob_alpha[i][j] = -100.0;
		}
	}
}


void CFormComm::CalcDistancesBetweenRobots (int NRobots)
// Calculate a matrix with distances between Robots
{
		int i,j;
		float x_i,y_i,x_j,y_j;	
	
		i = 0;
		j = 0;

		for (i =0; i<NRobots; i++)
		{
			x_i = ivRoboBlobs.at(i)->GetState()->data.fl[0];
			y_i = ivRoboBlobs.at(i)->GetState()->data.fl[1];
			for (j =0; j<NRobots; j++)
			{
				if (i<j)
				{
					x_j = ivRoboBlobs.at(j)->GetState()->data.fl[0];
					y_j = ivRoboBlobs.at(j)->GetState()->data.fl[1];

					Rob2Rob_d[i][j] = sqrt((x_j-x_i)*(x_j-x_i) + (y_j-y_i)*(y_j-y_i));

				} else if (i>j) {
					Rob2Rob_d[i][j] = Rob2Rob_d[j][i];
				} else { //i==j
					Rob2Rob_d[i][j] = 0;
				}
				//cout << "d" << i << j << " = " << Rob2Rob_d[i][j] << "; ";
			}
		}
		cout << endl;
}

void CFormComm::CalcAnglesBetweenRobots (int NRobots)
// Calculate a matrix with distances between Robots
{
		int i,j;
		float x_i,y_i,theta_i,x_j,y_j;	

		i = 0;
		j = 0;

		for (i =0; i<NRobots; i++)
		{
			x_i = ivRoboBlobs.at(i)->GetState()->data.fl[0];
			y_i = ivRoboBlobs.at(i)->GetState()->data.fl[1];
			theta_i = ivRoboBlobs.at(i)->GetState()->data.fl[2];
			for (j =0; j<NRobots; j++)
			{
				if (i!=j)
				{
					x_j = ivRoboBlobs.at(j)->GetState()->data.fl[0];
					y_j = ivRoboBlobs.at(j)->GetState()->data.fl[1];

					Rob2Rob_alpha[i][j] = atan2(-(y_j-y_i),(x_j-x_i)) - (PI - theta_i);
				} else { //i==j
					Rob2Rob_alpha[i][j] = 0;
				}
				//cout << "a" << i << j << " = " << Rob2Rob_alpha[i][j] << "; ";
			}
		}
		cout << endl;
}

void CFormComm::SetBlobsAndRobots( vector<CRoboBlob*> vRoboBlobs,vector<CRobot*> vRoboVect )
{
	ivRoboBlobs = vRoboBlobs;
	ivRoboVect = vRoboVect;
}

void CFormComm::StartAlgorithm()
{
	ThStopFlag = false;
    hThread = CreateThread(NULL, 0, StaticAlgorithmStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CFormComm::StaticAlgorithmStart(void* Param)
{
	CFormComm* This = (CFormComm*) Param;
    return This->AlgorithmRun();
}

vector<float> CFormComm::GetLastRobResponse(int Rob_id)
{
	vector <float> temp;
	if (Rob_id < ivRoboVect.size())
	{
		return ivRoboVect.at(Rob_id)->ReadLastRobPosUpdateResponse();
	} else {
		return temp;
	}
}


DWORD CFormComm::AlgorithmRun()
{
	int i,j;
	int NRobots;
	//int RobForm_pos[] = {8, 5, 4, 6, 2};
	int RobForm_pos[] = {8, 5, 2, 0, 4};
	int LeadForm_id[] = {0, 0, 0, 0, 2};
	int FolForm_id[] = {1, 1, 1, 1, 3};
	int TsComm = 300; //450; //[ms] Sampling time of communication (300ms works with 3 robots and onboard sampling time 0.15s)
	DWORD startTick, stopTick;
	j=0;
	while(!ThStopFlag)// flag to stop the thread, do something while thread runs, otherwise it will stop
	{	
		j++;
		NRobots = ivRoboBlobs.size();
		//cout << "NBlobs = " << NRobots << endl;

		CalcDistancesBetweenRobots (NRobots);
		CalcAnglesBetweenRobots (NRobots);

		startTick = GetTickCount();

		/* // Test: all keep same distance to robot 0
		for (i=0; i<NRobots; i++)
		{
			cout << "To Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << ")" << endl;
			ivRoboVect.at(i)->SendRobPosUpdateDA((Rob2Rob_d[i][0])/100-0.25,Rob2Rob_alpha[i][0],0,(float)j, form_pos[i]);
		}
		for (i=0; i<NRobots; i++)
		{
			LastRobResponse = ivRoboVect.at(i)->GetRobPosUpdateResponse();
			if (LastRobResponse.size() == 5)
			{
				cout << "Response: (" << LastRobResponse.at(0) << "," << LastRobResponse.at(1) << "," << LastRobResponse.at(2) << "," << LastRobResponse.at(3) << "," << LastRobResponse.at(4) << ")"<< endl;
			} else {
				cout << "Response: invalid size (" << LastRobResponse.size() << ")"<< endl;
			}
		} */

		 // Leader-Follower formation
		if (NRobots > 1)
		{
			//cout << "To Rob0(COM" << ivRoboVect.at(0)->GetComPort() << "): "<< endl << "Set Speed 350-300" << endl;
			//ivRoboVect.at(0)->SetMotorSpeed(350, 300);
			i=1;
			cout << "To Rob1(COM" << ivRoboVect.at(i)->GetComPort() << "): " << endl;
			ivRoboVect.at(i)->SendRobPosUpdateDA((Rob2Rob_d[i][LeadForm_id[i]])/100,Rob2Rob_alpha[i][LeadForm_id[i]], 0.0, 0.0, RobForm_pos[i]);
			LastRobResponse = ivRoboVect.at(i)->GetRobPosUpdateResponse();
			if (LastRobResponse.size() == 6)
			{
				cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): SP = [" << LastRobResponse.at(0) << "," << LastRobResponse.at(1) << "]; Src0 = [" << LastRobResponse.at(2) << "," << LastRobResponse.at(3) << "]; Src1 = [" << LastRobResponse.at(4) << "," << LastRobResponse.at(5) << "]"<< endl;
			} else {
				cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): invalid size (" << LastRobResponse.size() << ")"<< endl;
			}
			for (i=2; i<NRobots; i++)
			{
				cout << "To Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): " << endl;
				ivRoboVect.at(i)->SendRobPosUpdateDA((Rob2Rob_d[i][LeadForm_id[i]])/100,Rob2Rob_alpha[i][LeadForm_id[i]],(Rob2Rob_d[i][FolForm_id[i]])/100,Rob2Rob_alpha[i][FolForm_id[i]], RobForm_pos[i]);
				LastRobResponse = ivRoboVect.at(i)->GetRobPosUpdateResponse();
				if (LastRobResponse.size() == 6)
				{
					cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): SP = [" << LastRobResponse.at(0) << "," << LastRobResponse.at(1) << "]; Src0 = [" << LastRobResponse.at(2) << "," << LastRobResponse.at(3) << "]; Src1 = [" << LastRobResponse.at(4) << "," << LastRobResponse.at(5) << "]"<< endl;
				} else {
					cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): invalid size (" << LastRobResponse.size() << ")"<< endl;
				}
				//ivRoboVect.at(i)->SetMotorSpeed(450, 400);
			}

			/*
			for (i=1; i<NRobots; i++)
			{
				LastRobResponse = ivRoboVect.at(i)->GetRobPosUpdateResponse();
				if (LastRobResponse.size() == 6)
				{
					cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): SP = [" << LastRobResponse.at(0) << "," << LastRobResponse.at(1) << "]; Src0 = [" << LastRobResponse.at(2) << "," << LastRobResponse.at(3) << "]; Src1 = [" << LastRobResponse.at(4) << "," << LastRobResponse.at(5) << "]"<< endl;
				} else {
					cout << "Response Rob" << i << "(COM" << ivRoboVect.at(i)->GetComPort() << "): invalid size (" << LastRobResponse.size() << ")"<< endl;
				}
			}
			*/
		}
		

		/* //choo-choo train
		if (NRobots > 1)
		{
			ivRoboVect.at(0)->SetMotorSpeed(350, 300);
			for (i=1; i<NRobots; i++)
			{
				ivRoboVect.at(i)->SendRobPosUpdateDA((Rob2Rob_d[i][i-1])/100-0.1,Rob2Rob_alpha[i][i-1], 0.0, 0.0);
			}
		}
		*/
		stopTick = GetTickCount();
		cout << "Time: " << stopTick-startTick << " ms"<< endl;
		if (((stopTick-startTick))<=TsComm)
		{
			Sleep(TsComm-(stopTick-startTick)); //millisec 
		}
	}

	ExitThread( ThreadID );
	return 0; 

}