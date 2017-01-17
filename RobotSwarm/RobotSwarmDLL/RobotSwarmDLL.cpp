//==================================//
//	THIS IS THE MAIN DLL FILE		//
//==================================//

#include <stdio.h> 
#include <direct.h>
#include <iostream>
#include <time.h>
#include "CameraConfig.h"
#include "CKhepera.h"
#include "CElisa.h"
#include "CEpuck.h"
#include "CCamera.h"
#include "CVirtualPotential.h"
#include <CAlgorithmSF.h>
#include <CStochasticLocalization.h>
#include <CGridSearch.h>
#include <CLineSearch.h>
#include <CMetropolisHastings.h>
#include <CSimulatedAnnealing.h>
#include <CFormComm.h>
#include <CEKFTest.h>


#include "../RobotSwarmGUI/bin/Debug/Algorithms/VirtualPotentialWalk.h"
#include "../RobotSwarmGUI/bin/Debug/Algorithms/SetpointByUser.h" 
#include "../RobotSwarmGUI/bin/Debug/Algorithms/SetpointByUserGlob.h" 
#include "../RobotSwarmGUI/bin/Debug/Algorithms/RunFormation.h" 

using namespace std;


#define DLLEXPORT extern "C" __declspec(dllexport)
#define STDCALL   __stdcall


CRobot BaseObject;
CvCapture* capture;
CCamera* Camera = new CCamera();

CVirtualPotential* VirtualPotential = new CVirtualPotential();
bool bCamReady = false;

static vector<CRobot*> vRoboVect;
static bool bFirstCapture = true;
static bool bFirstEntry = false;
static bool bComPortSet = false;

bool Walk(int comPort, bool bCollisionAvoidance = true);
bool GetSensorData(int comPort, vector<int> &vProximitySensor);
vector<CRoboBlob*> GetBlobsFromCam(int comPort);
int GetFirstBlob();
bool SetSpeed(float vLeft, float vRight, int comPort);
bool SetSetpointDA(float SPd, float SPalpha, int comPort);
bool SetSetpointXY(float SPx, float SPy, int comPort);
bool SendRobPosUpdate(float RobPosx, float RobPosy, int comPort);
bool SetPlaySound(int sound_id, int comPort);

vector<int> GetMicroVolume(int comPort);
vector<float> GetDebugValues(int comPort);

CStochasticLocalization* StochLoc = new CStochasticLocalization();
CGridSearch* GridSearch = new CGridSearch();
CLineSearch* LineSearch = new CLineSearch();
CMetropolisHastings* MetropolisHastings = new CMetropolisHastings();
CSimulatedAnnealing* SimulatedAnnealing = new CSimulatedAnnealing();
CEKFTest* EKFTest =  new CEKFTest();


CFormComm* FormComm = new CFormComm();

//**********************************//
//	EXPORT FUNCTIONS				//
//**********************************//

DLLEXPORT bool STDCALL Connect(int type, int comPort, int iPriority)
{

	switch(type)
	{
	case 0:
		{
			CKhepera *khepera = new CKhepera();
			khepera->SetComPort(comPort);
			if(khepera->Connect())
			{
				vRoboVect.push_back(khepera);
				khepera->SetPriority(iPriority);
				return true;
			}
			else
			{
				cout << "Connection failed!" << endl;
				return false;
			}
		}
	case 1:
		{
			CEpuck *epuck = new CEpuck();
			epuck->SetComPort(comPort);
			if(epuck->Connect())
			{
				epuck->Reset(); //Reset, otherwise epuck doesn't move before second walk command after turning it on
				//epuck->ObstacleAvoidance_On_Off(); // obstacle avoidance is on by default, so it is turned off here
				vRoboVect.push_back(epuck);
				epuck->SetPriority(iPriority);
				return true;
			}
			cout << "Connection failed!" << endl;
			return false;
		}
	case 2:
		{
			CElisa *elisa = new CElisa();
			if(elisa->Connect())
			{
				elisa->SetComPort(comPort);
				vRoboVect.push_back(elisa);
				elisa->SetPriority(iPriority);
				elisa->ObstacleAvoidance_On_Off();
				return true;
			}
			cout << "Connection failed!" << endl;
			return false;
		}
	default:
		{
			cout << "Connection failed!" << endl;
			return false;
		}
	}	
}

DLLEXPORT bool STDCALL Disconnect(int type, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	
	if (index >= 0)
	{
		if (type != 2)
		{
			if(vRoboVect[index]->Disconnect())
			{
				vRoboVect.erase(vRoboVect.begin() + index);
				return true;
			}
			else
			{
				return false;
			}
		}
		else //elisa
		{
			vRoboVect[index]->Stop(); //Can only disconnect the base station, not a single elisa
			return true;
		}
	} else{ return false;}
}

DLLEXPORT bool STDCALL Stop(int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);

	StochLoc->ThStopFlag = true; // Set the Thread Stop Flag to true, this will terminate the thread of StochLoc
	GridSearch->ThStopFlag = true;
	LineSearch->ThStopFlag = true;
	MetropolisHastings->ThStopFlag = true;
	SimulatedAnnealing->ThStopFlag = true;
	FormComm->ThStopFlag = true; 
	EKFTest->ThStopFlag = true; 

	if (index >= 0)
	{
		vRoboVect[index]->Stop();
		return true;
	}		
	else
	{
		return false;
	}	

}

DLLEXPORT bool STDCALL Calibrate(int comPort)
{
	
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		if(vRoboVect[index]->Type() == 1)
		{
			dynamic_cast<CEpuck*>(vRoboVect[index])->Calibrate();
		}
		else if(vRoboVect[index]->Type() == 2)
		{
			dynamic_cast<CElisa*>(vRoboVect[index])->Calibrate();			
		}
		
		return true;
	}
	else
	{
		return false;
	}
}

DLLEXPORT void STDCALL Exit()
{

	vector<int> iElisaIndex;

	for(size_t i = 0; i < vRoboVect.size(); i++)
	{
		if(vRoboVect[i]->Type() != 2)
		{
			vRoboVect[i]->Stop();
			vRoboVect[i]->Reset();
			vRoboVect[i]->Disconnect();
			delete vRoboVect[i];
		}
		else
		{
			iElisaIndex.push_back(i);
			vRoboVect[i]->Stop();
			vRoboVect[i]->Reset();
		}
	}
	if(iElisaIndex.size() > 0)
	{
		vRoboVect[iElisaIndex.front()]->Disconnect();
		for(size_t i = 0; i < iElisaIndex.size(); i++)
		{
			delete vRoboVect[iElisaIndex[i]]; 
		}
	}

	delete VirtualPotential;
	delete StochLoc;
	delete GridSearch;
	delete LineSearch;
	delete MetropolisHastings;
	delete SimulatedAnnealing;
}

DLLEXPORT int* STDCALL DiscoverElisa()
{
	
	CElisa* cElisa = new CElisa();
	vector<int> vElisa;

	const int arrayLength = 100;
	static int pElisa[arrayLength];

	if (cElisa->Connect())
	{
		vElisa = cElisa->LoadXMLFile();
		int vectorLength = vElisa.size();
	
		for (int i = 0; i < arrayLength; i++)
		{
			if (i >= vectorLength)
			{
				break;
			}
			pElisa[i] = vElisa[i];
		}
	}
		
	delete cElisa;
	return pElisa;
}

DLLEXPORT void STDCALL UpdateList(int comPort)
{
		Camera->vComPortPriority.push_back(comPort);
}

DLLEXPORT void STDCALL RemoveFromList(int index)
{
	Camera->vComPortPriority.erase(Camera->vComPortPriority.begin() + index);
}

DLLEXPORT void STDCALL Clear_vComPortPriority()
{
	Camera->vComPortPriority.clear();
}


// This is the function where the navigation is done
// Call your algorithms in this function
DLLEXPORT int* STDCALL Navigate(char* algName, int comPort, int type)
{

	//Types:
	//0: Khepera
	//1: Epuck
	//2: Elisa

	string sAlgName = algName;

	if(bFirstCapture)
	{
		capture = cvCaptureFromCAM(-1);
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH_PIX );
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT_PIX);

		//delete files and create new ones
		StochLoc->DeleteFiles();
		StochLoc->WriteParameterstoFile();

		int numberOfBlobs = 0;
		while(numberOfBlobs == 0)
		{
			cout << "GetFirstBlob" << endl;
			numberOfBlobs = GetFirstBlob();
			Sleep(200);
		}

	}
	Camera->vRoboBlobs = GetBlobsFromCam(comPort);
	
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	
	//Algorithm Calls

	if (sAlgName == "Walk")
	{
		if (type == 0)	//Khepera
		{
			vector<int> vProximitySensor;
			GetSensorData(comPort,vProximitySensor);
			//CRoboBlob::WriteSensorDataToFile(vProximitySensor);
		}
		if (type == 1) //Epuck
		{
			//int index = BaseObject.GetVecIndex(vRoboVect, comPort);
			if (index >= 0)
			{
				vRoboVect[index]->ObstacleAvoidanceOn();
				//vector<int> vProximitySensor;
			   // GetSensorData(comPort,vProximitySensor);
			}
		}
		Walk(comPort);
	}


	else if(sAlgName == "VirtualPotentialWalk")
	{
		
		//int index;
		if (type == 1) //Epuck
		{
			//index = BaseObject.GetVecIndex(vRoboVect, comPort);
			if (index >= 0)
			{
				vRoboVect[index]->ObstacleAvoidanceOff();
				/*vector<int> vMicroAmp = vRoboVect[index]->GetMicroAmplitude();
				for (int i=0; i < vMicroAmp.size(); i++)
				{
					cout << "vMicroAmp[" << i << "]: " << vMicroAmp[i] << endl;
				}
				cout << endl;*/
			}
		}

		//index = -1;

		vector<float> vVelocity;

		//index = BaseObject.GetVecIndex(vRoboVect, comPort);
		if (index >= 0)
		{

			vVelocity = VirtualPotentialWalk(VirtualPotential, Camera->vRoboBlobs, index);

			if(vVelocity[0] > MAX_SPEED)
			{
				vVelocity[0] = (float) MAX_SPEED;
			}
			else if(vVelocity[0] < 0)
			{
				vVelocity[0] = 0;
			}

			if(vVelocity[1] > MAX_SPEED)
			{
				vVelocity[1] = (float) MAX_SPEED;
			}
			else if(vVelocity[1] < 0)
			{
				vVelocity[1] = 0;
			}
			SetSpeed(vVelocity[0], vVelocity[1], comPort);
		}

	} 
	
	else if(sAlgName == "SetpointByUser")
	//Only supports ePucks
	{
		vector<float> vSetpoint;
		//int index = -1;

		//for (size_t i=0; i<Camera->vRoboBlobs.size(); i++)
		//{
		//	if (Camera->vRoboBlobs[i]->GetComPort() == comPort)
		//	{
		//		index = i;
		//		break;
		//	}
		//}
	
		if (index >= 0)
		{
			int xClick, yClick;
			DWORD newLastClickTick;
			//vector<int> MicroVol;
			Camera->GetLastClick(&xClick, &yClick, &newLastClickTick);
			//cout << "Click x: " << xClick << ", y: " << yClick << ", new: " << newClick <<endl;
			vSetpoint = SetpointByUser(xClick,yClick,Camera->vRoboBlobs, index);

			DWORD lastSetpointTick = vRoboVect[index]->GetTimestampLastSetpoint();
			DWORD lastClickTick = vRoboVect[index]->GetTimestampLastClick();
			DWORD nowTick = GetTickCount();
			DWORD diffTickLC = (newLastClickTick - lastClickTick);//Milliseconds since last click
			DWORD diffTickLS = (nowTick - lastSetpointTick); //Milliseconds since last setpoint sent
			//cout << "R"<< index << ": diffTickLC: " << diffTickLC << ", diffTickLS: " << diffTickLS <<endl;
			//cout << "R"<< index << ": newLC: " << newLastClickTick << ", oldLC: " << lastClickTick <<endl;
			if ((diffTickLS > TsSetpoint) || (diffTickLC > 0)) //Send setpoint after click and every TsSetpoint seconds
			//if (diffTickLC > 0) //Only send setpoint after click
			{				
				vRoboVect[index]->SetTimestampLastSetpoint(nowTick);	
				vRoboVect[index]->SetTimestampLastClick(newLastClickTick);
				SetSetpointDA(vSetpoint[0], vSetpoint[1], comPort);				
				/*
				if (vSetpoint[0] > 0.20)
				{
					SetPlaySound(4,comPort); //Play sound when driving
				} else if (vSetpoint[0] < 0.03)
				{
					SetPlaySound(0,comPort); //Stop playing when arrived close to target
				}
				*/
			}
			/*
			MicroVol = GetMicroVolume(comPort);
			cout << "Mic Amp: ";
			for (int i = 0; i < MicroVol.size(); i++)
			{
			cout << MicroVol[i] << " ";
			}
			*/
			//vector<float> vVelocity;
			//vVelocity.push_back(0.8*MAX_SPEED);
			//vVelocity.push_back(0.6*MAX_SPEED);
			//SetSpeed(vVelocity[0], vVelocity[1], comPort);

		}

	}

	else if(sAlgName == "SetpointByUserGlob")
	//Only supports ePucks
	{
		vector<float> vPosUpdate;
		//int index = -1;

		//for (size_t i=0; i<Camera->vRoboBlobs.size(); i++)
		//{
		//	if (Camera->vRoboBlobs[i]->GetComPort() == comPort)
		//	{
		//		index = i;
		//		break;
		//	}
		//}
	
		if (index >= 0)
		{
			int xClick, yClick;
			DWORD newLastClickTick;
			//vector<int> MicroVol;
			Camera->GetLastClick(&xClick, &yClick, &newLastClickTick);
			//cout << "Click x: " << xClick << ", y: " << yClick << ", new: " << newClick <<endl;
			vPosUpdate = SetpointByUserGlob(xClick,yClick,Camera->vRoboBlobs, index);

			DWORD lastSetpointTick = vRoboVect[index]->GetTimestampLastSetpoint();
			DWORD lastClickTick = vRoboVect[index]->GetTimestampLastClick();
			DWORD nowTick = GetTickCount();
			DWORD diffTickLC = (newLastClickTick - lastClickTick);//Milliseconds since last click
			DWORD diffTickLS = (nowTick - lastSetpointTick); //Milliseconds since last setpoint sent
			if (diffTickLS > TsSetpoint || diffTickLC > 0) //Send setpoint after click and every TsSetpoint seconds
			{				
				vRoboVect[index]->SetTimestampLastSetpoint(nowTick);	
				vRoboVect[index]->SetTimestampLastClick(newLastClickTick);
				SendRobPosUpdate(vPosUpdate[0],vPosUpdate[1],comPort);
				SetSetpointXY(vPosUpdate[2],vPosUpdate[3],comPort);
			}


		}

	}

	else if(sAlgName == "ObserveOnly")
	//Only supports ePucks
	{
		//if( Camera->vRoboBlobs.size() > 0)
		//{
		//		EKFTest->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); //if this statement is in the if below, it does not work with more than one robot

		//		if( EKFTest->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
		//			EKFTest->ThreadID = NULL;

		//		if ( EKFTest->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
		//			EKFTest->StartEKFTest();
		//	//}
		//}
	}

	else if(sAlgName == "StochasticLocalization")
		//Only supports ePucks
	{

		if( Camera->vRoboBlobs.size() > 0)
		{
			//cout << "ThStopFlag: " << StochLoc->ThStopFlag << "ThreadID: " << StochLoc->ThreadID << endl;
/*			if(Camera->vRoboBlobs.size() != vRoboVect.size())
			{
				cout << "ERROR: Emergency Stop!" << endl;
				StochLoc->ThStopFlag = true;

			} else
			{*/			
				StochLoc->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); //if this statement is in the if below, it does not work with more than one robot

				if( StochLoc->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
					StochLoc->ThreadID = NULL;

				if ( StochLoc->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
					StochLoc->StartStochasticLocalization();
			//}
		}

	}
	else if(sAlgName == "GridSearch")
	//Only supports ePucks
	{
		if( Camera->vRoboBlobs.size() > 0)
		{
			GridSearch->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); 

			if(  GridSearch->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
				 GridSearch->ThreadID = NULL;		

			if ( GridSearch->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
				GridSearch->StartGridSearch();

		}

	} 
	else if(sAlgName == "LineSearch")
	//Only supports ePucks
	{
		if( Camera->vRoboBlobs.size() > 0)
		{
			
			LineSearch->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); 

			if(  LineSearch->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
				 LineSearch->ThreadID = NULL;

			if ( LineSearch->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
				LineSearch->StartLineSearch();

		}

	} 
	else if(sAlgName == "MetropolisHastings")
		//Only supports ePucks
	{
		if( Camera->vRoboBlobs.size() > 0)
		{

			MetropolisHastings->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); 

			if( MetropolisHastings->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
				MetropolisHastings->ThreadID = NULL;


			if ( MetropolisHastings->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
				MetropolisHastings->StartMetropolisHastings();

		}

	} 
	else if(sAlgName == "SimulatedAnnealing")
	//Only supports ePucks
	{
		if( Camera->vRoboBlobs.size() > 0)
		{
			
			SimulatedAnnealing->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect);

			if( SimulatedAnnealing->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
				SimulatedAnnealing->ThreadID = NULL;

			if ( SimulatedAnnealing->ThreadID == NULL ) // only start the thread once, IMPROVE: instead of stopping and starting when pressing stop/start in GUI, only pause and resume the thread.
				SimulatedAnnealing->StartSimulatedAnnealing();
		}

	} 
	else if (sAlgName == "RunFormation")
	//Only supports ePucks
	{
		//vector<float> vSetpoint;
		//int index = -1;

		//for (size_t i=0; i<Camera->vRoboBlobs.size(); i++)
		//{
		//	if (Camera->vRoboBlobs[i]->GetComPort() == comPort)
		//	{
		//		index = i;
		//		break;
		//	}
		//}
			
		FormComm ->SetBlobsAndRobots(Camera->vRoboBlobs , vRoboVect); //update blobs and robots alsor for CStochasticLocalization, this process could be improved (not all information + functions are needed)

		if( FormComm->ThStopFlag == true) // if the thread was previously stopped, set flags to resume thread
		{
			FormComm->ThreadID = NULL;
		}
		if ( FormComm->ThreadID == NULL ) // only start the thread once
		{				
			FormComm->StartAlgorithm();
		}
	}
	
	//Add algorithms
	//
	//if (sAlgName == "NewAlgName")
	//{
	//
	//
	//	NewAlgName(comPort, Camera->vRoboBlobs, sensorData, ...)
	//
	//	vx = .., vy = ..
	//
	//	SetSpeed(vx, vy, comPort)
	//}
	//
	//...



	//May not be done before all cvwaitkeys are done!
	Camera->ShowImages(sAlgName);

	const int arrayLength = 30;
	static int pRemovedRoboBlobs[arrayLength];

	for (int i = 0; i < arrayLength; i++)
	{
		if (pRemovedRoboBlobs[i] == 0)
		{
			break;
		}

		pRemovedRoboBlobs[i] = 0;
	}
	
	for (size_t i = 0; i < arrayLength; i++)
		{
			if (i >= Camera->vRemovedRobots.size())
			{
				break;
			}
			pRemovedRoboBlobs[i] = Camera->vRemovedRobots[i]->GetComPort();
		}

	//cvReleaseCapture(&capture);	
	return pRemovedRoboBlobs;

}


bool Walk(int comPort, bool bCollisionAvoidance)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->RandomWalk(bCollisionAvoidance);
		return true;
	}
	else
	{
		return false;
	}
}

bool GetSensorData(int comPort, vector<int> &vProximitySensor)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);

	if(index >= 0)
	{
		if(vRoboVect[index]->Type() == 2)
		{
			vProximitySensor = dynamic_cast<CElisa*>(vRoboVect[index])->GetProximitySensorData();
			return true;
		}
		else
		{
			vProximitySensor = vRoboVect[index]->GetProximitySensorData();

			if(vRoboVect[index]->Type() == 0)
			{
				//dynamic_cast<CKhepera*>(vRoboVect[index])->vProximity.clear();
				//dynamic_cast<CKhepera*>(vRoboVect[index])->vProximity = vProximitySensor;

				//dynamic_cast<CKhepera*>(vRoboVect[index])->vProximity.clear();
				vector<int> vProx = dynamic_cast<CKhepera*>(vRoboVect[index])->vProximity;
				int minimum;
				if(vProximitySensor.size() < 9)
				{
					minimum = vProximitySensor.size();
				}
				else
				{
					minimum = 9;
				}

				for(int i = 0; i < minimum; i++)
				{
					vProx[i] = vProximitySensor[i];
				}

				dynamic_cast<CKhepera*>(vRoboVect[index])->vProximity = vProx;
			}

			return true;
		}
	}

	return false;

}

vector<CRoboBlob*> GetBlobsFromCam(int comPort)
{
	IplImage* camera_frm_resized;
	
	int index;


	if( capture ) 
	{
	
		IplImage* camera_frm ;
		camera_frm = cvQueryFrame( capture );

		if( camera_frm )
		{

			camera_frm_resized = cvCreateImage( cvSize( SHOW_XPIX, SHOW_YPIX ),8,3);  

			//cvResize(camera_frm , camera_frm_resized  , CV_INTER_LINEAR );
			//cvShowImage("Original", camera_frm_resized);

			if (!bFirstCapture)
			{
				for (size_t i=0; i < Camera->vTempBlobs.size(); i++)
				{
					Camera->vTempBlobs[i]->bLost = true; // Assume that all blobs are lost
				}

				Camera->ExtractBlobsEpuckOriBlack(camera_frm,vRoboVect);
			}
			else
			{
				bFirstCapture = false;
			}
					
			bool bComPortAssigned = false;
			for(size_t i = 0; i < Camera->vComPort.size(); i++)
			{
				if(Camera->vComPort[i] == comPort)
				{
					bComPortAssigned = true;
					break;
				}
			}

			for(size_t i = 0; i < Camera->vTempBlobs.size(); i++)
			{
				
				index = BaseObject.GetVecIndex(vRoboVect, Camera->vTempBlobs[i]->GetComPort()); 
				
				if(Camera->vRemovedRobots.size() > 0)
				{
					if (!bComPortAssigned && Camera->vTempBlobs[i]->GetComPort() == -1  && bComPortSet == false && Camera->vRemovedRobots[0]->GetComPort() == comPort)
					{
						bComPortSet = true;
						bFirstEntry = true;
						Camera->vTempBlobs[i]->SetComPort(comPort);
						Camera->vComPort.push_back(comPort);

					}
				}
				else
				{
					
					if (!bComPortAssigned && Camera->vTempBlobs[i]->GetComPort() == -1  && bComPortSet == false)
					{
						bComPortSet = true;
						bFirstEntry = true;
						Camera->vTempBlobs[i]->SetComPort(comPort);
						Camera->vComPort.push_back(comPort);

					}

				}

				if (Camera->vTempBlobs[i]->bLost) //Open loop
				{
					if(index > -1 )
					{
					if( EKFtype == 2 )
					{
						Camera->vTempBlobs[i]->KalmanPredict2(); 
					}else
					{	
						Camera->vTempBlobs[i]->KalmanPredict(); 
					}

					}//} else // the case when there is a blob but no robot
					//{
					//	Camera->vTempBlobs[i]->KalmanPredict();
					//}
					Camera->vTempBlobs[i]->IncrementStep();
					cout << "BLOB LOST "<< i <<" - Predict open loop " << Camera->vTempBlobs[i]->GetStep()  << endl;
				}	
				else
				{
					if(index > -1 )
					{

					if( EKFtype == 2 )
					{
						Camera->vTempBlobs[i]->KalmanPredict2(); 
						Camera->vTempBlobs[i]->KalmanUpdate2();
					}else
					{	
						Camera->vTempBlobs[i]->KalmanPredict(); 
						Camera->vTempBlobs[i]->KalmanUpdate();
					}
	
						
					}//} else // the case when there is a blob but no robot
					//{
					//	Camera->vTempBlobs[i]->KalmanPredict();
					//	Camera->vTempBlobs[i]->KalmanUpdate();
					//}

					Camera->vTempBlobs[i]->SetStep(0);
				}

				vector<int> fakeMicVal;
				switch (USER_NAME)
				{
					case 1:

					//With logging microphone measurements ("u" command)
					//---------------------------------------------------
					//CRoboBlob::WriteDataToFile(Camera->vTempBlobs[i]->GetComPort(),Camera->vTempBlobs[i]->GetMeasurement(),Camera->vTempBlobs[i]->GetState(), Camera->vTempBlobs[i]->GetCovariance(), GetMicroVolume(Camera->vTempBlobs[i]->GetComPort()));

					//With logging debug values ("v" command)
					//---------------------------------------------------
					CRoboBlob::WriteDebugDataToFile(Camera->vTempBlobs[i]->GetComPort(),Camera->vTempBlobs[i]->GetMeasurement(),Camera->vTempBlobs[i]->GetState(), Camera->vTempBlobs[i]->GetCovariance(), GetDebugValues(Camera->vTempBlobs[i]->GetComPort()), 14);

					// Without logging microphone measurements (fake values 0,0,0)
					//---------------------------------------------------
					//fakeMicVal.push_back(0); fakeMicVal.push_back(0); fakeMicVal.push_back(0);
					//CRoboBlob::WriteDataToFile(Camera->vTempBlobs[i]->GetComPort(),Camera->vTempBlobs[i]->GetMeasurement(),Camera->vTempBlobs[i]->GetState(), Camera->vTempBlobs[i]->GetCovariance(),fakeMicVal);

					//With logging comm response values ("i" command)
					//---------------------------------------------------
					//CRoboBlob::WriteDebugDataToFile(Camera->vTempBlobs[i]->GetComPort(),Camera->vTempBlobs[i]->GetMeasurement(),Camera->vTempBlobs[i]->GetState(), Camera->vTempBlobs[i]->GetCovariance(), FormComm->GetLastRobResponse(i), 6);
					
					
					break;

					case 2:
						if( EKFtype == 2 )
						{
							CRoboBlob::WriteDataToFile2(Camera->vTempBlobs[i]->GetComPort() , Camera->vTempBlobs[i]->GetMeasurement() , Camera->vTempBlobs[i]->GetState() , Camera->vTempBlobs[i]->GetCovariance(),Camera->vTempBlobs[i]->GetInput(),Camera->vTempBlobs[i]->GetPrediction());

						}else
						{	
							CRoboBlob::WriteDataToFile(Camera->vTempBlobs[i]->GetComPort() , Camera->vTempBlobs[i]->GetMeasurement() , Camera->vTempBlobs[i]->GetState2() , Camera->vTempBlobs[i]->GetCovariance());

						}
					break;

				}

				Camera->DrawExtractedBlob(Camera->rgbOutputImg, Camera->vTempBlobs[i]->GetState()); // Draw Heading Angle and Circle

				if (!bComPortAssigned && bFirstEntry)
				{
					if(Camera->vRemovedRobots.size() == 0)
					{
						cout << "inserted comport: " << comPort << endl;
						Camera->vRoboBlobs.push_back(Camera->vTempBlobs[i]);
						bFirstEntry = false;
					} else {

						if (comPort == Camera->vRemovedRobots[0]->GetComPort())
						{
							cout << "inserted comport: " << comPort << endl;
							Camera->vRoboBlobs.push_back(Camera->vTempBlobs[i]);

							if(Camera->vRemovedRobots.size() > 0)
							{ 
								Camera->vRemovedRobots.erase(Camera->vRemovedRobots.begin());
							}

							bFirstEntry = false;
						}

					}
							
				}

			}

		}

	}else
	{
		cout << "NO new pic" << endl;
	}
	

	cvReleaseImage(&camera_frm_resized );

	bComPortSet = false;
	return Camera->vRoboBlobs;
}

int GetFirstBlob()
{

	int numberOfBlobs = 0;

	if( capture ) 
	{
		IplImage* camera_frm;
		camera_frm = cvQueryFrame( capture );

		//cvSaveImage("Snapshot.jpg", camera_frm);

			if( camera_frm )
			{
				Camera->rgbOutputImg = cvCreateImage(cvGetSize( camera_frm ),8,3);
				//cvSet(Camera->rgbOutputImg,cvScalar(255,255,255));
				Camera->rgbOutputImg_resized = cvCreateImage( cvSize( SHOW_XPIX, SHOW_YPIX ),8,3);     

				numberOfBlobs = Camera->ExtractBlobsEpuckOriBlack(camera_frm,vRoboVect);
				//cout << "init nr of blobs " << numberOfBlobs << endl;
			}
	}
	return numberOfBlobs;			
}

bool SetSpeed(float vLeft, float vRight, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->SetMotorSpeed(vRoboVect[index]->SpeedConverter(vLeft), vRoboVect[index]->SpeedConverter(vRight));
		return true;
	}
	else
	{
		return false;
	}
}

bool SetSetpointDA(float SPd, float SPalpha, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->SetSetpointDA(SPd,SPalpha);
		cout << "Sending d: " << SPd << ", alpha: " << SPalpha << endl;
		return true;
	}
	else
	{
		return false;
	}
}

bool SetSetpointXY(float SPx, float SPy, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->SetSetpointXY(SPx,SPy);
		cout << "Sending SP x: " << SPx << ", y: " << SPy << endl;
		return true;
	}
	else
	{
		return false;
	}
}

bool SendRobPosUpdate(float RobPosx, float RobPosy, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->SendRobPosUpdateXY(RobPosx,RobPosy);
		cout << "Sending x: " << RobPosx << ", y: " << RobPosy << endl;
		return true;
	}
	else
	{
		return false;
	}
}

bool SetPlaySound(int sound_id, int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);
	if (index >= 0)
	{
		vRoboVect[index]->SetPlaySound(sound_id);
		return true;
	}
	else
	{
		return false;
	}
}

vector<int> GetMicroVolume(int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);		
	vector<int> MicroVol; //(3);
	//MicroVol[0] = 0;
	//MicroVol[1] = 0;
	//MicroVol[2] = 0;
	if (index >= 0)
	{
		vector<int> temp = vRoboVect[index]->GetMicroAmplitude();
		//if (temp.size()==3)
		//{
			MicroVol = temp;
		//}			
	}

	return MicroVol;
}

vector<float> GetDebugValues(int comPort)
{
	int index = BaseObject.GetVecIndex(vRoboVect, comPort);		
	vector<float> DebugVals;
	if (index >= 0)
	{
		vector<float> temp = vRoboVect[index]->GetDebugValuesRobot();
		DebugVals = temp;		
	}

	return DebugVals;
}