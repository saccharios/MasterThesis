#include "CGridSearch.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;

CGridSearch::CGridSearch()
{
	AlgorithmSamplingTime = 2.5f; // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 4.0f; 
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms

}

CGridSearch::~CGridSearch()
{
}

bool CGridSearch::DriveToOrigin(CRobot* Robo, CRoboBlob* RoboBlob)
{
	CvMat* origin = cvCreateMat( 3, 1, CV_32FC1 );
	origin->data.fl[0] = 5;
	origin->data.fl[1] = 5;
	origin->data.fl[2] = 0;

	DriveFromToFast(Robo, RoboBlob, origin);
	WorldToTableCoord( RoboBlob->GetState(), origin);
	if(origin->data.fl[0] > 0 && origin->data.fl[0] < 10 && origin->data.fl[1] > 0 && origin->data.fl[1] < 10){return true;}else{return false;}

}

void CGridSearch::StartGridSearch()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
	hThread = CreateThread(NULL, 0, StaticGridSearchStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CGridSearch::StaticGridSearchStart(void* Param)
{// Author: Stefan Frei, 2013
	CGridSearch* This = (CGridSearch*) Param;
	return This->GridSearchRun();
}

DWORD CGridSearch::GridSearchRun()
{
	//if (ivRoboVect.size() != 1 || ivRoboBlobs.size() != 1 ){return -1;}

	algorithm_start = clock();
	int indexRobot = 0; //since we allow only one robot there is only one blob and the matching is easy
	int indexBlob = 0;
	CRobot* pTHERobot = ivRoboVect[indexRobot];
	CRoboBlob* pTHEBlob = ivRoboBlobs[indexBlob];
	int num_Robots = ivRoboVect.size();


	int num_DiscreteSamples = (int) (AlgorithmSamplingTime*1000.0f / InputSamplingTime);
	CvMat* Vin = cvCreateMat(num_Robots,num_DiscreteSamples,CV_32FC1);
	CvMat* Omegain = cvCreateMat(num_Robots,num_DiscreteSamples,CV_32FC1);

	pTHERobot->Stop(); // first command is not accepted by robot

	CvMat *currentState, *endState, *MaxState;
	currentState = cvCreateMat(3,1,CV_32FC1); 
	endState = cvCreateMat(3,1,CV_32FC1); 
	MaxState = cvCreateMat(3,1,CV_32FC1); 
	float Con, MaxCon = 0;

	

	float dx, dy;
	int sigma = 1; //inidcator if left or right line



	while(!ThStopFlag && !DriveToOrigin(pTHERobot,pTHEBlob) ); //drive very close to the origin

	pTHERobot->SetSamplingTime(AlgorithmSamplingTime);


	WorldToTableCoord( pTHEBlob->GetState(), currentState);
	
	float alpha;
	
	float ref_y = currentState->data.fl[1];
	float ref_x = currentState->data.fl[0] + StepLength*floor( ( WIDTH_TABLE-currentState->data.fl[0] )/StepLength);

	//cout << "1" << endl;

	while(!ThStopFlag && (ref_y <= HEIGHT_TABLE - position_noise ) )
	{
		start_time = clock();

		WorldToTableCoord( pTHEBlob->GetState(), currentState);

		WriteTrackStatestoFile(currentState);
		Con = MeasureConcentration( currentState);
		WriteContoFile(Con);
		if( Con > MaxCon)
		{
			MaxCon = Con;
			SetState(currentState, MaxState);
		}

		// The robot drive straight to the end point

		if( fabs(ref_x - currentState->data.fl[0]) > 2* StepLength)
		{
			alpha = atan2(ref_y - currentState->data.fl[1], 2* StepLength*sigma);
		}else
		{
			alpha = atan2(ref_y - currentState->data.fl[1], ref_x - currentState->data.fl[0]);
		}


		dx = StepLength*cosf(alpha);
		dy = StepLength*sinf(alpha);


		if (sigma == 1) //right
		{
			if( ref_x - currentState->data.fl[0] > StepLength/2 && currentState->data.fl[0] + StepLength < WIDTH_TABLE )// straight
			{
				SetState( currentState->data.fl[0] + dx , currentState->data.fl[1] + dy , alpha , endState);
			}else //turn
			{
				SetState( currentState->data.fl[0] , currentState->data.fl[1] + StepLength , PI , endState);

				ref_x = currentState->data.fl[0] - StepLength*floor( currentState->data.fl[0] / StepLength);
				ref_y = endState->data.fl[1];
				if( ref_y > HEIGHT_TABLE - position_noise){break;}
				cout << "turn left, new end point: " << ref_x << "  " << ref_y << endl;
				sigma = -1;
//				Sleep(200);
				//DriveFromToFast(pTHERobot, pTHEBlob, endState) ;
				//
				//continue;
			}
		}else if(sigma == -1) //left
		{
			if( currentState->data.fl[0] > ref_x + StepLength/2 && currentState->data.fl[0] - StepLength > 0)// straight
			{
				SetState( currentState->data.fl[0] + dx , currentState->data.fl[1] + dy , alpha , endState);
			}else //turn
			{
				SetState( currentState->data.fl[0] , currentState->data.fl[1] + StepLength , 0 , endState);

				ref_x = currentState->data.fl[0] + StepLength*floor(( WIDTH_TABLE - currentState->data.fl[0] )/StepLength);
				ref_y = endState->data.fl[1];
				if( ref_y > HEIGHT_TABLE - position_noise){break;}
				cout << "turn right, new end point: " << ref_x << "  " << ref_y << endl;
				sigma = 1;
//				Sleep(200);
				//DriveFromToFast(pTHERobot, pTHEBlob, endState) ;
				//
				//continue;
			}
		}

		if( ref_y > HEIGHT_TABLE - position_noise){break;}


		//pTHERobot->SendNavigateCommandN(currentState, endState);
		//Sleep((DWORD) AlgorithmSamplingTime*1000-100 ); // in [ms]
		//do
		//{
		//	elapsed_time = clock()-start_time;
		//}while( elapsed_time < AlgorithmSamplingTime*1000 + 150);

		//DriveFromToInTime(pTHERobot, pTHEBlob,endState);
		pTHERobot->SendNavigateCommandN(currentState, endState);
		DetermineDiscreteInputsInTime(indexRobot,pTHEBlob,endState, Vin, Omegain, num_DiscreteSamples);
		ApplyInputsEKFandWait(num_Robots, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);

		cout << "---------------------------------------------------------------------------------------" << endl;

		

	}

	cvReleaseMat(&Vin);
	cvReleaseMat(&Omegain);

	if ( !ThStopFlag )
	{
		algorithm_end = clock() - algorithm_start;
		cout << "Grid Search has terminated. Total Time: " << algorithm_end /1000.0f << " seconds "<<endl;
		cout << "Maximum at: " << MaxState->data.fl[0]<< " " <<MaxState->data.fl[1] << " with " << MaxCon << endl;
		WriteResultstoFile(MaxState, MaxCon, algorithm_end /1000.0f);
	}

	cvReleaseMat(&currentState);
	cvReleaseMat(&endState);
	pTHERobot->Stop();
	ExitThread( ThreadID );
	return 0;
}