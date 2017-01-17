#include "CMetropolisHastings.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;


CMetropolisHastings::CMetropolisHastings()
{
	AlgorithmSamplingTime = 2.5f; // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 4.0f; 
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms
	epsilon = 0.0001f;


	sigma = 360;

	binsize = 10; // bin size in [cm]

	
	N = (int) ceil(WIDTH_TABLE/binsize);
	M = (int) ceil(HEIGHT_TABLE/binsize);
	//cout << "N: " << N << " M: " << M  <<endl;

	old_Visits; /*= cvCreateMat(M, N, CV_32FC1);
	cvSetZero(old_Visits);*/
	Visits = cvCreateMat(M+1, N+1, CV_32FC1);
	cvSetZero(Visits );
	normed_Visits  = cvCreateMat(M+1, N+1, CV_32FC1);
	cvSetZero(normed_Visits);

}

CMetropolisHastings::~CMetropolisHastings()
{
}

void CMetropolisHastings::propState(CvMat* currentState, CvMat* proposeState)
{
	CvMat* state_pos = cvCreateMat( 2, 1, CV_32FC1 );
	CvMat* state_neg = cvCreateMat( 2, 1, CV_32FC1 );

	do
	{
		proposeState->data.fl[0] = SampleNormal(currentState->data.fl[0], sqrt(sigma) );
		proposeState->data.fl[1] = SampleNormal(currentState->data.fl[1], sqrt(sigma) );
		state_pos->data.fl[0] = proposeState->data.fl[0] + position_noise;
		state_pos->data.fl[1] = proposeState->data.fl[1] + position_noise;
		state_neg->data.fl[0] = proposeState->data.fl[0] - position_noise;
		state_neg->data.fl[1] = proposeState->data.fl[1] - position_noise;

	}while(!IsInField(state_pos) || !IsInField(state_neg));

	proposeState->data.fl[2] = atan2( proposeState->data.fl[1] - currentState->data.fl[1], proposeState->data.fl[0] - currentState->data.fl[0]);

	cvReleaseMat(&state_pos);
	cvReleaseMat(&state_neg);
}

void CMetropolisHastings::StartMetropolisHastings()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
	hThread = CreateThread(NULL, 0, StaticMetropolisHastingsStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CMetropolisHastings::StaticMetropolisHastingsStart(void* Param)
{// Author: Stefan Frei, 2013
	CMetropolisHastings* This = (CMetropolisHastings*) Param;
	return This->MetropolisHastingsRun();
}

DWORD CMetropolisHastings::MetropolisHastingsRun()
{
	//if (ivRoboVect.size() != 1 || ivRoboBlobs.size() != 1 ){return -1;}

	algorithm_start = clock();

	int indexRobot = 0; //since we allow only one robot there is only one blob and the matching is easy
	int indexBlob = 0;
	CRobot* pTHERobot = ivRoboVect[indexRobot];
	CRoboBlob* pTHEBlob = ivRoboBlobs[indexBlob];

	pTHERobot->Stop(); // first command is not accepted by robot
	float delta = 10.0f;
	float currentCon, proposeCon;

	CvMat* proposeState = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* currentState = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* discretePos = cvCreateMat( 2, 1, CV_8UC1);

	
	int itn = 0;
	N = 5000;

	while(!ThStopFlag && delta > epsilon)// flag to stop the thread, do something while thread runs, otherwise it will stop
	{	//discrete time loop

		start_time = clock();
		//old_Visits = cvCloneMat(normed_Visits);

		WorldToTableCoord( ivRoboBlobs[indexBlob]->GetState(), currentState);
		WriteTrackStatestoFile(currentState); // track the states, use this file for visualization and processing in Matlab

		currentCon =  MeasureConcentration( currentState );

		propState(currentState, proposeState);

		DriveFromToFast(ivRoboVect[indexRobot], ivRoboBlobs[indexBlob], proposeState);

		WorldToTableCoord( ivRoboBlobs[indexBlob]->GetState(), proposeState);
		WriteTrackStatestoFile(proposeState);

		proposeCon =  MeasureConcentration( proposeState );

		if ( SampleUniform(0.0f, 1.0f) <= proposeCon / currentCon ) //accept
		{
			//DiscretizePosition(N, M, binsize, proposeState, discretePos);
			WriteAcceptedStatestoFile(proposeState);
		}else //reject
		{
			DriveFromToFast(ivRoboVect[indexRobot], ivRoboBlobs[indexBlob], currentState );
			WorldToTableCoord( ivRoboBlobs[indexBlob]->GetState(), currentState);
			WriteTrackStatestoFile(currentState);
			WriteAcceptedStatestoFile(currentState);
			//DiscretizePosition(N, M, binsize, currentState, discretePos);
		}

		
		//cout << "discretePos: " << discretePos->data.i[0] << "  " << discretePos->data.i[1] << endl;
		//cvmSet(Visits, discretePos->data.i[1],discretePos->data.i[0], cvmGet(Visits,discretePos->data.i[1],discretePos->data.i[0]) + 1); // increments the visited discrete position, the indcies are interchanged 
		//cvConvertScale(Visits, normed_Visits, 1 , 0);
		//delta = (float) cvNorm(normed_Visits, old_Visits, CV_C,NULL);

		//cout << " Delta: " << delta << endl;
		cout << "---------------------------------------------------------------------------------------" << endl;

	}
	cout << "4" << endl;
	cvReleaseMat(&proposeState);
	cvReleaseMat(&currentState);
	cvReleaseMat(&discretePos);
	if ( !ThStopFlag )
	{
		algorithm_end = clock() - algorithm_start;
		cout << "Stochastic Localization has terminated. Total Time: " << algorithm_end /1000.0f << endl;
	}
	ExitThread( ThreadID );
	pTHERobot->Stop();
	return 0; 
}