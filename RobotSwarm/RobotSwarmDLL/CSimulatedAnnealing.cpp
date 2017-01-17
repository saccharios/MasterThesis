#include "CSimulatedAnnealing.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;

CSimulatedAnnealing::CSimulatedAnnealing()
{//Author: Stefan Frei 2013
	AlgorithmSamplingTime = 2.5f; // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 4.0f; 
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms

	radius = 80; //80
	eta = 0.75f;//0.75
	T = 1.8f;//2
	gamma = 0.7f; //0.75

	N = 12;//14,12
	Nr = 4;// 7 (one less than in matlab)

	sigma = 0.75;

}

CSimulatedAnnealing::~CSimulatedAnnealing()
{
}

float CSimulatedAnnealing::AngleUDist(CvMat* currentState, float L)
{// Author: Stefan Frei, 2013
	// draws an angle such the vehicle stays inside (is on current state)

	float x_left, x_right, y_low, y_up, bound_left, bound_right;
	x_left = currentState->data.fl[0] - position_noise;
	x_right = WIDTH_TABLE - position_noise - currentState->data.fl[0] ;
	y_low = currentState->data.fl[1] - position_noise;
	y_up = HEIGHT_TABLE - position_noise - currentState->data.fl[1] ;
	


	//cout << "x_left: " << x_left << ", x_right: " << x_right << ", y_low: " << y_low << ", y_up: " << y_up << endl;
	if( x_left > -L && x_right > -L && y_low > - L && y_up > -L )
	{
		if (x_left>= L && x_right >= L && y_low >= L && y_up >= L)
		{// far away from border
			bound_left = 2*PI-0.0001f;
			bound_right = 0;
		} else if (x_left >= L && x_right >= L && y_low >= L && y_up < L)
		{// top
			bound_left = PI/2 - acosf(y_up / L);
			bound_right = PI/2 + acosf(y_up / L);
		} else if (x_left >= L && x_right >= L && y_low < L && y_up >= L)
		{// bottom
			bound_left = 3*PI/2 - acosf(y_low / L);
			bound_right = 3*PI/2 + acosf(y_low / L);
		} else if (x_left >= L && x_right < L && y_low >= L && y_up >= L)
		{// right
			bound_left = 2*PI - acosf(x_right / L);
			bound_right = acosf(x_right / L);
		} else if (x_left < L && x_right >= L && y_low >= L && y_up >= L)
		{// left
			bound_left = PI - acosf(x_left / L);
			bound_right = PI + acosf(x_left / L);
		} else if (x_left < L && x_right >= L && y_low < L && y_up >= L)
		{// corner bottom left
			bound_left = PI - acosf(x_left / L);
			bound_right = 3*PI/2 + acosf(y_low / L);
		} else if (x_left < L && x_right >= L && y_low >= L && y_up < L)
		{// corner top left
			bound_left = PI/2 - acosf(y_up / L);
			bound_right = PI + acosf(x_left / L);
		} else if (x_left >= L && x_right < L && y_low < L && y_up >= L)
		{// corner bottom right
			bound_left = 3*PI/2 - acosf(y_low / L);
			bound_right = acosf(x_right / L);
		} else if (x_left >= L && x_right < L && y_low >= L && y_up < L)
		{// corner top right
			bound_left = 2*PI - acosf(x_right / L);
			bound_right = PI/2 + acosf(y_up / L);
		}
	}else// cases when vehicle drove outside 
	{ 
		cout << "This case should not happen!" << endl;
		if(x_left < -L && y_up < -L)
		{
			return -PI/4;
		}else if(x_left < -L && y_low < -L)
		{
			return PI/4;
		}else if(x_left < -L)
		{
			return 0;
		}else if(y_up < -L)
		{
			return -PI/2;
		}else if(y_low < -L)
		{
			return PI/2;
		}else if(x_right < -L && y_up < -L)
		{
			return -3*PI/4;
		}else if(x_right < -L && y_low < -L)
		{
			return 3*PI/4;
		}else if(x_right)
		{
			return PI;
		}else {
			cout << "This case is not assigned!! " << x_left << ", " << x_right << ", " << y_up << ", " << y_low << endl;
			return 0;
		}
	}

	bound_left =  fmod(bound_left,2*PI);
	if( bound_left < 0){bound_left += 2* PI;}
	bound_right =  fmod(bound_right,2*PI);
	if( bound_right < 0){bound_right += 2* PI;}
	//cout << "bound_right: " << bound_right/PI*180 << ", bound_left: " << bound_left/PI*180;

	float propose_theta;


	if ( bound_right < bound_left)
	{
		propose_theta = bound_right + SampleUniform(0.0f, 1.0f)*(bound_left - bound_right);
	}else if( bound_left < bound_right)
	{
		propose_theta = bound_right + SampleUniform(0.0f, 1.0f)*(2*PI + bound_left - bound_right);
	}else{
		propose_theta = bound_left;
	}
	propose_theta = SetAngleInRangeZ2P(propose_theta);
	//cout <<" propose_theta: " << propose_theta/PI*180 << endl;

	return propose_theta;

}

void CSimulatedAnnealing::ProposePositionUnifDisc( CvMat* currentState, float L, CvMat* propose_new_pos)
{//Author: Stefan Frei 2013

	float theta = AngleUDist(currentState, L);
	propose_new_pos->data.fl[0] = currentState->data.fl[0] + L*cosf(theta);
	propose_new_pos->data.fl[1] = currentState->data.fl[1] + L*sinf(theta);
	propose_new_pos->data.fl[2] = SetAngleInRangeZ2P(theta);// - PI);
}

void CSimulatedAnnealing::ProposePosition_Bias_Disc(float L, float sigma, CvMat* last_acc_state, CvMat* before_acc_state, float last_acc_con, float before_acc_con, CvMat* propose_new_pos)
{//Author: Stefan Frei 2013

	CvMat *s1, *s2; //, *LastRejectState;
	s1 = cvCreateMat(3,1,CV_32FC1); 
	s2 = cvCreateMat(3,1,CV_32FC1);

	float dx, dy, theta, phi;

	dx = before_acc_state->data.fl[0] - last_acc_state->data.fl[0];
	dy = before_acc_state->data.fl[1] - last_acc_state->data.fl[1];

	theta = atan2(dy,dx);

	if(last_acc_con > before_acc_con) // bias angle away from last pos if con higher
		theta += PI;

	int i = 0;
	do
	{
		phi = SetAngleInRangeZ2P(SampleNormal(theta, sigma));
		propose_new_pos->data.fl[0] = last_acc_state->data.fl[0] + L*cosf(phi);
		propose_new_pos->data.fl[1] = last_acc_state->data.fl[1] + L*sinf(phi);

		s1->data.fl[0] = propose_new_pos->data.fl[0] + position_noise;
		s1->data.fl[1] = propose_new_pos->data.fl[1] + position_noise;
		s2->data.fl[0] = propose_new_pos->data.fl[0] - position_noise;
		s2->data.fl[1] = propose_new_pos->data.fl[1] - position_noise;

		i++;
	}while(i < 10 && (!IsInField(s1) || !IsInField(s2) ) );

	propose_new_pos->data.fl[2] = SetAngleInRangeZ2P(phi);// - PI);

	if( i > 9)
	{
		while(!IsInField(propose_new_pos))
		{
			ProposePositionUnifDisc( last_acc_state, radius, propose_new_pos);
		}
	}
	

	cvReleaseMat(&s1);
	cvReleaseMat(&s2);

}

void CSimulatedAnnealing::StartSimulatedAnnealing()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
	hThread = CreateThread(NULL, 0, StaticSimulatedAnnealingStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CSimulatedAnnealing::StaticSimulatedAnnealingStart(void* Param)
{// Author: Stefan Frei, 2013
	CSimulatedAnnealing* This = (CSimulatedAnnealing*) Param;
	return This->SimulatedAnnealingRun();
}

DWORD CSimulatedAnnealing::SimulatedAnnealingRun()
{//Author: Stefan Frei 2013
		//if (ivRoboVect.size() != 1 || ivRoboBlobs.size() != 1 ){return -1;}

	algorithm_start = clock();
	int indexRobot = 0; //since we allow only one robot there is only one blob and the matching is easy
	int indexBlob = 0;
	CRobot* pTHERobot = ivRoboVect[indexRobot];
	CRoboBlob* pTHEBlob = ivRoboBlobs[indexBlob];

	pTHERobot->Stop(); // first command is not accepted by robot

	CvMat *currentState, *propose_new_pos, *MaxState; 
	currentState = cvCreateMat(3,1,CV_32FC1); 
	propose_new_pos = cvCreateMat(3,1,CV_32FC1);
	MaxState = cvCreateMat(3,1,CV_32FC1);
	

	float currentCon, proposeCon, MaxCon = -1, rejCon = -1;
	int count_rej = 0; //, count_eps = 0;

	CvMat *last_acc_state, *before_acc_state;
	last_acc_state = cvCreateMat(3,1,CV_32FC1); 
	before_acc_state = cvCreateMat(3,1,CV_32FC1);
	float last_acc_con, before_acc_con;
	//cout << "1" << endl;

	AlgorithmSamplingTime = radius/Velocity;
	int num_DiscreteSamples = (int) (AlgorithmSamplingTime*1000.0f / InputSamplingTime);
	CvMat* Vin = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);
	CvMat* Omegain = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);

	int num_acc = 0;
	while(!ThStopFlag &&  count_rej <= Nr)
	{
		AlgorithmSamplingTime = radius/Velocity;
		pTHERobot->SetSamplingTime(AlgorithmSamplingTime);
		num_DiscreteSamples = (int) (AlgorithmSamplingTime*1000.0f / InputSamplingTime);
		Vin = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);
		Omegain = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);

		for( int i = 0; i < N; i++) // One Temperatur period
		{
			if ( ThStopFlag || count_rej > Nr) break; //take a shortcut when finished
		
			WorldToTableCoord( pTHEBlob->GetState(), currentState);
			WriteTrackStatestoFile(currentState);

			currentCon = MeasureConcentration( currentState);
			WriteContoFile(currentCon);
			if( count_rej > 0 && currentCon < rejCon) //if the state was previously rejected AND it concentration was higher than where the robot drove bakc to, use this concentration
			{
				currentCon = rejCon;
			}

			//Propose a new position
			if ( num_acc > 1)
			{ //with bias when more than two position got accepted
				ProposePosition_Bias_Disc(radius, sigma, last_acc_state, before_acc_state, last_acc_con, before_acc_con, propose_new_pos); //propose_new_pos will have its heading angle towards currentState
			}else{
				ProposePositionUnifDisc(currentState, radius, propose_new_pos); //propose_new_pos will have its heading angle away currentState
			}


			//Drive there and take a measurement
			//DriveFromToFast(pTHERobot, pTHEBlob, propose_new_pos);

			pTHERobot->SendNavigateCommandN(currentState, propose_new_pos);
			DetermineDiscreteInputsInTime(0,pTHEBlob,propose_new_pos, Vin, Omegain, num_DiscreteSamples);
			ApplyInputsEKFandWait(1, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);



			WorldToTableCoord( pTHEBlob->GetState(), propose_new_pos);
			WriteTrackStatestoFile(propose_new_pos);

			proposeCon = MeasureConcentration( propose_new_pos);
			WriteContoFile(proposeCon);
			

			cout << "Acceptance Probability: "  << exp( (proposeCon - currentCon) / T )  << endl;
			if ( SampleUniform(0,1) <= exp( (proposeCon - currentCon) / T ) )//Accept proposed position
			{
				currentCon = proposeCon; 
				if (proposeCon > MaxCon)
				{
					MaxCon = proposeCon;
					SetState(propose_new_pos, MaxState);
				}
				count_rej = 0;

				if ( num_acc > 0)
				{
					SetState(last_acc_state, before_acc_state);
					before_acc_con = last_acc_con;
				}
				num_acc++;
				SetState(propose_new_pos, last_acc_state);
				last_acc_con = currentCon;
			}else //reject proposed position
			{
				currentState->data.fl[2] = SetAngleInRangeZ2P(propose_new_pos->data.fl[2]-PI);
				//DriveFromToFast(pTHERobot, pTHEBlob, currentState);
				
				pTHERobot->SendNavigateCommandN(propose_new_pos, currentState);
				DetermineDiscreteInputsInTime(0,pTHEBlob,currentState, Vin, Omegain, num_DiscreteSamples);
				ApplyInputsEKFandWait(1, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);



				//SetState(currentState, LastRejectState);
				if( count_rej == 0)
				{
					rejCon = currentCon;
				}
				count_rej++;
				cout << "count_rej: " << count_rej << endl;
			}

			//TwoNormDiff(LastRejectState , currentState) <= 4 ? count_rej++ : count_rej = 0;
			//fabs(MaxCon - currentCon) < epsilon ? count_eps++ : count_eps = 0;
			cout << "---------------------------------------------------------------------------------------" << endl;
		}

	
		radius *= eta;
		if(radius <= 5) {radius = 5;} // minimum radius of 5 or 2*position_noise
		if(radius <= 2*position_noise) {radius = 2*position_noise;}
		T *= gamma;
		cout << "Temperatur change. Radius = " << radius << " Temp = " << T << endl;
		cvReleaseMat(&Vin);
		cvReleaseMat(&Omegain);
	}

	if ( !ThStopFlag )
	{
		algorithm_end = clock() - algorithm_start;
		cout << "Simulated Annealing has terminated. Total Time: " << algorithm_end /1000.0f << " seconds." <<endl;
		cout << "Maximum at: " << MaxState->data.fl[0]<<" , " << MaxState->data.fl[1] << " with " << MaxCon << endl;
		WriteResultstoFile(MaxState, MaxCon, algorithm_end /1000.0f);
	}

	cvReleaseMat(&currentState);
	cvReleaseMat(&propose_new_pos);
	cvReleaseMat(&MaxState);
	cvReleaseMat(&last_acc_state);
	cvReleaseMat(&before_acc_state);
	pTHERobot->Stop();
	ExitThread( ThreadID );
	return 0;
}