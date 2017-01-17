#include "CStochasticLocalization.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;




CStochasticLocalization::CStochasticLocalization()
{// Author: Stefan Frei, 2013

	AlgorithmSamplingTime = 3.125f;//3.33 // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 6.4f; //6
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms

	J = 4;
	K = 10;
	
	epsilon = 0.0001f;

	binsize = 10; // bin size in [cm]

	
	N = (int) ceil(WIDTH_TABLE/binsize);
	M = (int) ceil(HEIGHT_TABLE/binsize);
	//cout << "N: " << N << " M: " << M  <<endl;

	old_Visits; 
	Visits = cvCreateMat(M+1, N+1, CV_32FC1);
	cvSetZero(Visits );
	normed_Visits  = cvCreateMat(M+1, N+1, CV_32FC1);
	cvSetZero(normed_Visits);

	its = 0;
}

CStochasticLocalization::~CStochasticLocalization()
{
}

float CStochasticLocalization::proposeAngle( CvMat* currentState )
{// Author: Stefan Frei, 2013
	CvMat* new_pos = cvCreateMat( 2, 1, CV_32FC1 );
	ProjectAhead(currentState, new_pos );

	float x_left, x_right, y_low, y_up, bound_left, bound_right;
	x_left = new_pos->data.fl[0] - 2 * position_noise;
	x_right = WIDTH_TABLE - 2*position_noise - new_pos->data.fl[0] ;
	y_low = new_pos->data.fl[1] - 2*position_noise;
	y_up = HEIGHT_TABLE - 2*position_noise - new_pos->data.fl[1] ;
	//cout << "x: " << currentState->data.fl[0] <<" y: " <<currentState->data.fl[1] << " a: " << currentState->data.fl[2] << " xp: " << new_pos->data.fl[0] << " yp: "<< new_pos->data.fl[1] << endl;



	//cout << "x_left: " << x_left << ", x_right: " << x_right << ", y_low: " << y_low << ", y_up: " << y_up << endl;
	if( x_left > -StepLength && x_right > -StepLength && y_low > -StepLength && y_up > -StepLength )
	{
		if (x_left>= StepLength && x_right >= StepLength && y_low >= StepLength && y_up >= StepLength)
		{// far away from border
			bound_left = 2*PI-0.0001f;
			bound_right = 0;
		} else if (x_left >= StepLength && x_right >= StepLength && y_low >= StepLength && y_up < StepLength)
		{// top
			bound_left = PI/2 - acosf(y_up / StepLength);
			bound_right = PI/2 + acosf(y_up / StepLength);
		} else if (x_left >= StepLength && x_right >= StepLength && y_low < StepLength && y_up >= StepLength)
		{// bottom
			bound_left = 3*PI/2 - acosf(y_low / StepLength);
			bound_right = 3*PI/2 + acosf(y_low / StepLength);
		} else if (x_left >= StepLength && x_right < StepLength && y_low >= StepLength && y_up >= StepLength)
		{// right
			bound_left = 2*PI - acosf(x_right / StepLength);
			bound_right = acosf(x_right / StepLength);
		} else if (x_left < StepLength && x_right >= StepLength && y_low >= StepLength && y_up >= StepLength)
		{// left
			bound_left = PI - acosf(x_left / StepLength);
			bound_right = PI + acosf(x_left / StepLength);
		} else if (x_left < StepLength && x_right >= StepLength && y_low < StepLength && y_up >= StepLength)
		{// corner bottom left
			bound_left = PI - acosf(x_left / StepLength);
			bound_right = 3*PI/2 + acosf(y_low / StepLength);
		} else if (x_left < StepLength && x_right >= StepLength && y_low >= StepLength && y_up < StepLength)
		{// corner top left
			bound_left = PI/2 - acosf(y_up / StepLength);
			bound_right = PI + acosf(x_left / StepLength);
		} else if (x_left >= StepLength && x_right < StepLength && y_low < StepLength && y_up >= StepLength)
		{// corner bottom right
			bound_left = 3*PI/2 - acosf(y_low / StepLength);
			bound_right = acosf(x_right / StepLength);
		} else if (x_left >= StepLength && x_right < StepLength && y_low >= StepLength && y_up < StepLength)
		{// corner top right
			bound_left = 2*PI - acosf(x_right / StepLength);
			bound_right = PI/2 + acosf(y_up / StepLength);
		}
	}else// cases when vehicle drove outside 
	{ 
		cout << "This case should not happen!" << endl;
		if(x_left < -StepLength && y_up < -StepLength)
		{
			return -PI/4;
		}else if(x_left < -StepLength && y_low < -StepLength)
		{
			return PI/4;
		}else if(x_left < -StepLength)
		{
			return 0;
		}else if(y_up < -StepLength)
		{
			return -PI/2;
		}else if(y_low < -StepLength)
		{
			return PI/2;
		}else if(x_right < -StepLength && y_up < -StepLength)
		{
			return -3*PI/4;
		}else if(x_right < -StepLength && y_low < -StepLength)
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

	float sample = SampleUniform(0.0f, 1.0f);
	//cout << "Uniform Sample: " << sample << endl;

	if ( bound_right < bound_left)
	{
		propose_theta = bound_right + sample*(bound_left - bound_right);
	}else if( bound_left < bound_right)
	{
		propose_theta = bound_right + sample*(2*PI + bound_left - bound_right);
	}else{
		propose_theta = bound_left;
	}
	propose_theta = SetAngleInRangeZ2P(propose_theta);
	//cout <<" propose_theta: " << propose_theta/PI*180 << endl;

	return propose_theta;
;
}

float CStochasticLocalization::acceptance( CvMat* currentState )
{// Author: Stefan Frei, 2013
	float meas_con;
	CvMat* new_state1 = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* new_state2 = cvCreateMat( 3, 1, CV_32FC1 );

	ProjectAhead( currentState, new_state1 );
	ProjectAhead( currentState, new_state2 );


	new_state1->data.fl[0] += position_noise; 
	new_state1->data.fl[1] += position_noise;
	new_state1->data.fl[2] = currentState->data.fl[2]; 

	new_state2->data.fl[0] -= position_noise; 
	new_state2->data.fl[1] -= position_noise;
	new_state2->data.fl[2] = currentState->data.fl[2];

	

	if ( !IsValidState( new_state1 ) || !IsValidState( new_state2 ) ) {
		// if the robot would drive outside, always accept
		return 1;
	} else {
		meas_con = MeasureConcentration( currentState );
		//cout << "meas_con: " << meas_con << " acc: " << 1 - exp( -pow(K*meas_con,J)) << endl;
		return 1 - exp( -pow(K*meas_con,J));
	}

}

void CStochasticLocalization::MakeSureNoPathsCross(vector<CvMat*> vcuState , vector<CvMat*> vendState)
{//Author: Stefan Frei, 2013
	//solution: when ANY two paths cross, those robots are assigned new angles (no matter if they accepted or rejected it in the first place, then the whole system is checked again
	size_t num_Bots = vcuState.size();

	if( num_Bots != vendState.size())
		return;

	bool* crossindicator = new bool[num_Bots];
	memset( crossindicator, 0 , num_Bots );

	CvMat* tempState;
	vector<CvMat*> projStates;

	for( size_t i = 0 ; i< num_Bots; i++)
	{
		tempState = cvCreateMat(3,1, CV_32FC1 );
		ProjectAhead(vendState[i],tempState);
		projStates.push_back(tempState);
	}

	while( ! CheckValidPaths2( vendState , projStates, crossindicator) )
	{
		for( size_t i = 0 ; i< num_Bots; i++)
		{
			if ( crossindicator[i] )
			{
				cout << "Originial angle: "  << i << "  " << vendState[i]->data.fl[2] <<endl;
				vendState[i]->data.fl[2] = proposeAngle(vcuState[i]); 
				ProjectAhead(vendState[i], projStates[i]);
				cout << "Changed angle of Robot " << i << "  " << vendState[i]->data.fl[2] <<endl;
			}
		}
		memset( crossindicator, 0 , num_Bots ); //set content of crossindicator to zero
	}

	delete[] crossindicator;
}

bool CStochasticLocalization::CheckValidPaths(vector<CvMat*> vcuState, vector<CvMat*> vendState, bool* crossindicator)
{//Author: Stefan Frei, 2013
	size_t num_Bots = vcuState.size();
	if( num_Bots != vendState.size())
		return false;

	bool bret = true;
	float  dx1, dx2, dy1, dy2, dx, dy, det;
	CvMat *A, *B, *C, *D;

	for( size_t i = 0; i < num_Bots; i++)
	{

		if (vcuState[i]->data.fl[1] <  vendState[i]->data.fl[1] )
		{
			A = vcuState[i];
			B = vendState[i];
		}else
		{
			B = vcuState[i];
			A = vendState[i];
		}

		for(size_t k = i+1 ; k < num_Bots; k++)
		{
			if (vcuState[k]->data.fl[1] <  vendState[k]->data.fl[1] )
			{
				C = vcuState[k];
				D = vendState[k];
			}else
			{
				D = vcuState[k];
				C = vendState[k];
			}

			dx1 = B->data.fl[0] - A->data.fl[0]; 
			dx2 = D->data.fl[0] - C->data.fl[0]; 
			dy1 = B->data.fl[1] - A->data.fl[1]; 
			dy2 = D->data.fl[1] - C->data.fl[1]; 

			dx = A->data.fl[0] - C->data.fl[0]; 
			dy = C->data.fl[1] - A->data.fl[1]; 

			det = (dx2*dy1 - dx1*dy2);

			if ( (((dy2*dx + dx2*dy ) /det  > 0 && (dy2*dx + dx2*dy ) / det < 1)  &&  ( (dy1*dx + dx1*dy) / det > 0 && (dy1*dx + dx1*dy) / det < 1)) || TwoNormDiff( vendState[i], vendState[k]) < (4*RobotTurningRadius + 9)  ) //condition if pahts cross
			{

				crossindicator[i] = 1;
				crossindicator[k] = 1;
				bret = false;
			}

		}
	}
	return bret;

}

bool CStochasticLocalization::CheckValidPaths2(vector<CvMat*> vcuState, vector<CvMat*> vendState, bool* crossindicator)
{
	size_t num_Bots = vcuState.size();
	if( num_Bots != vendState.size())
		return false;

	bool bret = true;

	vector<CvMat*> FirstBotStart;
	FirstBotStart.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	FirstBotStart.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	vector<CvMat*> FirstBotStop;
	FirstBotStop.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	FirstBotStop.push_back(cvCreateMat( 2, 1, CV_32FC1 ));

	vector<CvMat*> SecondBotStart;
	SecondBotStart.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	SecondBotStart.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	vector<CvMat*> SecondBotStop;
	SecondBotStop.push_back(cvCreateMat( 2, 1, CV_32FC1 ));
	SecondBotStop.push_back(cvCreateMat( 2, 1, CV_32FC1 ));

	for( size_t i = 0; i < num_Bots; i++)
	{
		//make a rectangle out of the path, but neglect the first quarter of the rectangle, since there they can cross (they wont be there at the same time)
		FirstBotStart[0]->data.fl[0] = vcuState[i]->data.fl[0] - Rbot * sinf(vcuState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
		FirstBotStart[0]->data.fl[1] = vcuState[i]->data.fl[1] + Rbot * cosf(vcuState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

		FirstBotStart[1]->data.fl[0] = vcuState[i]->data.fl[0] + Rbot * sinf(vcuState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
		FirstBotStart[1]->data.fl[1] = vcuState[i]->data.fl[1] - Rbot * cosf(vcuState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

		FirstBotStop[0]->data.fl[0] = vendState[i]->data.fl[0] - Rbot * sinf(vcuState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
		FirstBotStop[0]->data.fl[1] = vendState[i]->data.fl[1] + Rbot * cosf(vcuState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

		FirstBotStop[1]->data.fl[0] = vendState[i]->data.fl[0] + Rbot * sinf(vcuState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
		FirstBotStop[1]->data.fl[1] = vendState[i]->data.fl[1] - Rbot * cosf(vcuState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

		for(size_t k = i+1 ; k < num_Bots; k++)
		{

			SecondBotStart[0]->data.fl[0] = vcuState[k]->data.fl[0] - Rbot * sinf(vendState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
			SecondBotStart[0]->data.fl[1] = vcuState[k]->data.fl[1] + Rbot * cosf(vendState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

			SecondBotStart[1]->data.fl[0] = vcuState[k]->data.fl[0] + Rbot * sinf(vendState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
			SecondBotStart[1]->data.fl[1] = vcuState[k]->data.fl[1] - Rbot * cosf(vendState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

			SecondBotStop[0]->data.fl[0] = vendState[k]->data.fl[0] - Rbot * sinf(vendState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
			SecondBotStop[0]->data.fl[1] = vendState[k]->data.fl[1] + Rbot * cosf(vendState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

			SecondBotStop[1]->data.fl[0] = vendState[k]->data.fl[0] + Rbot * sinf(vendState[i]->data.fl[3]) + StepLength/4* cosf(vcuState[i]->data.fl[3]);
			SecondBotStop[1]->data.fl[1] = vendState[k]->data.fl[1] - Rbot * cosf(vendState[i]->data.fl[3]) + StepLength/4* sinf(vcuState[i]->data.fl[3]);

			//cout << "target distance: "<< TwoNormDiff( vendState[i], vendState[k]) << endl;
			if ( TwoNormDiff( vendState[i], vendState[k]) < (4*RobotTurningRadius + 2*Rbot) || RectangleIntersect(FirstBotStart, FirstBotStop, SecondBotStart, SecondBotStop) ) //condition if pahts cross
			{
				crossindicator[i] = 1;
				crossindicator[k] = 1;
				bret = false;
			}

		}
	}
	FirstBotStart.clear();
	FirstBotStop.clear();
	SecondBotStart.clear();
	SecondBotStop.clear();

	return bret;
}

bool CStochasticLocalization::RectangleIntersect(vector<CvMat*> FirstBotStart, vector<CvMat*> FirstBotStop, vector<CvMat*> SecondBotStart, vector<CvMat*> SecondBotStop)
{// Author: Stefan Frei, 2013
	
	CvMat* M1 = cvCreateMat( 2, 1, CV_32FC1 );
	CvMat* M2 = cvCreateMat( 2, 1, CV_32FC1 );

	M1->data.fl[0] = FirstBotStart[0]->data.fl[0]/2 + FirstBotStop[1]->data.fl[0]/2;
	M1->data.fl[1] = FirstBotStart[0]->data.fl[1]/2 + FirstBotStop[1]->data.fl[1]/2;
	M2->data.fl[0] = SecondBotStart[0]->data.fl[0]/2 + SecondBotStop[1]->data.fl[0]/2;
	M2->data.fl[1] = SecondBotStart[0]->data.fl[1]/2 + SecondBotStop[1]->data.fl[1]/2;

	if( TwoNormDiff(M1,M2) < 2*sqrt(Rbot * Rbot + 9.0f/16.0f * StepLength*StepLength))  // precondition
	{
		cvReleaseMat(&M1);
		cvReleaseMat(&M2);
		cout << "Calculating rectangle intersection" << endl;
	}else // the rectangles are anyway too far away such that they cannot interstect
	{
		cvReleaseMat(&M1);
		cvReleaseMat(&M2);
		return false;
	}
	bool rectint = LineIntersect(FirstBotStart[0],FirstBotStop[0], SecondBotStart[0],SecondBotStop[0]) || LineIntersect(FirstBotStart[0],FirstBotStop[0], SecondBotStart[1],SecondBotStop[1]) || LineIntersect(FirstBotStart[0],FirstBotStop[0], SecondBotStart[0],SecondBotStart[1]) ||	LineIntersect(FirstBotStart[0],FirstBotStop[0], SecondBotStop[0],SecondBotStop[1]);
	if ( rectint )
		return true;
	rectint = LineIntersect(FirstBotStart[1],FirstBotStop[1], SecondBotStart[0],SecondBotStop[0]) || LineIntersect(FirstBotStart[1],FirstBotStop[1], SecondBotStart[1],SecondBotStop[1]) || LineIntersect(FirstBotStart[1],FirstBotStop[1], SecondBotStart[0],SecondBotStart[1]) ||	LineIntersect(FirstBotStart[1],FirstBotStop[1], SecondBotStop[0],SecondBotStop[1]);
	if ( rectint )
		return true;
	rectint = LineIntersect(FirstBotStart[0],FirstBotStart[1], SecondBotStart[0],SecondBotStop[0]) || LineIntersect(FirstBotStart[0],FirstBotStart[1], SecondBotStart[1],SecondBotStop[1]) || LineIntersect(FirstBotStart[0],FirstBotStart[1], SecondBotStart[0],SecondBotStart[1]) ||	LineIntersect(FirstBotStart[0],FirstBotStart[1], SecondBotStop[0],SecondBotStop[1]);
	if ( rectint )
		return true;	
	rectint = LineIntersect(FirstBotStop[0],FirstBotStop[1], SecondBotStart[0],SecondBotStop[0]) || LineIntersect(FirstBotStop[0],FirstBotStop[1], SecondBotStart[1],SecondBotStop[1]) || LineIntersect(FirstBotStop[0],FirstBotStop[1], SecondBotStart[0],SecondBotStart[1]) ||	LineIntersect(FirstBotStop[0],FirstBotStop[1], SecondBotStop[0],SecondBotStop[1]);
	if ( rectint )
		return true;
	cout << "Not intersecting" << endl;
	return false;

}

bool CStochasticLocalization::LineIntersect(CvMat* Start1, CvMat* Stop1, CvMat* Start2, CvMat* Stop2)
{//Author: Stefan Frei, 2013
	//checks wheter two lines intersect each other
	CvMat *A, *B, *C, *D;
	float dx, dy, dx1, dy1, dx2, dy2, det;
	if( Start1->data.fl[1] < Stop1->data.fl[1])
	{
		A = Start1;
		B = Stop1;
	}else
	{
		A = Stop1;
		B = Start1;
	}
	if( Start2->data.fl[1] < Stop2->data.fl[1])
	{
		C = Start2;
		D = Stop2;
	}else
	{
		C = Stop2;
		D = Start2;
	}

	dx1 = B->data.fl[0] - A->data.fl[0]; 
	dx2 = D->data.fl[0] - C->data.fl[0]; 
	dy1 = B->data.fl[1] - A->data.fl[1]; 
	dy2 = D->data.fl[1] - C->data.fl[1]; 

	dx = A->data.fl[0] - C->data.fl[0]; 
	dy = C->data.fl[1] - A->data.fl[1]; 

	det = (dx2*dy1 - dx1*dy2);

	det = (dx2*dy1 - dx1*dy2);

	return (((dy2*dx + dx2*dy ) /det  > 0 && (dy2*dx + dx2*dy ) / det < 1)  &&  ( (dy1*dx + dx1*dy) / det > 0 && (dy1*dx + dx1*dy) / det < 1)) ;
}

void CStochasticLocalization::StartStochasticLocalization()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
    hThread = CreateThread(NULL, 0, StaticStochasticLocalizationStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CStochasticLocalization::StaticStochasticLocalizationStart(void* Param)
{// Author: Stefan Frei, 2013
	CStochasticLocalization* This = (CStochasticLocalization*) Param;
    return This->StochasticLocalizationRun();
}

DWORD CStochasticLocalization::StochasticLocalizationRun()
{// Author: Stefan Frei, 2013
	//cout << "this thread runs" << endl;
	algorithm_start = clock();
	float delta = 10.0f;

	time_t battery_time = clock();

	float new_theta;

	

	CvMat* discretePos = cvCreateMat(2,1,CV_8UC1);

	vector<CvMat*> vcuState;
	vector<CvMat*> vendState;

	//cout << "ivRoboVect.size(): " << ivRoboVect.size() << endl;

	size_t num_Robots = ivRoboVect.size();

	int num_DiscreteSamples = (int) ( (AlgorithmSamplingTime*1000.0f + Tlag) / InputSamplingTime);
	CvMat* Vin = cvCreateMat(num_Robots,num_DiscreteSamples,CV_32FC1);
	CvMat* Omegain = cvCreateMat(num_Robots,num_DiscreteSamples,CV_32FC1);

	//stop and set sampling times for all robots
	for( size_t i = 0; i < num_Robots; i++)
	{
		ivRoboVect[i]->Stop(); //first command is not recognized of epuck
		ivRoboVect[i]->SetSamplingTime(AlgorithmSamplingTime);
		vcuState.push_back( cvCreateMat( 3, 1, CV_32FC1 )) ;
		vendState.push_back( cvCreateMat( 3, 1, CV_32FC1 )) ;
	}

	//float sampleunif, acc;

	while(!ThStopFlag && its < 50000)//delta > epsilon)// flag to stop the thread, do something while thread runs, otherwise it will stop
	{
		its += num_Robots;
		if(ivRoboBlobs.size() != num_Robots)
		{
			cout << "20 seconds to remove jam!" << endl;
			for( size_t i = 0; i < num_Robots; i++)
			{
				ivRoboVect[i]->Stop();
			}
			Sleep(20000);
		}

		if ( clock() - battery_time > 5*60*1000 ) //check battery status every 5min, 
		{
			cout << "Checking batteries..." << endl;
			BatteryExchange();
			battery_time = clock();
		}
		if( ThStopFlag ) {break;}
		//cout << "ivRoboBlobs.size() " << ivRoboBlobs.size() << endl;

		//old_Visits = cvCloneMat(normed_Visits);

		for( size_t i = 0; i < num_Robots; i++)
		{
			WorldToTableCoord( ivRoboBlobs[i]->GetState(), vcuState[i]);
			WriteTrackStatestoFile(vcuState[i]); // track the states, use this file for visualization and processing in Matlab

			//sampleunif = SampleUniform(0.0f, 1.0f);
			//acc = acceptance(vcuState[i]);
			//cout << vcuState[i]->data.fl[0] << " " << vcuState[i]->data.fl[1]  << endl;
			//cout << "sampleunif: " << sampleunif << " <= " << acc << " acceptance" << endl;

			if (  SampleUniform(0.0f, 1.0f) <=  acceptance(vcuState[i]))
			if ( true )
			{
				cout << "Accept turn" << endl;
				new_theta = proposeAngle(vcuState[i]); 
			}else{
				new_theta = vcuState[i]->data.fl[2];
			}

			ProjectAhead(vcuState[i], vendState[i]);
			vendState[i]->data.fl[2] = new_theta;
		}


		//cout << "vcuState: " << vcuState.size() << " vendState: " << vendState.size() << endl;

		MakeSureNoPathsCross(vcuState, vendState);

		start_time = clock();
		for( size_t i = 0; i < num_Robots; i++)
		{
			ivRoboVect[i]->SendNavigateCommandN(vcuState[i], vendState[i]);
			DetermineDiscreteInputsInTime(i,ivRoboBlobs[i],vendState[i], Vin, Omegain, num_DiscreteSamples);
						


			DiscretizePosition(N, M, binsize,vcuState[i], discretePos);
			//cout << "discretePos: " << discretePos->data.i[0] << "  " << discretePos->data.i[1] << endl;
			cvmSet(Visits,discretePos->data.i[1],discretePos->data.i[0], cvmGet(Visits,discretePos->data.i[1],discretePos->data.i[0])+1); // increments the visited discrete position, the indcies are interchanged 
			//cout << " Delta: " << delta << endl;
		}

		//cvConvertScale(Visits, normed_Visits, 1 , 0);
		//delta = (float) cvNorm(normed_Visits, old_Visits, CV_C,NULL);


		ApplyInputsEKFandWait(num_Robots, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);

		cout << "--------------------------------------"<< its <<"-------------------------------------------------" << endl;

	}

	cvReleaseMat(&Vin);
	cvReleaseMat(&Omegain);

	cvReleaseMat(&discretePos);
	if ( !ThStopFlag )
	{
		algorithm_end = clock() - algorithm_start;
		cout << "Stochastic Localization has terminated. Total Time: " << algorithm_end /1000.0f << endl;
	}

	for( size_t i = 0; i < num_Robots; i++)
	{
		ivRoboVect[i]->Stop();
	}

	vcuState.clear();
	vendState.clear();
	ExitThread( ThreadID );
	return 0; 

}

