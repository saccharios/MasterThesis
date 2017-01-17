#include "CLineSearch.h"
// Author: Stefan Frei, 2013
using namespace boost::numeric;
using namespace std;

CLineSearch::CLineSearch()
{
	AlgorithmSamplingTime = 2.5f; // if the algorithm sampling time and velocity are chosen badly, the robot may not reach the target and wont move at all!
	Velocity = 4.0f; 
	StepLength = Velocity * AlgorithmSamplingTime;
	InputSamplingTime = 20.0f; //in ms

	N = 4;
	LegLength = 300.0f;
	PHI = PI/6;
	gamma = 0.75f;
}

CLineSearch::~CLineSearch()
{
}

void CLineSearch::Determine_leg_ends( CvMat* currentState, float LegLength, CvMat* end_pos, CvMat* end_neg )
{//Author: Stefan Frei 2013
	
	float x = currentState->data.fl[0];
	float y = currentState->data.fl[1];
	float alpha = SetAngleInRangeZ2P(currentState->data.fl[2]);

	end_pos->data.fl[0] = x + LegLength/2 * cosf(alpha);
	end_pos->data.fl[1] = y + LegLength/2 * sinf(alpha);
	end_pos->data.fl[2] = SetAngleInRangeZ2P(alpha + PI);

	end_neg->data.fl[0] = x - LegLength/2 * cosf(alpha);
	end_neg->data.fl[1] = y - LegLength/2 * sinf(alpha);
	end_neg->data.fl[2] = alpha;

	float xlp, ylp, xln, yln, avlp,  avln;

	if ( ! (IsInField(end_pos) && IsInField(end_neg) ) ) // if both ends are not in the field, do something (no shifting possible)
	{
		//cout << "determine legs complicated" << endl;
		if ( alpha >=3*PI/2  || alpha < PI/2)
		{
			xlp = WIDTH_TABLE-position_noise - x;
			xln = -x + position_noise;

		} else 
		{
			xlp = - x +position_noise;
			xln = WIDTH_TABLE-position_noise  - x;
		}

		if ( alpha < PI)
		{
			ylp = HEIGHT_TABLE - position_noise - y;
			yln = -y +position_noise;
		}else
		{
			ylp = -y +position_noise;		
			yln = HEIGHT_TABLE-position_noise - y;
		}
		
		xlp = fabsf(xlp/cosf(alpha));
		ylp = fabsf(ylp/sinf(alpha));
		xln = fabsf(xln/cosf(alpha));
		yln = fabsf(yln/sinf(alpha));

		// take the shortes available length
		if ( xlp < ylp)
		{
			avlp = xlp;
		}else
		{
			avlp = ylp;
		}

		if ( xln < yln)
		{
			avln = xln;
		}else
		{
			avln = yln;
		}




		float total_av_length = avlp + avln;
		float lp, ln;

		ln = avln;
		lp = avlp;

		if ( avlp >= LegLength/2.0f && avln < LegLength/2.0f && total_av_length > LegLength)
		{
			lp =  LegLength - avln;
		}else if( avlp < LegLength/2.0f && avln >= LegLength/2.0f && total_av_length > LegLength )
		{
			ln = LegLength - avlp;
		}

		//for control
		if ( ln+lp - LegLength > 0.1f)
		{
			cout << "my calculations are wrong, in Determine_leg_ends " << ln+lp << " " << LegLength <<endl;
		}

		//make the positive leg the shorter leg
		if ( lp > ln )
		{
			swap(lp, ln);
			swap(end_pos->data.fl[2],end_neg->data.fl[2]);
			alpha = SetAngleInRangeZ2P(alpha + PI);
		}



		end_pos->data.fl[0] = x + lp * cosf(alpha);
		end_pos->data.fl[1] = y + lp * sinf(alpha);

		end_neg->data.fl[0] = x - ln * cosf(alpha);
		end_neg->data.fl[1] = y - ln * sinf(alpha);



	}
	//cout << "State: " << x << " " << y << " " << alpha/PI *180 << endl;
	//cout << "pos leg end: " << end_pos->data.fl[0] << " " << end_pos->data.fl[1] << " " << end_pos->data.fl[2]/PI *180 << endl;
	//cout << "neg leg end: " << end_neg->data.fl[0] << " " << end_neg->data.fl[1] << " " << end_neg->data.fl[2]/PI *180 << endl;
}

float CLineSearch::DriveLineTo(CRobot* Robot, CRoboBlob* Blob,  CvMat* endStateLine , CvMat* HStateCon = NULL)
{//Author: Stefan Frei 2013
	float dx, dy, alpha;
	float Con;
	float MaxCon = 0;

	CvMat* currentState = cvCreateMat(3,1,CV_32FC1); 
	CvMat* endState = cvCreateMat(3,1,CV_32FC1); 


	int num_DiscreteSamples = (int) (AlgorithmSamplingTime*1000.0f / InputSamplingTime);
	CvMat* Vin = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);
	CvMat* Omegain = cvCreateMat(1,num_DiscreteSamples,CV_32FC1);
	

	WorldToTableCoord( Blob->GetState(), currentState);
	WriteTrackStatestoFile(currentState);
	Con = MeasureConcentration(currentState);
	WriteContoFile(Con);


	
	float init_x = currentState->data.fl[0];
	float init_y = currentState->data.fl[1];
	float init_alpha = currentState->data.fl[2];
	cout << "Start State: " << init_x << " "  << init_y << " " << init_alpha /PI*180 << endl;
	cout << "End State: " << endStateLine->data.fl[0] << " "  << endStateLine->data.fl[1]<< " " << endStateLine->data.fl[2]/PI*180 << endl;



	float distance_to_go = TwoNormDiff( endStateLine , currentState );
	//--------------
	if ( distance_to_go < StepLength ) // if close to edge this is the case
	{

		//Robot->SendNavigateCommandN(currentState, endStateLine);

		//Sleep((DWORD) AlgorithmSamplingTime*1000-100 ); // in [ms]
		//do
		//{
		//	elapsed_time = clock()-start_time;
		//}while( elapsed_time < AlgorithmSamplingTime*1000);

		//DriveFromToInTime(Robot, Blob,endStateLine);

		Robot->SendNavigateCommandN(currentState, endStateLine);
		DetermineDiscreteInputsInTime(0,Blob,endStateLine, Vin, Omegain, num_DiscreteSamples);
		ApplyInputsEKFandWait(1, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);




		WorldToTableCoord( Blob->GetState(), currentState);

		WriteTrackStatestoFile(currentState);
		Con = MeasureConcentration(currentState);
		WriteContoFile(Con);
	}
	//-------------
	while(!ThStopFlag && distance_to_go > StepLength)
	{
		start_time = clock();

		if(Con > MaxCon)
		{
			//cout << "Con: " << Con << " highest_con: " << *highest_con << endl;
			MaxCon = Con;
			if( HStateCon != NULL) {SetState(currentState, HStateCon);}
		}

		// The robot drive straight to the end point
		alpha = atan2( endStateLine->data.fl[1] - currentState->data.fl[1], endStateLine->data.fl[0] - currentState->data.fl[0] );
		dx = StepLength * cosf(alpha);
		dy = StepLength * sinf(alpha);


		SetState(currentState->data.fl[0] + dx , currentState->data.fl[1] + dy, alpha, endState);

		//Robot->SendNavigateCommandN(currentState, endState);
		
		//Sleep((DWORD) AlgorithmSamplingTime*1000-100 ); // in [ms]
		//do
		//{
		//	elapsed_time = clock()-start_time;
		//}while( elapsed_time < AlgorithmSamplingTime*1000);
		//Sleep(100);

		//DriveFromToInTime(Robot, Blob, endState);
		Robot->SendNavigateCommandN(currentState, endState);
		DetermineDiscreteInputsInTime(0,Blob,endState, Vin, Omegain, num_DiscreteSamples);
		ApplyInputsEKFandWait(1, Vin, Omegain, num_DiscreteSamples, InputSamplingTime);

		WorldToTableCoord( Blob->GetState(), currentState);

		WriteTrackStatestoFile(currentState);
		Con = MeasureConcentration(currentState);
		WriteContoFile(Con);

		distance_to_go = TwoNormDiff(endStateLine , currentState);
		//cout << "Current State: " << currentState->data.fl[0] << " " << currentState->data.fl[1] << " "<< currentState->data.fl[2]/PI*180 << " " << endl;
		cout << "---------------------------------------------------------------------------------------" << endl;
	}


	if ( ThStopFlag ){ExitThread( ThreadID );}

	//turn vehicle for new line
	DriveFromToFast(Robot, Blob, endStateLine );


	cvReleaseMat(&Vin);
	cvReleaseMat(&Omegain);
	cvReleaseMat(&currentState);
	cvReleaseMat(&endState);

	return MaxCon;
}

bool CLineSearch::func_start(CvMat* New_Start,vector<CvMat*> All_Starts)
{//Author: Stefan Frei 2013
	//checks if New_Start is containt in All_Starts (with tolerance)
	int startnum = All_Starts.size();
	float tol = PHI/3;

	for(int i = 0; i < startnum; i++)
	{
		if( TwoNormDiff(New_Start, All_Starts[i]) < 1.5*StepLength && ( fabs(New_Start->data.fl[2] - All_Starts[i]->data.fl[2]) < tol || fabs(New_Start->data.fl[2] - All_Starts[i]->data.fl[2] -PI) < tol  || fabs(New_Start->data.fl[2] - All_Starts[i]->data.fl[2] + PI) < tol ) )
		{
			return true;
		}

	}
	return false;
}

void CLineSearch::StartLineSearch()
{// Author: Stefan Frei, 2013
	ThStopFlag = false;
	hThread = CreateThread(NULL, 0, StaticLineSearchStart, (void*) this, 0, &ThreadID);
}

DWORD WINAPI CLineSearch::StaticLineSearchStart(void* Param)
{// Author: Stefan Frei, 2013
	CLineSearch* This = (CLineSearch*) Param;
	return This->LineSearchRun();
}

DWORD CLineSearch::LineSearchRun()
{//Author: Stefan Frei 2013
	//if (ivRoboVect.size() != 1 || ivRoboBlobs.size() != 1 ){return -1;}


	algorithm_start = clock();
	int indexRobot = 0; //since we allow only one robot there is only one blob and the matching is easy
	int indexBlob = 0;
	CRobot* pTHERobot = ivRoboVect[indexRobot];
	CRoboBlob* pTHEBlob = ivRoboBlobs[indexBlob];

	pTHERobot->Stop(); // first command is not accepted by robot

	CvMat *currentState, *endState; 
	currentState = cvCreateMat(3,1,CV_32FC1); 
	endState = cvCreateMat(3,1,CV_32FC1); 

	CvMat *End_positive, *End_negative, *New_Start, *HStateCon_p, *HStateCon_n, *MaxState, *cpyNew_Start; 
	End_positive = cvCreateMat(3,1,CV_32FC1); 
	End_negative = cvCreateMat(3,1,CV_32FC1); 
	New_Start = cvCreateMat(3,1,CV_32FC1); 
	HStateCon_p = cvCreateMat(3,1,CV_32FC1); 
	HStateCon_n = cvCreateMat(3,1,CV_32FC1); 
	MaxState = cvCreateMat(3,1,CV_32FC1); 

	vector<CvMat*> All_Starts;

	float highest_con_pos, highest_con_neg, highest_con_leg, MaxCon = 0 ;
	float phi;

	float sparetime = 0.01f;
	pTHERobot->SetSamplingTime(AlgorithmSamplingTime - sparetime);

	int count = 0, itn;
	float init_alpha;
	bool nbreak_cond = true;
	bool decLL = false;

	while(!ThStopFlag && count <= N && nbreak_cond )
	{
		

		WorldToTableCoord( pTHEBlob->GetState(), currentState);

		init_alpha =  currentState->data.fl[2];
		// Calculate leg ends
		Determine_leg_ends( currentState, LegLength, End_positive, End_negative );

		//drive the positive leg
		highest_con_pos = DriveLineTo(pTHERobot,pTHEBlob,End_positive, HStateCon_p);
		
		cout << "Short leg done " << endl;

		//negative leg
		highest_con_neg = DriveLineTo(pTHERobot,pTHEBlob,End_negative, HStateCon_n);
		//pTHERobot->SetSamplingTime(AlgorithmSamplingTime - sparetime);

		cout << "Long leg done " << endl;

		//cout << "positive: " << highest_con_pos << " at " << HStateCon_p->data.fl[0] << " " << HStateCon_p->data.fl[1] << " " << HStateCon_p->data.fl[2]/PI*180 << endl;
		//cout << "negative: " << highest_con_neg << " at " << HStateCon_n->data.fl[0] << " " << HStateCon_n->data.fl[1] << " " << HStateCon_p->data.fl[2]/PI*180 << endl;

		//---Determine new start //
		if( highest_con_pos > highest_con_neg)
		{
			highest_con_leg = highest_con_pos;
			SetState(HStateCon_p, New_Start);
		}
		else
		{
			highest_con_leg = highest_con_neg;
			SetState(HStateCon_n, New_Start);
		}
		//cout << "highest_con_leg: " << highest_con_leg << "MaxCon: " << MaxCon << endl;
		if(highest_con_leg > MaxCon)
		{
			MaxCon = highest_con_leg;
			SetState(New_Start, MaxState);
			count = 0;
		}else
		{
			count++;
		}
		//cout << "counter: " << count << endl;
		if (count % 2 == 1) //rotate axis and shorten leg
		{
			phi = PHI;
			LegLength *= gamma;
			if( LegLength <= 3* StepLength) //minimum leg length
			{
				LegLength *= 3;
			}
			decLL = true;
			cout << "Decreasing LegLength to: " << LegLength << endl;

		} else
		{
			phi = PI/2;
			decLL = false;
		}

		New_Start->data.fl[2] = SetAngleInRangeZ2P(init_alpha + phi);
		//cout << "new start: " << New_Start->data.fl[0] << " " << New_Start->data.fl[1] <<" " << New_Start->data.fl[2]/PI*180 << endl;

		if (func_start(New_Start, All_Starts))
		{
			itn = 1;
			New_Start->data.fl[2] = SetAngleInRangeZ2P(New_Start->data.fl[2] - phi + PHI);
			if(!decLL)
			{
				LegLength *= gamma;
				if( LegLength <= 3* StepLength){LegLength *= 3;} //minimum leg length
				cout << "Decreasing LegLength: " << LegLength;
			}
			cout << "\t rotate new starting state: " << itn << endl;
			while( func_start(New_Start, All_Starts) && nbreak_cond)
			{
				if (itn >= ceil(PI/PHI) ) 
				{
					break;
					nbreak_cond = false;
				}
				itn++;
				New_Start->data.fl[2] = SetAngleInRangeZ2P(New_Start->data.fl[2] + PHI);
				cout << "Rotate new starting state: " << itn << endl;
				
				
			}
		}
		if( !nbreak_cond ) { break; }

		cpyNew_Start =  cvCreateMat(3,1,CV_32FC1);
		SetState(New_Start, cpyNew_Start);
		All_Starts.push_back(cpyNew_Start);
		////drive to new start
		
		DriveLineTo(pTHERobot,pTHEBlob, New_Start);

		//pTHERobot->SetSamplingTime(AlgorithmSamplingTime - sparetime);

		cout << "arrived at new start, LegLength: " << LegLength << endl;
		//Sleep(1000);
	}

	if ( !ThStopFlag )
	{
		algorithm_end = clock() - algorithm_start;
		cout << "Line Search has terminated. Total Time: " << algorithm_end /1000.0f << endl;
		cout << "Maximum at: " << MaxState->data.fl[0]<< " " << MaxState->data.fl[1] << " with " << MaxCon << endl;
		WriteResultstoFile(MaxState, MaxCon, algorithm_end /1000.0f);
	}

	cvReleaseMat(&currentState);
	cvReleaseMat(&endState); 
	cvReleaseMat(&End_positive);
	cvReleaseMat(&End_negative);
	cvReleaseMat(&New_Start);
	cvReleaseMat(&HStateCon_p);
	cvReleaseMat(&HStateCon_n); 
	All_Starts.clear();

	pTHERobot->Stop();
	ExitThread( ThreadID );
	return 0;
}