#include "CAlgorithmSF.h"

using namespace boost::numeric;
using namespace std;


CAlgorithmSF::CAlgorithmSF()
{// Author: Stefan Frei, 2013

	iConcentrationSigma = 0.02f;
	position_noise = 5.0f;

	start_time = clock();

	ThStopFlag = false;
	ThreadID = NULL;

	RobotTurningRadius = 2.65f;//2.65
	Rbot = 5.50f; //radius of robot (4.2+1.3 for safety)
	maxVelocity = 12.88f;
	

	maxCircleVelocityRobot = maxVelocity/(1.0f+5.3f/2.0f/RobotTurningRadius); //max constant speed (limited by circle) =6.44


	Tlag = 120; // in [ms], 120 is good

	  //  X=[50,100,160,250,270]*4/5;
      //  Y=[50,200,100,160,60]*4/5;

	X[0] = 40;
	X[1] = 80;
	X[2] = 128;
	X[3] = 200;
	X[4] = 216; 

	Y[0] = 40;
	Y[1] = 160;
	Y[2] = 80;
	Y[3] = 128;
	Y[4] = 48;

	w[0] = 1;
	w[1] = 0.4f;
	w[2] = 0.55f;
	w[3] = 0.65f;
	w[4] = 0.2f;

	s[0] = 0.06f;
	s[1] = 0.07f;
	s[2] = 0.13f;
	s[3] = 0.35f;
	s[4] = 0.145f;

	
}

CAlgorithmSF::~CAlgorithmSF()
{
}

void CAlgorithmSF::WorldToTableCoord( CvMat* Coord_World, CvMat* Coord_Table)
{// Author: Stefan Frei, 2013
	// Transformation betweencamera world and algorithm coordinates 
	CvSize size = cvGetSize(Coord_Table);

	// ADJUST so that left lower corner is (0,0) 
	Coord_Table->data.fl[0] = Coord_World->data.fl[0] - OFFSET_X;
	Coord_Table->data.fl[1] = HEIGHT_WORLD - Coord_World->data.fl[1] - OFFSET_Y;
	Coord_Table->data.fl[2] = fmod(PI - Coord_World->data.fl[2], 2*PI);
	if(Coord_Table->data.fl[2] < 0) {Coord_Table->data.fl[2] += 2*PI;}


	if( size.height == 5)
	{
		Coord_Table->data.fl[3] = Coord_World->data.fl[3];
		Coord_Table->data.fl[4] = Coord_World->data.fl[4];
	}

}

void CAlgorithmSF::TableCoordToWorld( CvMat* Coord_Table, CvMat* Coord_World)
{// Author: Stefan Frei, 2013
	// Transformation between algorithm coordinates and camera world

	Coord_World->data.fl[0] = Coord_Table->data.fl[0] + OFFSET_X;
	Coord_World->data.fl[1] = HEIGHT_WORLD - Coord_Table->data.fl[1] - OFFSET_Y;
	Coord_World->data.fl[2] = PI - Coord_Table->data.fl[2]; 
}

float CAlgorithmSF::SampleNormal(float mean, float sigma)
{
    // Create a Mersenne twister random number generator
    // that is seeded once with #seconds since 1970
    //boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
 
    // select Gaussian probability distribution
    boost::normal_distribution<float> norm_dist(mean, sigma);
 
    // bind random number generator to distribution, forming a function
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> >  normal_sampler(rng, norm_dist);
 
    // sample from the distribution
    return normal_sampler();
}

float CAlgorithmSF::SampleUniform(float a, float b)
{// Author: Stefan Frei, 2013

	if ( a == b)
	{
		return a;
	}else if( b < a)
	{   
		boost::random::uniform_real_distribution<float> unif(b,a);
		return unif(rng);          
	}else{
		boost::random::uniform_real_distribution<float> unif(a,b);
		return unif(rng);                   
	}
}

float CAlgorithmSF::ConcentrationField(CvMat* currentState)
{// Author: Stefan Frei, 2013
	// Returns exact concentration of TF1
	float x = currentState->data.fl[0];
	float y = currentState->data.fl[1];

	float concentration = 0;



	for( int i = 0; i < 5; i++)
	{
		concentration += w[i]*exp(-s[i]*sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i])));
	}


	return concentration;
}

float CAlgorithmSF::MeasureConcentration( CvMat* currentState)
{// Author: Stefan Frei, 2013
	// Returns measured concentration

	float concentration;
	concentration = ConcentrationField(currentState);
	concentration = SampleNormal(concentration, iConcentrationSigma);

	// Cap concentration
	if (concentration < 0) { concentration = 0;}


	return concentration;
}

void CAlgorithmSF::SetBlobsAndRobots( vector<CRoboBlob*> vRoboBlobs, vector<CRobot*> vRoboVect)
{// Author: Stefan Frei, 2013
	ivRoboBlobs = vRoboBlobs;
	ivRoboVect = vRoboVect;

}

void CAlgorithmSF::ProjectAhead( CvMat* currentState, CvMat* newposition)
{// Author: Stefan Frei, 2013
	//projects the state ahead, in: 3x1, out: 2x1
	newposition->data.fl[0] = currentState->data.fl[0] + StepLength*cosf(currentState->data.fl[2]);
	newposition->data.fl[1] = currentState->data.fl[1] + StepLength*sinf(currentState->data.fl[2]);
}

bool CAlgorithmSF::IsInField( CvMat* Position )
{// Author: Stefan Frei, 2013
	// 2x1,checks if Position is in the field (table)

	if ( Position->data.fl[0] >= 0 && Position->data.fl[0] <= WIDTH_TABLE && Position->data.fl[1] >= 0 && Position->data.fl[1] <= HEIGHT_TABLE )
	{
		return true;
	} else {
		return false;
	}
}

bool CAlgorithmSF::IsValidState( CvMat* currentState )
{// Author: Stefan Frei, 2013
	// 3x1
	CvMat* new_pos1 = cvCreateMat( 2, 1, CV_32FC1 );
	CvMat* new_pos2 = cvCreateMat( 2, 1, CV_32FC1 );

	ProjectAhead( currentState, new_pos1 );
	ProjectAhead( currentState, new_pos2 );

	return IsInField( new_pos1 ) && IsInField( new_pos2 );

}

void CAlgorithmSF::DeleteFiles(void)
{
	remove( "TrackStates.txt" );
	remove( "TrackConcentration.txt" );
	remove( "DataMeasStateCovLog.txt" );
	remove( "NavigateCommands.txt" ) ;
	remove( "EFKTime.txt" );
	remove( "DataMeasStateCovLog2.txt");
	remove( "MHacceptedStates.txt");

}

void CAlgorithmSF::WriteParameterstoFile()
{
	ofstream file;
	file.open("TrackStates.txt",ios::app|ios::out);		//open a file
	if( file.is_open() )
	{
		file << setprecision(4);

		file << WIDTH_WORLD<< "\t"  << HEIGHT_WORLD <<"\t" << WIDTH_TABLE << "\t" << HEIGHT_TABLE <<"\t"<< OFFSET_X  <<"\t" << OFFSET_Y << "\t" << position_noise;
		file << endl;
		for( int k= 0; k< 5; k++)
		{
			file << X[k] << "\t";
		}
		file << endl;
		for( int k= 0; k< 5; k++)
		{
			file << Y[k] << "\t";
		}
		file << endl;
		for( int k= 0; k< 5; k++)
		{
			file << w[k] << "\t";
		}
		file << endl;
		for( int k= 0; k< 5; k++)
		{
			file << s[k] << "\t";
		}
		file << endl;

		file.close();	
	}
}

void CAlgorithmSF::WriteTrackStatestoFile(CvMat* currentState)
{// Author: Stefan Frei, 2013

	ofstream file;
	file.open("TrackStates.txt",ios::app|ios::out);		//open a file
	file << setprecision(4);
	for( int i = 0; i < 3; i++)
	{
		file << currentState->data.fl[i] << "\t";
	}
	file << endl;

	file.close();	
}

void CAlgorithmSF::WriteAcceptedStatestoFile(CvMat* currentState)
{// Author: Stefan Frei, 2013

	ofstream file;
	file.open("MHacceptedStates.txt",ios::app|ios::out);		//open a file
	file << setprecision(4);
	for( int i = 0; i < 3; i++)
	{
		file << currentState->data.fl[i] << "\t";
	}
	file << endl;

	file.close();	
}
void CAlgorithmSF::WriteResultstoFile(CvMat* MaxState,float MaxCon,float time)
{// Author: Stefan Frei, 2013
	ofstream file;
	file.open("TrackStates.txt",ios::app|ios::out);		//open a file
	file << setprecision(4);
	file << MaxState->data.fl[0] << "\t" << MaxState->data.fl[1] << "\t" << MaxCon << "\t" << time << endl;
	file.close();
}

void CAlgorithmSF::WriteContoFile(float Conc)
{// Author: Stefan Frei, 2013


	ofstream file;
	file.open("TrackConcentration.txt",ios::app|ios::out);		//open a file
	file << setprecision(4);

	file << Conc << "\t";
	file.close();	
}

void CAlgorithmSF::DiscretizePosition(int N, int M, float binsize,CvMat* currentStateNormal, CvMat* discretePos)
{// Author: Stefan Frei, 2013
// returns discrete position [0...N, 0...M], the discrete position is in the middle of its bin

	float fx = currentStateNormal->data.fl[0];
	float fy = currentStateNormal->data.fl[1];

	int ix = (int) (round(fx/binsize));
	int iy = (int) (round(fy/binsize)); 
	if( ix < 0)
	{
		cout << "The vehicle drove outside: " << currentStateNormal->data.fl[0]  << " " <<currentStateNormal->data.fl[1];
		ix = 0;
	}else if(ix > N)
	{
		cout << "The vehicle drove outside: " << currentStateNormal->data.fl[0]  << " " <<currentStateNormal->data.fl[1];
		ix = N;
	}else if (iy < 0)
	{
		cout << "The vehicle drove outside: " << currentStateNormal->data.fl[0]  << " " <<currentStateNormal->data.fl[1];
		iy = 0;
	}else if( iy > M)
	{
		cout << "The vehicle drove outside: " << currentStateNormal->data.fl[0]  << " " <<currentStateNormal->data.fl[1];
		iy = M;
	}


	discretePos->data.i[0] = ix;
	discretePos->data.i[1] = iy;


}

int CAlgorithmSF::round(float number)
{// from: http://stackoverflow.com/questions/554204/where-is-round-in-c ,Patrick Daryll Glandien
	return (int) (number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5));
}

void CAlgorithmSF::SetState(float x, float y, float alpha, CvMat* State)
{//Author: Stefan Frei 2013
	State->data.fl[0] = x;
	State->data.fl[1] = y;
	State->data.fl[2] = alpha;
}

void CAlgorithmSF::SetState(CvMat* src, CvMat* dst)
{//Author: Stefan Frei 2013
	dst->data.fl[0] = src->data.fl[0];
	dst->data.fl[1] = src->data.fl[1];
	dst->data.fl[2] = src->data.fl[2];
}

float CAlgorithmSF::SetAngleInRangeZ2P(float alpha)
{// Author: Stefan Frei, 2013
	// wraps alpha into [0, 2*pi]

	alpha = fmodf(alpha, 2*PI);
	if (alpha < 0 ) { alpha += 2*PI; }
	return alpha;
}

float CAlgorithmSF::TwoNormDiff(CvMat* src1, CvMat* src2)
{//Author: Stefan Frei 2013
	//return 2norm of the difference of to vectors (i.e. the distance between them)

	CvSize size1 = cvGetSize(src1);
	CvSize size2 = cvGetSize(src2);
	if ( ((size1.height == 2 ||  size1.height == 3 ||  size1.height == 5 ) && size1.width == 1 ) && ((size2.height == 2 ||  size2.height == 3 ||  size2.height == 5) && size2.width == 1))
		return sqrt( (src1->data.fl[0] - src2->data.fl[0])*(src1->data.fl[0] - src2->data.fl[0]) + (src1->data.fl[1] - src2->data.fl[1])*(src1->data.fl[1] - src2->data.fl[1]));
	else
	{
		return -1;
	}
}

bool CAlgorithmSF::DriveFromToFast(CRobot* Robot, CRoboBlob* RoboBlob,CvMat* endState)
{// Author: Stefan Frei, 2013
	//for ONE bot

	CvMat* currentState = cvCreateMat( 3, 1, CV_32FC1 );
	WorldToTableCoord(RoboBlob->GetState(), currentState);

	
	Robot->SendNavigateCommandNAFAP( currentState, endState );
	int num_Robots = 1;


	//wait until it has arrived (approx) 
	float circleLength1,circleLength2, straightLength;
	int casedubin;
	CircleDrive( currentState, endState , &circleLength1,&circleLength2, &straightLength, &casedubin );

	float T1 = circleLength1/ maxCircleVelocityRobot *1000.0f + Tlag;
	float T2 = T1 + straightLength/maxVelocity *1000.0f;
	float T3 = T2 + circleLength2/ maxCircleVelocityRobot *1000.0f;
		

	int num_DiscreteSamples = (int) (T3/ InputSamplingTime);

	CvMat* Vin = cvCreateMat(num_Robots , num_DiscreteSamples,CV_32FC1);
	CvMat* Omegain = cvCreateMat(num_Robots ,num_DiscreteSamples,CV_32FC1);	

	int sign[2];
	if(casedubin == 0 ||casedubin == 2)
	{
		sign[0] = 1;
	}else{
		sign[0] = -1;
	}
	if(casedubin == 0 ||casedubin == 3)
	{
		sign[1] = 1;
	}else{
		sign[1] = -1;
	}
	float omega = maxCircleVelocityRobot/RobotTurningRadius;
	// Fill up Vin am Omegain
	int i = 0;

	while(InputSamplingTime*((float) i) < Tlag && i < num_DiscreteSamples )
	{
		cvmSet(Vin,0,i,maxVelocity/5);
		cvmSet(Omegain,0,i,0);
		i++;
	}


	while(InputSamplingTime*((float) i) < T1 && i < num_DiscreteSamples )
	{
		cvmSet(Vin,0,i,maxCircleVelocityRobot);
		cvmSet(Omegain,0,i,sign[0]*omega);
		i++;
	}
	while(InputSamplingTime*((float) i) < T2 && i < num_DiscreteSamples  )
	{
		cvmSet(Vin,0,i,maxVelocity);
		cvmSet(Omegain,0,i,0.0f);
		i++;
	}
	while(InputSamplingTime*((float) i) < T3 && i < num_DiscreteSamples )
	{
		cvmSet(Vin,0,i,maxCircleVelocityRobot);
		cvmSet(Omegain,0,i,sign[1]*omega);
		i++;
	}

	ApplyInputsEKFandWait(num_Robots,  Vin, Omegain, num_DiscreteSamples, InputSamplingTime);

	cvReleaseMat(&Omegain);
	cvReleaseMat(&Vin);

	WorldToTableCoord(RoboBlob->GetState(), currentState);
	if( TwoNormDiff(currentState , endState) <= 5 )
	{
		cvReleaseMat(&currentState);
		return true;
	}else
	{
		cvReleaseMat(&currentState);
		return false;
	}

}

void CAlgorithmSF::CircleDrive(CvMat* currentState, CvMat* endState, float* circleLength1, float* circleLength2, float* straightLength, int* casedubin)
{//Author: Stefan Frei, 2013
	//Returns the total length of the path of the robot, when it follows the CircleDrive strategy
	float x0 = currentState->data.fl[0];
	float y0 = currentState->data.fl[1];
	float alpha0 = currentState->data.fl[2];

	float xN = endState->data.fl[0];
	float yN = endState->data.fl[1];
	float alphaN = endState->data.fl[2];

	float dx, dy, gamma, beta1, beta2;
	float sLength[4], cLength1[4], cLength2[4];

	float xAl = x0 - sinf(alpha0)*RobotTurningRadius;
	float yAl = y0 + cosf(alpha0)*RobotTurningRadius;

	float xBl = xN - sinf(alphaN)*RobotTurningRadius;
	float yBl = yN + cosf(alphaN)*RobotTurningRadius;

	float xAr = x0 + sinf(alpha0)*RobotTurningRadius;
	float yAr = y0 - cosf(alpha0)*RobotTurningRadius;

	float xBr = xN + sinf(alphaN)*RobotTurningRadius;
	float yBr = yN - cosf(alphaN)*RobotTurningRadius;

	float zx, zy, zz;

	float angle_tol = 0.001f;
	//case left-left
	dx = xBl - xAl;
	dy = yBl - yAl;

	sLength[0] = sqrt(dx*dx + dy*dy); 

	gamma = atan2(dy, dx);
	gamma = SetAngleInRangeZ2P( gamma );


	if( fabs( gamma-alpha0 ) < angle_tol)
	{
		beta1 = 0.0f;
	}else
	{
		beta1 = SetAngleInRangeZ2P( gamma-alpha0 );
	}
	if( fabs( alphaN-gamma ) < angle_tol)
	{
		beta2 = 0.0f;
	}else
	{
		beta2 = SetAngleInRangeZ2P( alphaN-gamma );
	}
	cLength1[0] = RobotTurningRadius * beta1;
	cLength2[0] = RobotTurningRadius * beta2;

	//case right-right
	dx = xBr - xAr;
	dy = yBr - yAr;

	sLength[1] = sqrt(dx*dx + dy*dy); 

	gamma = atan2(dy, dx);
	gamma = SetAngleInRangeZ2P( gamma );

	if( fabs(alpha0 - gamma ) < angle_tol)
	{
		beta1 = 0.0f;
	}else
	{
		beta1 = SetAngleInRangeZ2P( alpha0 - gamma );
	}
	if ( fabs( gamma - alphaN ) < angle_tol)
	{
		beta2 = 0.0f;
	}else
	{
		beta2 = SetAngleInRangeZ2P( gamma - alphaN );
	}

	cLength1[1] = RobotTurningRadius * beta1;
	cLength2[1] = RobotTurningRadius * beta2;


	//case left-right
	dx = xBr - xAl;
	dy = yBr - yAl;

	if( dx*dx + dy*dy >= 4*RobotTurningRadius*RobotTurningRadius)
	{
		sLength[2] = sqrt(dx*dx + dy*dy - 4*RobotTurningRadius*RobotTurningRadius);

		gamma = asinf( (2*RobotTurningRadius*dx + dy*sLength[2]) / (dx*dx + dy*dy) );
		gamma = SetAngleInRangeZ2P( gamma );


		zx = xAl + RobotTurningRadius*sinf(gamma) + sLength[2]*cosf(gamma) - xBr;
		zy = yAl - RobotTurningRadius*cosf(gamma) + sLength[2]*sinf(gamma) - yBr;
		zz = sqrt(zx*zx + zy*zy);

		if ( fabs(zz - RobotTurningRadius) > 0.1 ) {gamma = PI - gamma;}


		if(fabs( gamma - alpha0 ) < angle_tol)
		{
			beta1 = 0.0f;
		}else
		{
			beta1 = SetAngleInRangeZ2P( gamma - alpha0 );
		}
		if(fabs( gamma - alphaN ) < angle_tol)
		{
			beta2 = 0.0f;
		}else
		{
			beta2 = SetAngleInRangeZ2P( gamma - alphaN );
		}

		cLength1[2] = RobotTurningRadius * beta1;
		cLength2[2] = RobotTurningRadius * beta2;

	}else
	{
		sLength[2] = 999;
		cLength1[2] = 999;
		cLength2[2] = 999;
	}

	//case right-left
	dx = xBl - xAr;
	dy = yBl - yAr;

	if( dx*dx + dy*dy >= 4*RobotTurningRadius*RobotTurningRadius)
	{
		sLength[3] = sqrt(dx*dx + dy*dy - 4*RobotTurningRadius*RobotTurningRadius);

		gamma = asinf( (-2*RobotTurningRadius*dx + dy*sLength[3]) / (dx*dx + dy*dy));
		gamma = SetAngleInRangeZ2P( gamma );


		zx = xAr - RobotTurningRadius*sinf(gamma) + sLength[3]*cosf(gamma) - xBl;
		zy = yAr + RobotTurningRadius*cosf(gamma) + sLength[3]*sinf(gamma) - yBl;

		zz = sqrt(zx*zx + zy*zy);

		if ( fabs(zz - RobotTurningRadius) > 0.1 ) {gamma = PI - gamma;}

		if(fabs( alpha0 - gamma ) < angle_tol)
		{
			beta1 = 0.0f;
		}else
		{
			beta1 = SetAngleInRangeZ2P( alpha0 - gamma );
		}
		if(fabs( alphaN - gamma ) < angle_tol)
		{
			beta2 = 0.0f;
		} else
		{
			beta2 = SetAngleInRangeZ2P( alphaN - gamma );
		}
		cLength1[3] = RobotTurningRadius * beta1;
		cLength2[3] = RobotTurningRadius * beta2;

	}else
	{
		sLength[3] = 999;
		cLength1[3] = 999;
		cLength2[3] = 999;


	}
	//------------
	int index = -1;
	float value = 999;
	for(int i = 0; i< 4; i++)
	{
		//cout << "straight: " << sLength[i] << " circle: " << cLength[i] << endl;
		if( sLength[i] + cLength1[i] + cLength2[i] < value )
		{
			value = sLength[i] + cLength1[i] + cLength2[i];
			index = i;
		}
	}
	//cout << "Case: " << index << " l2 = " << sLength[2] + cLength1[2] + cLength2[2] << " l3 = " << sLength[3] + cLength1[3] + cLength2[3] << endl;
	*circleLength1 = cLength1[index];
	*circleLength2 = cLength2[index];
	*straightLength = sLength[index];

	*casedubin = index;


}

void CAlgorithmSF::DetermineDiscreteInputsInTime(int currentRobot,CRoboBlob* RoboBlob, CvMat* endState, CvMat* Vin, CvMat* Omegain, int num_DiscreteSamples)
{//Author: Stefan Frei, 2013
	//Calculates the discrete inputs and stores it in rows in Vin, Omegain, for multiple bots
	CvMat* currentState = cvCreateMat( 3, 1, CV_32FC1 );
	WorldToTableCoord(RoboBlob->GetState(), currentState);

	float circleLength1, circleLength2, straightLength;
	int casedubin;
	CircleDrive( currentState, endState , &circleLength1,&circleLength2, &straightLength,&casedubin );

	

	float v_circ, v_straight;

	
	//drive with constant speed
	v_circ = (circleLength1 + circleLength2 + straightLength)/AlgorithmSamplingTime;
	v_straight = v_circ;

	if(v_circ > maxCircleVelocityRobot ) 
	{
		if( (circleLength1 + circleLength2)/maxCircleVelocityRobot + straightLength/maxVelocity < AlgorithmSamplingTime)
		{
			v_circ = maxCircleVelocityRobot;
			v_straight = straightLength/(AlgorithmSamplingTime - (circleLength1 + circleLength2)/maxCircleVelocityRobot );

		}else
		{
			cout << "Choose higher sampling time!" << endl;
		}
	}
	float omega = v_circ/RobotTurningRadius;


	float T1 = circleLength1/ v_circ *1000.0f + Tlag ;
	float T2 = T1 + straightLength/v_straight *1000.0f;
	float T3 = T2 + circleLength2/ v_circ *1000.0f;
		
	
	float sign[2];
	if(casedubin == 0 ||casedubin == 2)
	{
		sign[0] = 1;
	}else{
		sign[0] = -1;
	}
	if(casedubin == 0 ||casedubin == 3)
	{
		sign[1] = 1;
	}else{
		sign[1] = -1;
	}

	// Fill up Vin am Omegain
	int i = 0;
	while(InputSamplingTime*((float) i) < Tlag && i < num_DiscreteSamples )
	{
		cvmSet(Vin,currentRobot,i,maxVelocity/5);
		cvmSet(Omegain,currentRobot,i,0);
		i++;
	}

	while(InputSamplingTime*((float) i) < T1 && i < num_DiscreteSamples )
	{
		cvmSet(Vin,currentRobot,i,v_circ);
		cvmSet(Omegain,currentRobot,i,sign[0]*omega);
		i++;
	}

	while(InputSamplingTime*((float) i) < T2 && i < num_DiscreteSamples  )
	{

		cvmSet(Vin,currentRobot,i,v_straight);
		cvmSet(Omegain,currentRobot,i,0.0f);
		i++;
	}

	while(InputSamplingTime*((float) i) < T3 && i < num_DiscreteSamples )
	{
		cvmSet(Vin,currentRobot,i,v_circ);
		cvmSet(Omegain,currentRobot,i,sign[1]*omega);
		i++;
	}

}

void CAlgorithmSF::ApplyInputsEKFandWait(int num_Robots, CvMat* Vin, CvMat* Omegain, int num_DiscreteSamples, float InputSamplingTime)
{//Author: Stefan Frei, 2013
	//Sets the calculated inputs in real time into the EFK for all Robots, (expires time)


	for( int i = 0; i < num_DiscreteSamples;i++)
	{
		for(int k = 0; k < num_Robots; k++)
		{
			//cout << cvmGet(Vin,k,i) << " , " << cvmGet(Omegain,k,i) << endl;
			ivRoboBlobs[k]->SetInput((float) cvmGet(Vin,k,i), (float) cvmGet(Omegain,k,i));
		}
		Sleep( (DWORD) InputSamplingTime);
	}
	Sleep(100);
	for(int k = 0; k < num_Robots; k++)
	{
		ivRoboBlobs[k]->SetInput(maxVelocity/5.0f,0);
	}


}

void CAlgorithmSF::BatteryExchange(void)
{// Author: Stefan Frei, 2013
	//checks if battery status is ok, if not there is time to exchange batteries
	// DOES NOT WORK, comports get mixed up!

	size_t num_Robots = ivRoboVect.size();
	bool* BatStat = new bool[num_Robots];
	int timetoex = 60;
	int count = 0;
	bool statlow = false;

	for( size_t i = 0; i < num_Robots; i++)
	{
		BatStat[i] = ivRoboVect[i]->GetBatteryLevel();
		if ( !BatStat[i])
		{
			cout << "Battery of Robot " << i << " low" << endl;
			timetoex += 40;
			statlow = true;
		}
	}

	if(statlow)
	{
		
		for( size_t i = 0; i < num_Robots; i++)
		{
			ivRoboVect[i]->Stop();
			ivRoboBlobs[i]->SetInput(0.0f,0.0f);
			//if ( !BatStat[i] )
			//{
			//	ivRoboVect[i]->Disconnect();
			//}
		}


		for(int k = 0; k < 10; k++)
		{
			cout << "PRESS STOP" << endl;
			Sleep(2000);
		}

	}


	//if(timetoex > 60)
	//{

	//	cout << "Exchange those batteries. You have " << timetoex << " seconds." << endl;
	//	for( size_t i = 0; i < num_Robots; i++)
	//	{
	//		ivRoboVect[i]->Stop();
	//		ivRoboBlobs[i]->SetInput(0.0f,0.0f);
	//		if ( !BatStat[i])
	//		{
	//			ivRoboVect[i]->Disconnect();
	//		}
	//	}

	//	Sleep(timetoex*1000);

	//	cout << "Times is up! Continue Algorithm." << endl;
	//	
	//	for( size_t i = 0; i < num_Robots; i++) // Stop all bots
	//	{
	//		if ( !BatStat[i])
	//		{
	//			while( !ivRoboVect[i]->Connect())
	//			{
	//				count++;
	//				if ( count == 10)
	//				{
	//					cout << "Connecting totally failed. Stop algorithm" << endl;
	//					return;
	//				}
	//			}
	//			ivRoboVect[i]->Stop();
	//			ivRoboVect[i]->SetSamplingTime(AlgorithmSamplingTime);
	//		}
	//	}
	//	algorithm_start += timetoex; //subtract wasted time from algorithm time
	//}

	delete[] BatStat;


}