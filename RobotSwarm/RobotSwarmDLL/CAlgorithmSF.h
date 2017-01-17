#ifndef __CALGORITHMSF_H__
#define __CALGORITHMSF_H__
// Author: Stefan Frei, 2013

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <windows.h>
#include <opencv\cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv\highgui.h>
#include <conio.h>
#include <BlobResult.h>
#include <boost\numeric\ublas\io.hpp>
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\random.hpp>
#include <boost\random\normal_distribution.hpp>
#include <CCamera.h>
#include <CRoboBlob.h>
#include <RobotSwarmDLL.h>
#include <CameraConfig.h>


using namespace boost::numeric;
using namespace std;

static boost::random::mt19937 rng(static_cast<unsigned> (std::time(0)) );  //set the seed of the uniform generator once per run
//static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));

class CAlgorithmSF{
private: 

	// coefficients for concentration function
	float X[5];
	float Y[5];
	float w[5];
	float s[5];


	

public:
	time_t elapsed_time;
	time_t start_time;
	
	time_t algorithm_start;
	time_t algorithm_end;

	vector<CRoboBlob*> ivRoboBlobs;
	vector<CRobot*> ivRoboVect;



	CAlgorithmSF();
	~CAlgorithmSF();

	float RobotTurningRadius;
	float maxCircleVelocityRobot;
	float maxVelocity;
	float Rbot;


	float AlgorithmSamplingTime; // Set algorithm sampling time in [s]
	float Velocity; // velocity of the algorithm
	float StepLength;
	float iConcentrationSigma;
	float position_noise;

	float InputSamplingTime; //is the sampling time for the inputs [ms]
	DWORD Tlag; //lag for the kalman filter, is because of bluetooth transmission

	void WorldToTableCoord( CvMat* Coord_World, CvMat* Coord_Table);
	void TableCoordToWorld( CvMat* Coord_Table, CvMat* Coord_World);


	float SampleNormal (float mean, float sigma);
	float SampleUniform(float a, float b);
	
	float ConcentrationField(CvMat* currentState);
	float MeasureConcentration( CvMat* currentState);

	void SetBlobsAndRobots( vector<CRoboBlob*> vRoboBlobs, vector<CRobot*> vRoboVect);

	void ProjectAhead( CvMat* currentState, CvMat* newposition);
	bool IsInField( CvMat* currentState );
	bool IsValidState( CvMat* currentState );

	void DeleteFiles(void);
	void WriteParameterstoFile();
	void WriteTrackStatestoFile(CvMat* currentState);
	void WriteContoFile(float Conc);
	void WriteResultstoFile(CvMat* MaxState,float MaxCon,float time);
	void WriteAcceptedStatestoFile(CvMat* currentState);

	void DiscretizePosition(int N, int M, float binsize,CvMat* currentStateNormal, CvMat* discretePos);
	int round(float number);
	void SetState(float x, float y, float alpha, CvMat* State);
	void SetState(CvMat* src, CvMat* dst);
	float SetAngleInRangeZ2P(float alpha);
	float TwoNormDiff(CvMat* src1, CvMat* src2);


	bool DriveFromToFast(CRobot* Robot, CRoboBlob* RoboBlob,CvMat* endState);
	

	void CircleDrive(CvMat* currentState, CvMat* endState, float* circleLength1, float* circleLength2, float* straightLength, int* casedubin);
	void DetermineDiscreteInputsInTime(int currentRobot,CRoboBlob* RoboBlob, CvMat* endState, CvMat* Vin, CvMat* Omegain, int num_DiscreteSamples);
	void ApplyInputsEKFandWait(int num_Robots, CvMat* Vin, CvMat* Omegain, int num_DiscreteSamples, float InputSamplingTime);

	void BatteryExchange(void);



	DWORD ThreadID;
	HANDLE hThread;
	bool ThStopFlag;


};

#endif
