#ifndef __CLINESEARCH_H__
#define __CLINESEARCH_H__
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
#include <CCamera.h>
#include <CRoboBlob.h>
#include <RobotSwarmDLL.h>
#include <CameraConfig.h>
#include "CAlgorithmSF.h"

using namespace boost::numeric;
using namespace std;



//void StochasticLocalization(vector<CRoboBlob*> vRoboBlobs,vector<CRobot*> vRoboVect, CStochasticLocalization* SL);



class CLineSearch  : public CAlgorithmSF {

private:
	int N;
	float LegLength;
	float gamma;
	float PHI;

	void Determine_leg_ends( CvMat* currentState, float LegLength, CvMat* end_pos, CvMat* end_neg );
	float DriveLineTo(CRobot* Robot, CRoboBlob* Blob, CvMat* endState , CvMat* HStateCon);
	bool func_start(CvMat* New_Start,vector<CvMat*> All_Starts);
	


public:


	CLineSearch();
	~CLineSearch();


	void StartLineSearch();
	static DWORD WINAPI StaticLineSearchStart(void* Param);
	DWORD LineSearchRun();

};


#endif