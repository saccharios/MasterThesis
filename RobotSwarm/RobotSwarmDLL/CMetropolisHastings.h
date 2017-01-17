#ifndef __CMetropolisHastings_H__
#define __CMetropolisHastings_H__
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



class CMetropolisHastings  : public CAlgorithmSF {

private:
	float sigma;
	float binsize;

	float epsilon;
	int M;
	int N;

	CvMat* old_Visits;
	CvMat* Visits;
	CvMat* normed_Visits;

	void propState(CvMat* currentState, CvMat* proposeState);
public:


	CMetropolisHastings();
	~CMetropolisHastings();


	void StartMetropolisHastings();
	static DWORD WINAPI StaticMetropolisHastingsStart(void* Param);
	DWORD MetropolisHastingsRun();

};


#endif