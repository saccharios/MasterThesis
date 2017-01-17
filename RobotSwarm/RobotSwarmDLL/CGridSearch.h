#ifndef __CGRIDSEARCH_H__
#define __CGRIDSEARCH_H__
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



class CGridSearch  : public CAlgorithmSF {

private:
	float StepLength; // == gridsize

	bool DriveToOrigin(CRobot* Robo, CRoboBlob* RoboBlob);

public:


	CGridSearch();
	~CGridSearch();

	
	//void SendNavigate(CvMat* Start, CvMat* End);


	void StartGridSearch();
	static DWORD WINAPI StaticGridSearchStart(void* Param);
	DWORD GridSearchRun();

};


#endif
