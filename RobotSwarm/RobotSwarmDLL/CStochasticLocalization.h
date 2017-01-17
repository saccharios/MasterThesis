#ifndef __CSTOCHASTICLOCALIZATION_H__
#define __CSTOCHASTICLOCALIZATION_H__
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



class CStochasticLocalization  : public CAlgorithmSF {

private:
	
	float acceptance(CvMat* currentState);
	float proposeAngle( CvMat* currentState );
	

	float K;
	float J;

	float binsize;

	float epsilon;
	int M;
	int N;

	CvMat* old_Visits;
	CvMat* Visits;
	CvMat* normed_Visits;

	int its;

public:

	CStochasticLocalization();
	~CStochasticLocalization();

	void MakeSureNoPathsCross(vector<CvMat*> vcuState , vector<CvMat*> vendState);
	bool CheckValidPaths(vector<CvMat*> vcuState, vector<CvMat*> vendState, bool* crossindicator);
	bool CheckValidPaths2(vector<CvMat*> vcuState, vector<CvMat*> vendState, bool* crossindicator);
	bool RectangleIntersect(vector<CvMat*> vcuStart, vector<CvMat*> vcuStop, vector<CvMat*> vendStart, vector<CvMat*> vendStop);
	bool LineIntersect(CvMat* Start1, CvMat* Stop1, CvMat* Start2, CvMat* Stop2);


	void StartStochasticLocalization();
	static DWORD WINAPI StaticStochasticLocalizationStart(void* Param);
	DWORD StochasticLocalizationRun();


};


#endif
