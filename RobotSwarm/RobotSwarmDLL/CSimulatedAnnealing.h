#ifndef __CSimulatedAnnealing_H__
#define __CSimulatedAnnealing_H__
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



class CSimulatedAnnealing  : public CAlgorithmSF {

private:
	float radius, eta, T, gamma, epsilon, sigma;
	int N, Ne, Nr;

	float AngleUDist(CvMat* currentState, float L);
	void ProposePositionUnifDisc( CvMat* currentStateTable, float L, CvMat* propose_new_pos);
	void ProposePosition_Bias_Disc(float L, float sigma, CvMat* last_acc_state, CvMat* before_acc_state, float last_acc_con, float before_acc_con, CvMat* propose_new_pos);

public:


	CSimulatedAnnealing();
	~CSimulatedAnnealing();


	void StartSimulatedAnnealing();
	static DWORD WINAPI StaticSimulatedAnnealingStart(void* Param);
	DWORD SimulatedAnnealingRun();

};


#endif