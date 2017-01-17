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

//ref class CFormComm
class CFormComm
{
private:

	vector<CRoboBlob*> ivRoboBlobs;
	vector<CRobot*>    ivRoboVect;

	float Rob2Rob_d[7][7];     // Rob2Rob_d[i][j] is distance from robot i to robot j. Symmetric matrix with 0 diagonal
	float Rob2Rob_alpha[7][7]; // Rob2Rob_alpha[i][j] is relative angle from robot i to robot j. Depends on global orientation, so Rob2Rob_alpha[i][j] is NOT equal to Rob2Rob_alpha[j][i]. Diagonal is meaningless, but has value 0.

	void CalcDistancesBetweenRobots (int NRobots);
	void CalcAnglesBetweenRobots (int NRobots);

	vector<float> LastRobResponse;
public:
	DWORD ThreadID;
	HANDLE hThread;
	bool ThStopFlag;

	CFormComm(void);

	void SetBlobsAndRobots( vector<CRoboBlob*> vRoboBlobs, vector<CRobot*> vRoboVect );
	void StartAlgorithm();
	static DWORD WINAPI StaticAlgorithmStart(void* Param);
	DWORD AlgorithmRun();
	vector<float> GetLastRobResponse(int Rob_id);
};

