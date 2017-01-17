#ifndef _CROBOBLOB_H_
#define _CROBOBLOB_H_

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
#include <CameraConfig.h>
//#include <CRobot.h>

#include "boost\date_time\posix_time\posix_time_types.hpp"

using namespace boost::numeric;
using namespace std;

//const double PI = 3.1415926535;
const float PI = 3.1415926f;


const int MAX_STEPS = 40;


const float WHEEL_DISTANCE_EPUCK = 5.3f; //in cm
const float WHEEL_DISTANCE_KHEPERA = 9.0f; //in cm



const float F_f[5][5] = { {1, 0, 0, 0,0},{0, 1, 0, 0, 0},{0, 0, 1, 0, 0},{0, 0, 0, 1, 0},{0, 0, 0, 0, 1} }; 
const float F_transposed_f[5][5] = { {1, 0, 0, 0,0},{0, 1, 0, 0, 0},{0, 0, 1, 0, 0},{0, 0, 0, 1, 0},{0, 0, 0, 0, 1} }; 
const float H_f[3][5] = {{1, 0, 0, 0, 0},{ 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}};
const float H_transposed_f[5][3]= {{1, 0, 0},{ 0, 1, 0}, {0, 0, 1},{0, 0, 0},{0, 0, 0}}; 
const float I_f[5][5] = { {1, 0, 0, 0,0},{0, 1, 0, 0, 0},{0, 0, 1, 0, 0},{0, 0, 0, 1, 0},{0, 0, 0, 0, 1} };

const float R_f[3][3] = { {0.2f, 0, 0} , {0, 0.2f, 0} , {0, 0, 0.2f}}; // measurement noise
const float Q_f[5][5] = { {0.02f, 0,0,0,0} , {0, 0.02f, 0, 0, 0} , {0, 0, 0.02f, 0,0} , {0, 0,0, 0.4f, 0} , {0, 0,0, 0, 0.4f} }; //process noise


const float F2_f[3][3] = { {1, 0, 0},{0, 1, 0},{0, 0, 1}}; 
const float F2_transposed_f[3][3] = { {1, 0, 0},{0, 1, 0},{0, 0, 1}}; 
const float H2_f[3][3] = { {1, 0, 0},{0, 1, 0},{0, 0, 1}}; 
const float H2_transposed_f[3][3]= { {1, 0, 0},{0, 1, 0},{0, 0, 1}}; 
const float I2_f[3][3] = { {1, 0, 0},{0, 1, 0},{0, 0, 1}}; 

const float R2_f[3][3] = { {0.2f, 0, 0} , {0, 0.2f, 0} , {0, 0, 0.2f}}; // measurement noise
const float Q2_f[3][3] = { {0.02f, 0,0} , {0, 0.02f, 0} , {0, 0, 0.02f} }; //process noise





enum EType {Epuck, Khepera, Elisa};


class CRoboBlob
{
	private:

		CvMat* x_k; //state (x, y, alpha, v, omega) in world coordinates
		

		CvMat* z_k; // Measurements
		CvMat* y_k; // measurement residual, error


		CvMat* P;
		CvMat* F;
		CvMat* F_transposed;
		CvMat* R;
		CvMat* Q;
		CvMat* H;
		CvMat* H_transposed;
		CvMat* I;

//---------------------------------
		CvMat* u_k;
		CvMat* x_k2; //state (x, y, alpha ) in world coordinates
		CvMat* x_p2;

		CvMat* P2;
		CvMat* F2;
		CvMat* F2_transposed;
		CvMat* R2;
		CvMat* Q2;
		CvMat* H2;
		CvMat* H2_transposed;
		CvMat* I2;
//---------------------------------


		DWORD startTime;
		
		EType eType;

		int iStep;

		int comPort;
		float *localPotential;


		boost::posix_time::ptime mst_old, mst_new;
		float EKF_ms;
		
	public:
		CRoboBlob();
		CRoboBlob(EType type);
		~CRoboBlob();

		bool bExists;
		bool bLost;
		bool bIsReady;
		
		int counter;
		float oldTime;
		


		EType GetType() const;
		void SetType(EType type);

		int GetStep() const;
		void SetStep(int step);
		void IncrementStep();

		int GetComPort() const;
		void SetComPort(int comPort);


		void SetInput(float v, float w);
		void SetMeasurement(float x, float y,float alpha); 
		CvMat* GetMeasurement() const;

		void InitState(float x, float y, float alpha);
		CvMat* GetState() const;
		CvMat* GetCovariance() const;
		CvMat* GetState2() const;
		CvMat* GetInput() const;
		CvMat* GetPrediction() const;
		
		void KalmanPredict();
		void KalmanPredict2();
		void KalmanUpdate();
		void KalmanUpdate2();
		float GetEKFsamplingTime();


		static float PixToWorldX(float x);
		static float PixToWorldY(float y);
		static int WorldToPixX(float x);
		static int WorldToPixY(float y);

		void SetLocalPotential(float* localPot);
		float* GetLocalPotential() const;

		void SetStartTime(DWORD startTime);
		DWORD GetStartTime() const;
		static float SetAngleInRange(float alpha);
		static void WriteDataToFile(int comPort, CvMat* Measurement, CvMat* States, CvMat* Covariance, vector<int> MicVolumes);
		static void WriteDataToFile(int comPort, CvMat* Measurement, CvMat* State, CvMat* Covariance);//, float oldTime);
		static void WriteDebugDataToFile(int comPort, CvMat* Measurement, CvMat* States, CvMat* Covariance, vector<float> DebugValues, int nDebugValues);
		static void WritePixStatesDataToFile(int x_pix, int y_pix);
		static void WriteSensorDataToFile(vector<int> vProx);
		static void WriteDataToFile2(int comPort,CvMat* Measurement, CvMat* State, CvMat* Covariance, CvMat* Input, CvMat* Prediction);
};





#endif 