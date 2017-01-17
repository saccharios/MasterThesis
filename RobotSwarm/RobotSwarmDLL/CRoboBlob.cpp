#include "CRoboBlob.h"

using namespace std;
using namespace boost::numeric;

CRoboBlob::CRoboBlob()
{
	SetStartTime(GetTickCount());

	oldTime = ((float)clock())/1000.0f;
	mst_old = boost::posix_time::microsec_clock::local_time();

	x_k = cvCreateMat( 5, 1, CV_32FC1 );
	z_k = cvCreateMat( 3, 1, CV_32FC1 );
	y_k = cvCreateMat( 3, 1, CV_32FC1 );
	u_k = cvCreateMat( 2, 1, CV_32FC1 );
	SetInput(0.0f,0.0f);


	float P_0[5][5] = { {0, 0, 0, 0,0},{0, 0, 0, 0, 0},{0, 0, 0, 0, 0},{0, 0, 0, 0, 0},{0, 0, 0, 0, 0} }; 
	
	
	P = cvCreateMat( 5, 5, CV_32FC1 );
	F = cvCreateMat( 5, 5, CV_32FC1 );
	F_transposed = cvCreateMat( 5, 5, CV_32FC1 );
	R = cvCreateMat( 3, 3, CV_32FC1 );
	Q = cvCreateMat( 5, 5, CV_32FC1 );
	H = cvCreateMat( 3, 5, CV_32FC1 );
	H_transposed = cvCreateMat( 5, 3, CV_32FC1 );
	I = cvCreateMat( 5, 5, CV_32FC1 );

	//memcpy( x_k->data.fl, x_0, sizeof(x_0));
	memcpy( P->data.fl, P_0, sizeof(P_0));
	memcpy( F->data.fl, F_f, sizeof(F_f));
	//memcpy( F_transposed->data.fl, F_transposed_f, sizeof(F_transposed_f));
	memcpy( R->data.fl, R_f, sizeof(R_f));
	memcpy( Q->data.fl, Q_f, sizeof(Q_f));
	memcpy( H->data.fl, H_f, sizeof(H_f));
	cvTranspose(H,H_transposed);
	//memcpy( H_transposed->data.fl, H_transposed_f, sizeof(H_transposed_f));
	memcpy( I->data.fl, I_f, sizeof(I_f));



	bExists = true;
	bLost = false;
	iStep = 0;
	bIsReady = false;

	comPort = -1;
	localPotential = new float[9];


}

CRoboBlob::CRoboBlob(EType type)
{
	eType = type;
	SetStartTime(GetTickCount());

	oldTime = ((float)clock())/1000.0f;
	mst_old = boost::posix_time::microsec_clock::local_time();


	x_k = cvCreateMat( 5, 1, CV_32FC1 );
	z_k = cvCreateMat( 3, 1, CV_32FC1 );
	y_k = cvCreateMat( 3, 1, CV_32FC1 );
	u_k = cvCreateMat( 2, 1, CV_32FC1 );
	SetInput(0.0f,0.0f);

	// I dont like this kind of assignment of the matrices, but I found no other way
	float P_0[5][5] = { {0.1f, 0, 0, 0,0},{0, 0.1f, 0, 0, 0},{0, 0, 0.1f, 0, 0},{0, 0, 0, 0.1f, 0},{0, 0, 0, 0, 0.1f} }; 
	//float x_0[5][1] = {150, 117, 0, 0, 0}; 

	oldTime = ((float)clock())/1000.0f;

	
	P = cvCreateMat( 5, 5, CV_32FC1 );
	F = cvCreateMat( 5, 5, CV_32FC1 );
	F_transposed = cvCreateMat( 5, 5, CV_32FC1 );
	R = cvCreateMat( 3, 3, CV_32FC1 );
	Q = cvCreateMat( 5, 5, CV_32FC1 );
	H = cvCreateMat( 3, 5, CV_32FC1 );
	H_transposed = cvCreateMat( 5, 3, CV_32FC1 );
	I = cvCreateMat( 5, 5, CV_32FC1 );


	//memcpy( x_k->data.fl, x_0, sizeof(x_0));
	memcpy( P->data.fl, P_0, sizeof(P_0));
	memcpy( F->data.fl, F_f, sizeof(F_f));
	memcpy( F_transposed->data.fl, F_transposed_f, sizeof(F_transposed_f));
	memcpy( R->data.fl, R_f, sizeof(R_f));
	memcpy( Q->data.fl, Q_f, sizeof(Q_f));
	memcpy( H->data.fl, H_f, sizeof(H_f));
	memcpy( H_transposed->data.fl, H_transposed_f, sizeof(H_transposed_f));
	memcpy( I->data.fl, I_f, sizeof(I_f));



	x_k2 = cvCreateMat( 3, 1, CV_32FC1 );
	x_p2 = cvCreateMat( 3, 1, CV_32FC1 );
	x_p2->data.fl[0] = 0;
	x_p2->data.fl[1] = 0;
	x_p2->data.fl[2] = 0;

	// I dont like this kind of assignment of the matrices, but I found no other way
	float P2_0[3][3] = { {0.1f, 0, 0},{0, 0.1f, 0},{0, 0, 0.1f} }; 
	//float x_0[5][1] = {150, 117, 0, 0, 0}; 

	
	
	P2 = cvCreateMat( 3, 3, CV_32FC1 );
	F2 = cvCreateMat( 3, 3, CV_32FC1 );
	F2_transposed = cvCreateMat( 3, 3, CV_32FC1 );
	R2 = cvCreateMat( 3, 3, CV_32FC1 );
	Q2 = cvCreateMat( 3, 3, CV_32FC1 );
	H2 = cvCreateMat( 3, 3, CV_32FC1 );
	H2_transposed = cvCreateMat( 3, 3, CV_32FC1 );
	I2 = cvCreateMat( 3, 3, CV_32FC1 );


	//memcpy( x_k->data.fl, x_0, sizeof(x_0));
	memcpy( P2->data.fl, P2_0, sizeof(P2_0));
	memcpy( F2->data.fl, F2_f, sizeof(F2_f));
	memcpy( F2_transposed->data.fl, F2_transposed_f, sizeof(F2_transposed_f));
	memcpy( R2->data.fl, R2_f, sizeof(R2_f));
	memcpy( Q2->data.fl, Q2_f, sizeof(Q2_f));
	memcpy( H2->data.fl, H2_f, sizeof(H2_f));
	memcpy( H2_transposed->data.fl, H2_transposed_f, sizeof(H2_transposed_f));
	memcpy( I2->data.fl, I2_f, sizeof(I2_f));





	bExists = true;
	bLost = false;
	iStep = 0;
	bIsReady = false;
	counter = 0;

	comPort = -1;
}

CRoboBlob::~CRoboBlob()
{

}

EType CRoboBlob::GetType() const
{
	return eType;
}

void CRoboBlob::SetType(EType type)
{

	eType = type;
}

int CRoboBlob::GetComPort() const
{
	return comPort;
}
		
void CRoboBlob::SetComPort(int port)
{
	comPort = port;
}

void CRoboBlob::SetInput(float v, float w)
{
	u_k->data.fl[0] = v;
	u_k->data.fl[1]= -w;// different coordinate system
}

void CRoboBlob::SetMeasurement(float x, float y, float alpha)
{

	z_k->data.fl[0] = x;
	z_k->data.fl[1] = y;
	z_k->data.fl[2] = alpha;
}

CvMat* CRoboBlob::GetMeasurement() const
{
	return z_k;
}

CvMat* CRoboBlob::GetState() const
{
	if( EKFtype == 2 )
	{
		return x_k2;
	}else
	{	
		return x_k;
	}
}

CvMat* CRoboBlob::GetState2() const
{
	return x_k;
}

CvMat* CRoboBlob::GetInput() const
{
	return u_k;
}

void CRoboBlob::InitState(float x, float y, float alpha)
{// Author: Stefan Frei, 2013
	x_k->data.fl[0] = x;
	x_k->data.fl[1] = y;
	x_k->data.fl[2] = alpha;
	x_k->data.fl[3] = 0;
	x_k->data.fl[4] = 0;

	x_k2->data.fl[0] = x;
	x_k2->data.fl[1] = y;
	x_k2->data.fl[2] = alpha;

}

void CRoboBlob::KalmanPredict( )
{// Author: Stefan Frei, 2013
	//This filter has no inputs, instead v and omega are part of the state


	boost::posix_time::time_duration EKF_time;

	mst_new = boost::posix_time::microsec_clock::local_time();
	EKF_time = mst_new - mst_old;

	// ekf at discrete time instances
	//do {
	//	mst_new = boost::posix_time::microsec_clock::local_time();
	//	EKF_time = mst_new - mst_old;
	//}while( EKF_time.total_milliseconds() < 50);

	////cout << EKF_time.total_milliseconds() << "\t" << clock() << endl;

	mst_old = mst_new;

	EKF_ms = ((float) EKF_time.total_milliseconds() )/1000.0f;
	if (EKF_ms > 1.0f)
	{
		EKF_ms = 0.1f;
	}
	//cout << EKF_ms << endl;

	ofstream file;
	file.open("EFKTime.txt",ios::app|ios::out);		//open a file
	file << setprecision(6);
	file << EKF_ms << endl;
	file.close();	


	CvMat* x_p = cvCreateMat( 5, 1, CV_32FC1 );
	
	//x_p->data.fl[0] = x_k->data.fl[0] - x_k->data.fl[3]*SAMPLING_TIME_CAMERA*cosf(x_k->data.fl[2]);
	//x_p->data.fl[1] = x_k->data.fl[1] - x_k->data.fl[3]*SAMPLING_TIME_CAMERA*sinf(x_k->data.fl[2]);
	//x_p->data.fl[2] = SetAngleInRange(x_k->data.fl[2] + x_k->data.fl[4]*SAMPLING_TIME_CAMERA);
	//x_p->data.fl[3] = x_k->data.fl[3];
	//x_p->data.fl[4] = x_k->data.fl[4];


	//// add non-constant elemtents to F and F'
	//cvmSet(F,0,2,x_k->data.fl[3]*SAMPLING_TIME_CAMERA*sinf(x_k->data.fl[2]));
	//cvmSet(F,0,3,-SAMPLING_TIME_CAMERA*cosf(x_k->data.fl[2]));
	//cvmSet(F,1,2,-x_k->data.fl[3]*SAMPLING_TIME_CAMERA*cosf(x_k->data.fl[2]));
	//cvmSet(F,1,3,-SAMPLING_TIME_CAMERA*sinf(x_k->data.fl[2]));
	//cvmSet(F,2,4,SAMPLING_TIME_CAMERA);

	x_p->data.fl[0] = x_k->data.fl[0] - x_k->data.fl[3]*EKF_ms*cosf(x_k->data.fl[2]);
	x_p->data.fl[1] = x_k->data.fl[1] - x_k->data.fl[3]*EKF_ms*sinf(x_k->data.fl[2]);
	x_p->data.fl[2] = SetAngleInRange(x_k->data.fl[2] + x_k->data.fl[4]*EKF_ms);
	x_p->data.fl[3] = x_k->data.fl[3];
	x_p->data.fl[4] = x_k->data.fl[4];
	

	// add non-constant elemtents to F and F'
	cvmSet(F,0,2,x_k->data.fl[3]*EKF_ms*sinf(x_k->data.fl[2]));
	cvmSet(F,0,3,-EKF_ms*cosf(x_k->data.fl[2]));
	cvmSet(F,1,2,-x_k->data.fl[3]*EKF_ms*cosf(x_k->data.fl[2]));
	cvmSet(F,1,3,-EKF_ms*sinf(x_k->data.fl[2]));
	cvmSet(F,2,4,EKF_ms);

	
	cvTranspose(F,F_transposed);
	
	//CvMat* S1 = cvCreateMat(5,5,CV_32FC1); 

	cvMatMul(F,P,P); // P = F*P
	cvMatMulAdd(P,F_transposed,Q,P); //P(k|k-1) = F(k)*P(k-1|k-1)*F(k)' + Q
	x_k = x_p;

	//cout << "P predict: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;



}

void CRoboBlob::KalmanPredict2( )
{// Author: Stefan Frei, 2013

	boost::posix_time::time_duration EKF_time;

	mst_new = boost::posix_time::microsec_clock::local_time();
	EKF_time = mst_new - mst_old;

	// ekf at discrete time instances
	//do {
	//	mst_new = boost::posix_time::microsec_clock::local_time();
	//	EKF_time = mst_new - mst_old;
	//}while( EKF_time.total_milliseconds() < 50);

	////cout << EKF_time.total_milliseconds() << "\t" << clock() << endl;

	mst_old = mst_new;

	EKF_ms = ((float) EKF_time.total_milliseconds() )/1000.0f;
	if (EKF_ms > 1.0f)
	{
		EKF_ms = 0.1f;
	}
	//cout << EKF_ms << endl;

	ofstream file;
	file.open("EFKTime.txt",ios::app|ios::out);		//open a file
	file << setprecision(6);
	file << EKF_ms << endl;
	file.close();	


	
	
	//x_p2->data.fl[0] = x_k2->data.fl[0] - u_k->data.fl[0]*SAMPLING_TIME_CAMERA*cosf(x_k2->data.fl[2]);
	//x_p2->data.fl[1] = x_k2->data.fl[1] - u_k->data.fl[0]*SAMPLING_TIME_CAMERA*sinf(x_k2->data.fl[2]);
	//x_p2->data.fl[2] = SetAngleInRange(x_k2->data.fl[2] + u_k->data.fl[1]*SAMPLING_TIME_CAMERA);

	//// add non-constant elemtents to f and f'
	//cvmSet(F2,0,2,u_k->data.fl[0]*SAMPLING_TIME_CAMERA*sinf(x_k2->data.fl[2]));
	//cvmSet(F2,1,2,-u_k->data.fl[0]*SAMPLING_TIME_CAMERA*cosf(x_k2->data.fl[2]));



	x_p2->data.fl[0] = x_k2->data.fl[0] - u_k->data.fl[0]*EKF_ms*cosf(x_k2->data.fl[2]);
	x_p2->data.fl[1] = x_k2->data.fl[1] - u_k->data.fl[0]*EKF_ms*sinf(x_k2->data.fl[2]);
	x_p2->data.fl[2] = SetAngleInRange(x_k2->data.fl[2] + u_k->data.fl[1]*EKF_ms);

	// add non-constant elemtents to F and F'
	cvmSet(F2,0,2,u_k->data.fl[0]*EKF_ms*sinf(x_k2->data.fl[2]));
	cvmSet(F2,1,2,-u_k->data.fl[0]*EKF_ms*cosf(x_k2->data.fl[2]));


	
	cvTranspose(F2,F2_transposed);
	
	//CvMat* S1 = cvCreateMat(3,3,CV_32FC1); 

	cvMatMul(F2,P2,P2); // P = F*P
	cvMatMulAdd(P2,F2_transposed,Q2,P2); //P(k|k-1) = F(k)*P(k-1|k-1)*F(k)' + Q
	
	x_k2->data.fl[0] = x_p2->data.fl[0];
	x_k2->data.fl[1] = x_p2->data.fl[1];
	x_k2->data.fl[2] = x_p2->data.fl[2];

	//cout << "P predict: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;



}

void CRoboBlob::KalmanUpdate()
{// Author: Stefan Frei, 2013


	y_k->data.fl[0] = z_k->data.fl[0] - x_k->data.fl[0]; //y(k) = z(k) - H*x_p(k)
	y_k->data.fl[1] = z_k->data.fl[1] - x_k->data.fl[1]; 
	y_k->data.fl[2] = SetAngleInRange(z_k->data.fl[2] - x_k->data.fl[2]);
	
	//cout << "angle difference: " << y_k->data.fl[2] << endl;


	CvMat* T = cvCreateMat(3,5,CV_32FC1); 
	cvMatMul(H,P,T);// T = H*P(k|k-1)

	CvMat* S = cvCreateMat(3,3,CV_32FC1);
	cvMatMulAdd(T,H_transposed,R,S); // S = T*H'+R

	cvInvert(S,S);// S = S^-1
	CvMat* K = cvCreateMat(5,3,CV_32FC1); 
	cvMatMul(P,H_transposed,K); // K = P(k|k-1)*H'
	cvMatMul(K,S,K); // K = P(k|k-1)*H'*(H*P(k|k-1)*H'+R)^-1


	cvMatMulAdd(K,y_k,x_k,x_k); //x(k|k) = x_(k|k-1) + K*y(k)
	SetAngleInRange(x_k->data.fl[2]);

	CvMat* U = cvCreateMat(5,5,CV_32FC1); 
	cvMatMul(K,H,U); // U = K*H
	cvSub(I,U,U); // U = I-U
	cvMatMul(U,P,P); // P(k|k) = (I-K*H)*P(k|k-1) , P = U*P

	//cout << "P: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;
	//cout << "K: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 3; k++)
	//	{
	//		cout << cvmGet(K,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;

	//cvTranspose(U,U); // U = U'
	//cvMatMul(P,U,P); // P = P*U

	//cvTranspose(K,T); // T = K'
	//cvMatMul(R,T,T); //T = R*T
	//cvMatMulAdd(K,T,P,P);

	//cout << "After: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;

	cvReleaseMat( &S );
	cvReleaseMat( &T );
	cvReleaseMat( &U );
	cvReleaseMat( &K );
}

void CRoboBlob::KalmanUpdate2()
{// Author: Stefan Frei, 2013


	y_k->data.fl[0] = z_k->data.fl[0] - x_p2->data.fl[0]; //y(k) = z(k) - H*x_p(k)
	y_k->data.fl[1] = z_k->data.fl[1] - x_p2->data.fl[1]; 
	y_k->data.fl[2] = SetAngleInRange(z_k->data.fl[2] - x_p2->data.fl[2]);
	
	//cout << "angle difference: " << y_k->data.fl[2] << endl;


	CvMat* T2 = cvCreateMat(3,3,CV_32FC1); 
	cvMatMul(H2,P2,T2);// T = H*P(k|k-1)

	CvMat* S2 = cvCreateMat(3,3,CV_32FC1);
	cvMatMulAdd(T2,H2_transposed,R2,S2); // S = T*H'+R

	cvInvert(S2,S2);// S = S^-1
	CvMat* K2 = cvCreateMat(3,3,CV_32FC1); 
	cvMatMul(P2,H2_transposed,K2); // K = P(k|k-1)*H'
	cvMatMul(K2,S2,K2); // K = P(k|k-1)*H'*(H*P(k|k-1)*H'+R)^-1


	cvMatMulAdd(K2,y_k,x_k2,x_k2); //x(k|k) = x_(k|k-1) + K*y(k)
	SetAngleInRange(x_k2->data.fl[2]);

	CvMat* U2 = cvCreateMat(3,3,CV_32FC1); 
	cvMatMul(K2,H2,U2); // U = K*H
	cvSub(I2,U2,U2); // U = I-U
	cvMatMul(U2,P2,P2); // P(k|k) = (I-K*H)*P(k|k-1) , P = U*P

	//cout << "P: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;
	//cout << "K: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 3; k++)
	//	{
	//		cout << cvmGet(K,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;

	//cvTranspose(U,U); // U = U'
	//cvMatMul(P,U,P); // P = P*U

	//cvTranspose(K,T); // T = K'
	//cvMatMul(R,T,T); //T = R*T
	//cvMatMulAdd(K,T,P,P);

	//cout << "After: " << endl;
	//for(int i =0;i < 5; i++)
	//{
	//	for(int k = 0; k < 5; k++)
	//	{
	//		cout << cvmGet(P,i,k) << " ";
	//	}
	//	cout << endl;
	//}
	//cout << endl;

	cvReleaseMat( &S2 );
	cvReleaseMat( &T2 );
	cvReleaseMat( &U2 );
	cvReleaseMat( &K2 );
}

float CRoboBlob::GetEKFsamplingTime()
{
	return EKF_ms;
}

int CRoboBlob::GetStep() const
{
	return iStep;
}

void CRoboBlob::SetStep(int step)
{
	iStep = step; 
}

void CRoboBlob::IncrementStep()
{
	iStep++;
}

CvMat* CRoboBlob::GetCovariance() const
{
	return P;
}

CvMat* CRoboBlob::GetPrediction() const
{
	return x_p2;
}

float CRoboBlob::PixToWorldX(float x)
{// converts pixel to world coordinates (x - cm)

	float x_world = x/WIDTH_PIX * WIDTH_WORLD;
	return x_world;
}

float CRoboBlob::PixToWorldY(float y)
{// converts pixel to world coordinates (y - cm)
	float y_world = y/HEIGHT_PIX * HEIGHT_WORLD;
	return y_world;
}

int CRoboBlob::WorldToPixX(float x)
{
	// converts world coordinates (x - cm) to pixel
	float x_pix_f = x/WIDTH_WORLD * WIDTH_PIX;
	int x_pix = 0;

	if (x_pix_f - (int)x_pix_f < 0.5)
	{
		x_pix = (int)x_pix_f;
	}
	else
	{
		x_pix = (int)x_pix_f + 1;
	}

	return x_pix;
}

int CRoboBlob::WorldToPixY(float y)
{	// converts world coordinates (y - cm) to pixel
	float y_pix_f = y/HEIGHT_WORLD * HEIGHT_PIX;
	int y_pix = 0;

	if (y_pix_f - (int)y_pix_f < 0.5)
	{
		y_pix = (int)y_pix_f;
	}
	else
	{
		y_pix = (int)y_pix_f + 1;
	}

	return y_pix;
}

void CRoboBlob::SetLocalPotential(float* localPot)
{
	localPotential = localPot;

}

float* CRoboBlob::GetLocalPotential() const
{
	return localPotential;
}

void CRoboBlob::WriteDataToFile(int comPort, CvMat* Measurement, CvMat* State, CvMat* Covariance, vector<int> MicVolumes)
{
	double timestamp = ((double)GetTickCount()/1000.0);
	ofstream file;
	file.open("DataMeasStateCovMicLog.txt",ios::app|ios::out);		//open a file
	file << comPort << "\t";	
	file << setprecision(16) << timestamp << "\t";


	file << Measurement->data.fl[0] << "\t";
	file << Measurement->data.fl[1] << "\t";
	file << State->data.fl[0] << "\t";
	file << State->data.fl[1] << "\t";
	file << State->data.fl[2] << "\t";
	file << State->data.fl[3] << "\t";
	file << Covariance->data.fl[0] << "\t";
	file << Covariance->data.fl[1] << "\t";
	file << Covariance->data.fl[2] << "\t";
	file << Covariance->data.fl[3] << "\t";
	file << Covariance->data.fl[4] << "\t";
	file << Covariance->data.fl[5] << "\t";
	file << Covariance->data.fl[6] << "\t";
	file << Covariance->data.fl[7] << "\t";
	file << Covariance->data.fl[8] << "\t";
	file << Covariance->data.fl[9] << "\t";
	file << Covariance->data.fl[10] << "\t";
	file << Covariance->data.fl[11] << "\t";
	file << Covariance->data.fl[12] << "\t";
	file << Covariance->data.fl[13] << "\t";
	file << Covariance->data.fl[14] << "\t";
	file << Covariance->data.fl[15] << "\t";
	if (MicVolumes.size()==3)
	{
		file << MicVolumes[0] << "\t";
		file << MicVolumes[1] << "\t";
		file << MicVolumes[2] << "\n";
	} else {
		file << "0\t0\t0\n";
	}
	file.close();	
}

void CRoboBlob::WriteDebugDataToFile(int comPort, CvMat* Measurement, CvMat* States, CvMat* Covariance, vector<float> DebugValues, int nDebugValues)
{
	double timestamp = ((double)GetTickCount()/1000.0);
	ofstream file;
	file.open("debugDataSheet.txt",ios::app|ios::out);		//open a file
	file << comPort << "\t";	
	file << setprecision(16) << timestamp << "\t";
	file << States->data.fl[0] << "\t";
	file << States->data.fl[1] << "\t";
	file << States->data.fl[2] << "\t";
	file << States->data.fl[3] << "\t";
	if (DebugValues.size()==nDebugValues)
	{
		for (int i=0; i<nDebugValues;i++)
		{
			if (i<(nDebugValues-1))
			{
				file << DebugValues[i] << "\t";
			} else
			{
				file << DebugValues[i] << "\n";
			}
		}
	} else {
		for (int i=0; i<nDebugValues;i++)
		{
			if (i<(nDebugValues-1))
			{
				file << "0\t";
			} else
			{
				file << "0\n";
			}
		}
	}

	file.close();	
}

void CRoboBlob::WriteDataToFile(int comPort,CvMat* Measurement, CvMat* State, CvMat* Covariance)//, float oldTime)
{// Author: Stefan Frei, 2013

	ofstream file;
	file.open("DataMeasStateCovLog.txt",ios::app|ios::out);		//open a file
	//file << comPort << endl;	


	file << setprecision(4);


	//Prediction for the next step
	CvMat* x_p = cvCreateMat( 5, 1, CV_32FC1 );
	x_p->data.fl[0] = State->data.fl[0] - State->data.fl[3]*SAMPLING_TIME_CAMERA*cosf(State->data.fl[2]);
	x_p->data.fl[1] = State->data.fl[1] - State->data.fl[3]*SAMPLING_TIME_CAMERA*sinf(State->data.fl[2]);
	x_p->data.fl[2] = SetAngleInRange(State->data.fl[2] + State->data.fl[4]*SAMPLING_TIME_CAMERA);
	x_p->data.fl[3] = State->data.fl[3];
	x_p->data.fl[4] = State->data.fl[4];



	// works only for 5x1 State, 3x1 Measurement
	for(int i = 0; i < 5; i++)
	{
		if( i < 3)
		{
			file << Measurement->data.fl[i] << "\t\t\t" << State->data.fl[i] << "\t\t\t" << x_p->data.fl[i] << "\t\t\t";
			for(int k = 0; k < 5; k++)
			{
				file << cvmGet(Covariance,i,k) << "\t\t\t";
			}

		}else{
			file << -1 << "\t\t\t" << State->data.fl[i] << "\t\t\t" << x_p->data.fl[i] << "\t\t\t";
			for(int k = 0; k < 5; k++)
			{
				file << cvmGet(Covariance,i,k) << "\t\t\t";
			}

		}
		file << endl;
	}
	file << endl << endl;

	
	file.close();	


}

void CRoboBlob::WriteDataToFile2(int comPort,CvMat* Measurement, CvMat* State, CvMat* Covariance, CvMat* Input, CvMat* Prediction)
{
	ofstream file;
	file.open("DataMeasStateCovLog2.txt",ios::app|ios::out);		//open a file
	//file << comPort << endl;	

	file << setprecision(4);


	// works only for 3x1 State, 3x1 Measurement
	for(int i = 0; i < 5; i++)
	{
		if( i < 3)
		{
			file << Measurement->data.fl[i] << "\t\t\t" << State->data.fl[i] << "\t\t\t" << Prediction->data.fl[i] << "\t\t\t";
			for(int k = 0; k < 5; k++)
			{
				file << cvmGet(Covariance,i,k) << "\t\t\t";
			}

		}else{
			file << Input->data.fl[i-3] << "\t\t\t" << -1 << "\t\t\t" << -1 << "\t\t\t";
			for(int k = 0; k < 5; k++)
			{
				file << cvmGet(Covariance,i,k) << "\t\t\t";
			}

		}
		file << endl;
	}
	file << endl << endl;

	
	file.close();	

}


void CRoboBlob::WritePixStatesDataToFile(int x_pix, int y_pix)
{
	ofstream file;
	file.open("statesDataSheet.txt",ios::app|ios::out);		//open a file
	
	file << x_pix << "\t";
	file << y_pix << "\n";

	file.close();				//close it
}

void CRoboBlob::WriteSensorDataToFile(vector<int> vProx)
{
	ofstream file;
	file.open("statesDataSheet.txt",ios::app|ios::out);		//open a file

	for(size_t i = 0; i < vProx.size(); i++)
	{
		file << vProx[i] << "\t";
	}

	file << "\n";

	file.close();				//close it
}

void CRoboBlob::SetStartTime(DWORD time)
{
	startTime = time;
}

DWORD CRoboBlob::GetStartTime() const
{
	return startTime;
}



float CRoboBlob::SetAngleInRange(float alpha)
{// Author: Stefan Frei, 2013
	// wraps alpha into [-pi, pi]

	if( alpha > PI)
	{
		alpha -= 2*PI;
	}else if (alpha <= -PI)
	{
		alpha += 2*PI;
	}
	return alpha;
}


