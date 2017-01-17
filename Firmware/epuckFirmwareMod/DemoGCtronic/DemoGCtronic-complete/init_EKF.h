// Memory allocation and initialization of matrices used for the extended kalman filter in "runcontroldrive" demo
// By ATr (talex@student.ethz.ch)

// Parameters

// EKF
static const float par_EKF_min_d = 0.012; //=v_max*Ts/(pi/2); //0.05;  //[m] Avoid singularity at d=0 -->  d>=par_EKF_min_d for the sake of calculating EKF
// Rejecting unexpected measurements
static unsigned int par_RejectAngles = 0; //flag for rejecting angle measurement, which are not at all consistent with current estimate
static const float par_RejectAngleThres = 0.78; // [rad] if flag above is set, reject all angle measurements, which are further than par_RejectAngleThres radiants away from current estimate
static const unsigned int par_CapDist = 0;   // flag for saturating distance measurements
static const float par_CapDistThres = 0.30;  // [m] if flag above is set, saturate all distance measurements, which are larger than par_CapDistThres meters to the value par_CapDistValue
static const float par_CapDistValue = 0.50;	 // [m]

//ePuck
static const float par_v_max = 0.126; // Maximal speed [m/s];
static const float par_b_wheels = 0.053; //Distance between wheels [m]
//Microphones and sound
static const float par_micPosA0 = -1.22;  //[rad] angle from robot's heading to mic0
static const float par_micPosA1 = 1.22;   //[rad] angle from robot's heading to mic1
static const float par_micPosA2 = -3.14;  //[rad] angle from robot's heading to mic2
static const float par_micPosR0 = 0.033;  //[m] distance from robot's center to mic0
static const float par_micPosR1 = 0.033;  //[m] distance from robot's center to mic1
static const float par_micPosR2 = 0.0192; //[m] distance from robot's center to mic2

static const float par_micPosX[] = {0.0113, 0.0113,-0.0192};   //[m] x-coordinates of microphones relative to robot's center
static const float par_micPosY[] = {-0.031, 0.031, 0.0};       //[m] y-coordinates of microphones relative to robot's center


static const float par_mic20a = 0.0435;   //[m] distance between mic2 and mic0
static const float par_mic21a = 0.0435;	  //[m] distance between mic2 and mic1
static const float par_mic20beta = -0.7935; //[rad] angle between robots heading and vector mic2->mic0
static const float par_mic21beta = 0.7935;	//[rad] angle between robots heading and vector mic2->mic1
static const float par_d2xlim = 1.0;     // [0..1] distance traveled by sound between mic2 and micx is saturated at par_d2xlim*par_mic2xa to avoid numerical problems (such as acos(x) for x>1)
static const float par_speedSound = 340.0;	//[m/s] Speed of sound in the air 
static const float par_freqToDetect[] = {515.6, 1418.0, 1675.8, 1933.6, 2191.4}; //[Hz] Signal frequencies
static const unsigned int par_freqToDetect_id[] = {4,11,13,15,17}; //[-] id of the frequency of interest in fft array. (
//static const unsigned int par_signal_id = 2; //[0..4] id of the signal of interest in par_freqToDetect array.
//static const float par_freqToDetect = 900.0; //[Hz] Frequency of interest


// Noise
static const float par_sigma_input = 0.1; //[0..1] Stand.Dev. of input noise. Units: normalized motor input 
static const float par_sigma_srcSpeed = 0.126; //0.02;  //[m/s] Stand.Dev. of  noise source's movement speed.
//static const float par_sigma_srcAccel = 0.05; //1.0; //[m/s^2] Stand.Dev. of  noise source's movement speed.
static const float par_sigma_micAmp = 2.0;    //[amplitude] Stand.Dev. of measurement noise (amplitude of frequency of interest), inlk. numerical errors during FFT.
static const float par_sigma_Calib = 1;    //[1/m] Stand.Dev. of noise used for microphone calibration
static const float par_sigma_Mic_d = 0.05;  //[m] Stand.Dev. of the error on distance, as predicted from mic amplitudes.
static const float par_sigma_Mic_alpha = 0.5;  //[rad] Stand.Dev. of the error on angle, as predicted from phase shifts.
static const float par_sigma_Cam_d = 0.01; //0.03;    //[m] Stand.Dev. of distance measurement with camera. 
static const float par_sigma_Cam_alpha = 0.05; //0.2; //[rad] Stand.Dev. of angle measurement with camera.
//static const float par_sigma_d2x = 0.01; //[m] Stand.Dev. of error on distance traveled by sound between mic2 and micx (it is in principle a phase shift, but independent of frequency)
static const float par_sigma_Add_d = 0.01; //[m] Stand.Dev of additive process noise to distance state
static const float par_sigma_Add_alpha = 0.09; //[rad] Stand.Dev of additive process noise to angle state

// Matrix sizes
#define MSIZE_N 4 // Number of EKF states
#define MSIZE_M 2 // Number of inputs
#define MSIZE_P 4 // Number of process noise sources
#define MSIZE_R 2 // Number of measurement variables
#define MSIZE_R4 2 //Number of communicated measurement variables 
#define MSIZE_T 2 // Number of measurement noise sources
#define MSIZE_MAX 4 // Maximum allowed matrix dimension
// Number of time steps the measurement update gets delayed + 1 (i.e. this constant is 1 if no delay). States and EKF matrices P, F and K from past N time steps have to be stored
#define NSTORESTEPS 3 // !!! May not be smaller than 1 or larger than 4

// Number of signals to detect in FFT
#define NSIG 2


/*
// Matrix alignment sizes (same as above, but rounded to the next power of 2)
#define MSIZE_N_AL 2 // Number of EKF states
#define MSIZE_M_AL 2 // Number of inputs
#define MSIZE_P_AL 4 // Number of process noise sources
#define MSIZE_R_AL 2 // Number of measurement variables
#define MSIZE_T_AL 2 // Number of measurement noise sources
#define MSIZE_MAX_AL 4 // Highest of the sizes above
*/



// Memory allocation
/*
// Measurement update of calibration
float CalParamMic0_x0[] = {1.60, 2.50, -0.1, -1.5, 0.55}; //0.0;
float CalParamMic0_x1[] = {1.00, 0.18, 0.75, 0.89, 0.93}; //1.1;
float CalParamMic0_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic0_P[MSIZE_MAX*MSIZE_MAX];

float CalParamMic1_x0[] = {1.60, 2.50, -0.1, -1.5, 0.55}; //0.0;
float CalParamMic1_x1[] = {1.00, 0.18, 0.75, 0.89, 0.93}; //1.1;
float CalParamMic1_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic1_P[MSIZE_MAX*MSIZE_MAX];

float CalParamMic2_x0[] = {1.60, 2.90, 2.60, 2.20, 2.00}; //0.0;
float CalParamMic2_x1[] = {0.76, 0.20, 0.39, 0.49, 0.66}; //1.1;
float CalParamMic2_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic2_P[MSIZE_MAX*MSIZE_MAX];
*/

// Measurement update of calibration
float CalParamMic0_x0[] = {1.60, 0.00, -0.1, -1.5, 0.55}; //0.0;
float CalParamMic0_x1[] = {1.00, 1.00, 0.75, 0.89, 0.93}; //1.1;
float CalParamMic0_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic0_P[MSIZE_MAX*MSIZE_MAX];

float CalParamMic1_x0[] = {1.60, 0.00, -0.1, -1.5, 0.55}; //0.0;
float CalParamMic1_x1[] = {1.00, 1.00, 0.75, 0.89, 0.93}; //1.1;
float CalParamMic1_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic1_P[MSIZE_MAX*MSIZE_MAX];

float CalParamMic2_x0[] = {1.60, 1.50, 2.60, 2.20, 2.00}; //0.0;
float CalParamMic2_x1[] = {0.76, 0.78, 0.39, 0.49, 0.66}; //1.1;
float CalParamMic2_P[] = {1.0, 0.0, 0.0, 1.0}; //2x2 covariance matrix of calibration parameters. //float CalParamMic2_P[MSIZE_MAX*MSIZE_MAX];

// Matrices for Calibration
float Cal_R[1];    // [1x1] Covariance matrix of measurement noise;
float Cal_H[2];    // [1x2] Jacobian of measurement modell with respect to states (time variable)



//Camera update of source position (Hardcoded: 2 sources)
float SrcPosMeasCam_d[] = {0.0,0.0} ; 											// direct distance measurement, e.g. update from camera system
float SrcPosMeasCam_alpha[] = {0.0,0.0}; 										// direct angle measurement, e.g. update from camera system
int SrcPosMeasCam_new[] = {0,0};

//Communicated updates of source position from other members of the formation (Hardcoded: up to 3 updates)
//float SrcPosMeasComm_phi[] = {0.0,0.0,0.0};
//int SrcPosMeasComm_new[] = {-1,-1,-1};
int SrcPosMeasComm_new = 0;
float SrcPosMeasComm_x[] = {0.0,0.0,0.0};
float SrcPosMeasComm_y[] = {0.0,0.0,0.0};

//Measurement update of source position estimator 
int SrcPosMeas_new[NSIG]; 												// Flag for new measurement
float MicFreqMeasAmp[NSIG*3];	   											// Output of CaptureFFT function - estimated amplitude at certain frequency	
float MicFreqMeasPhase[NSIG*3];	   										// Output of CaptureFFT function - estimated phase at certain frequency	
float RobPosMeas_x = 0.0; 											// direct position measurement (x), e.g. update from camera system
float RobPosMeas_y = 0.0; 											// direct position measurement (y), e.g. update from camera system
int RobPosMeas_new = 0;												// Flag for new measurement


//State Estimator
//float SrcPosEst_d = 0.15; // (Estimated) 
//float SrcPosEst_alpha = 0.0; // (Estimated) Relative angle to the setpoint [rad] (e.g. 0 = in front of robot, +pi/2 = to the left of the robot, -pi/2 = to the right of the robot, etc.)
float x_m[MSIZE_N*NSTORESTEPS];
float EKF_P[MSIZE_N*MSIZE_N*NSTORESTEPS]   ;//__attribute__  ((space(xmemory)));      // [NxN] Covariance matrix of state estimates (d, alpha)
//Matrices for EKF
float EKF_Q[MSIZE_P*MSIZE_P]  ;//__attribute__  ((space(xmemory)));  		    // [PxP] Covariance matrix of process noise; diag(left wheel noise ,right wheel noise)
float EKF_F[MSIZE_N*MSIZE_N*NSTORESTEPS]  ;//__attribute__  ((space(xmemory)));				// [NxN] Jacobian of process modell with respect to states (time variable)
float EKF_L[MSIZE_N*MSIZE_P]  ;//__attribute__  ((space(xmemory)));				// [NxP] Jacobian of process modell with respect to noise (time variable)
float EKF_R0[MSIZE_T*MSIZE_T]  ;//__attribute__  ((space(xmemory)));  	        // [TxT] Covariance matrix of measurement noise; 
float EKF_R1[MSIZE_T*MSIZE_T]  ;//__attribute__  ((space(xmemory)));  	        // [TxT] Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
float EKF_R2[MSIZE_T*MSIZE_T]  ;//__attribute__  ((space(xmemory)));  	        // [TxT] Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
float EKF_R3[MSIZE_T*MSIZE_T]  ;//__attribute__  ((space(xmemory)));  	        // [TxT] Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
float EKF_R4[MSIZE_R4*MSIZE_R4]  ;//__attribute__  ((space(xmemory)));  	    // [R4xR4] Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
float EKF_H0[MSIZE_R*MSIZE_N]  ;//__attribute__  ((space(xmemory)));			// [RxN] Jacobian of measurement modell with respect to states (time variable)
float EKF_H1[MSIZE_R*MSIZE_N]  ;//__attribute__  ((space(xmemory)));			// [RxN] Jacobian of measurement modell with respect to states (time variable)
float EKF_H2[MSIZE_R*MSIZE_N]  ;//__attribute__  ((space(xmemory)));			// [RxN] Jacobian of measurement modell with respect to states (time variable)
float EKF_H3[MSIZE_R*MSIZE_N]  ;//__attribute__  ((space(xmemory)));			// [RxN] Jacobian of measurement modell with respect to states (time variable)
float EKF_H4[MSIZE_R4*MSIZE_N]  ;//__attribute__  ((space(xmemory)));			// [R4xN] Jacobian of measurement modell with respect to states (time variable)
float EKF_K[MSIZE_N*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));				// [NxR] Kalman gain matrix
float EKF_D[MSIZE_N*MSIZE_N]  ;//__attribute__  ((space(xmemory)));				// [NxN] Delay correction matrix
float EKF_TEMP1[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations
float EKF_TEMP2[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations
float EKF_TEMP3[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations


/*
float EKF_P[MSIZE_MAX*MSIZE_MAX]   ;//__attribute__  ((space(xmemory)));      // [NxN] 2x2 Covariance matrix of state estimates (d, alpha)
//Matrices for EKF
float EKF_Q[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));  		    // [PxP] 4x4 Covariance matrix of process noise; diag(left wheel noise ,right wheel noise)
float EKF_F[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));				// [NxN] 2x2 Jacobian of process modell with respect to states (time variable)
float EKF_L[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));				// [NxP] 2x4 Jacobian of process modell with respect to noise (time variable)
float EKF_R[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));  	        // [TxT] 3x3 Covariance matrix of measurement noise; diag(mic1 noise ,mic2 noise, mic3 noise)
float EKF_H[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));			    // [RxN] 3x2 Jacobian of measurement modell with respect to states (time variable)
//float EKF_M[MSIZE_R*MSIZE_T]  ;//__attribute__  ((space(xmemory)));	        // [RxT] 3x3 Jacobian of measurement modell with respect to noise -> trivial diag(1,1,1);
float EKF_K[MSIZE_MAX*MSIZE_MAX]  ;//__attribute__  ((space(xmemory)));				// [NxR] 2x3 Kalman gain matrix
float EKF_TEMP1[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations
float EKF_TEMP2[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations
float EKF_TEMP3[MSIZE_MAX*MSIZE_MAX] ;//__attribute__ ((space(xmemory)));	    // temporary storage for intermediate values in matrix calculations
*/
//void init_EKF_values(void);

void Tr_MatrixMultiply(unsigned int nrowA,unsigned int ncolArowB, unsigned int ncolB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixAdd(unsigned int nrowAB,unsigned int ncolAB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixSubtract(unsigned int nrowAB,unsigned int ncolAB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixTranspose(unsigned int nrowA, unsigned int ncolA, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixSubtractFromId (unsigned int nrowAcolA, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX]);
int Tr_MatrixInverse3x3(float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX]);
int Tr_MatrixInverse2x2(float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX]);
void Tr_MatrixTimeShift(unsigned int nrowA, unsigned int ncolA, unsigned int N_TimeSteps, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX]);
//Matching filter, which is used to estimate the strength of a signal (Certain frequency played badly by another ePuck)
//Coefficients only for the frequency of interest and its odd multiples, 
// e.g. frequenciy 7 is 902Hz, so the coefficients to detect this frequency would 
// correspond to fft-frequencies 7,21,35,49,63,77 (odd multiples of 7th frequency)
const float filters_short[] = 
{
//1x	  3x	   5x		7x		9x	   	11x
/*
1.00,    0.00,    0.00,    0.00,    0.00,    0.00, // Signal 0:  515.6 Hz
1.00,    0.00,    0.00,    0.00,    0.00,    0.00, // Signal 1: 1418.0 Hz
1.00,    0.00,    0.00,    0.00,    0.00,    0.00, // Signal 2: 1675.8 Hz
1.00,    0.00,    0.00,    0.00,    0.00,    0.00, // Signal 3: 1933.6 Hz
1.00,    0.00,    0.00,    0.00,    0.00,    0.00  // Signal 4: 2191.4 Hz
*/
0.50,    0.10,    0.10,    0.10,    0.10,    0.10, // Signal 0:  515.6 Hz
0.50,    0.10,    0.10,    0.10,    0.10,    0.10, // Signal 1: 1418.0 Hz
0.50,    0.10,    0.10,    0.10,    0.10,    0.10, // Signal 2: 1675.8 Hz
0.50,    0.10,    0.10,    0.10,    0.10,    0.10, // Signal 3: 1933.6 Hz
0.50,    0.10,    0.10,    0.10,    0.10,    0.10  // Signal 4: 2191.4 Hz

};

/*
//Matching filter, which is used to estimate the strength of a signal (900Hz played by another ePuck)
//Coefficients only for frequencies 7,21,35,49,63,77 (odd multiples of 7th frequency, which is 902Hz)
const float filter_detect900Hz_short[] = 
{
0.0891,    0.0867,    0.2475,    0.2205,    0.0684,    0.0436
};
*/
//Coefficients for all 256 frequencies -> hits the limit of computation power, shouldn't be used
/*
const float filter_detect900Hz[] = 
{
0.0033923,
0.0032249,
0.003488,
0.0022603,
0.001529,
0.001356,
0.0014876,
0.022206,
0.0015922,
0.0012721,
0.001123,
0.001003,
0.0010111,
0.0010681,
0.0016208,
0.0010319,
0.0010892,
0.0010788,
0.001276,
0.0014274,
0.0021717,
0.021595,
0.0025413,
0.0016246,
0.0015155,
0.0014534,
0.0014767,
0.0015605,
0.0030513,
0.0019631,
0.0022313,
0.0026086,
0.0032285,
0.0046951,
0.0086209,
0.061664,
0.011682,
0.0054557,
0.0036388,
0.0029207,
0.0024624,
0.0023072,
0.0029907,
0.0024381,
0.0026359,
0.0030999,
0.0039058,
0.0054975,
0.010011,
0.054926,
0.015339,
0.0067243,
0.0044044,
0.0032552,
0.0026366,
0.0022522,
0.0044387,
0.0023794,
0.0019179,
0.001831,
0.0020056,
0.00248,
0.0039052,
0.017049,
0.0067838,
0.0030063,
0.0021067,
0.0016571,
0.0014122,
0.0013102,
0.0018071,
0.0014283,
0.0012697,
0.0013349,
0.0014571,
0.0018286,
0.0028793,
0.010871,
0.0057578,
0.0024283,
0.0017035,
0.0013214,
0.0011929,
0.001144,
0.0021042,
0.0016363,
0.0011276,
0.0011358,
0.0012149,
0.0014654,
0.0022099,
0.0070636,
0.0048376,
0.0019719,
0.0015801,
0.0011588,
0.0010146,
0.001027,
0.0013552,
0.0012556,
0.0010232,
0.0010426,
0.001154,
0.0013834,
0.0021084,
0.0062309,
0.0055351,
0.0020541,
0.0015342,
0.0011988,
0.0010601,
0.0010905,
0.0019495,
0.002047,
0.001088,
0.0011116,
0.0011658,
0.001383,
0.0019189,
0.0049768,
0.0056042,
0.0019117,
0.0014236,
0.0011069,
0.00099846,
0.0010382,
0.0014085,
0.0016795,
0.00096388,
0.0016795,
0.0014085,
0.0010382,
0.00099846,
0.0011069,
0.0014236,
0.0019117,
0.0056042,
0.0049768,
0.0019189,
0.001383,
0.0011658,
0.0011116,
0.001088,
0.002047,
0.0019495,
0.0010905,
0.0010601,
0.0011988,
0.0015342,
0.0020541,
0.0055351,
0.0062309,
0.0021084,
0.0013834,
0.001154,
0.0010426,
0.0010232,
0.0012556,
0.0013552,
0.001027,
0.0010146,
0.0011588,
0.0015801,
0.0019719,
0.0048376,
0.0070636,
0.0022099,
0.0014654,
0.0012149,
0.0011358,
0.0011276,
0.0016363,
0.0021042,
0.001144,
0.0011929,
0.0013214,
0.0017035,
0.0024283,
0.0057578,
0.010871,
0.0028793,
0.0018286,
0.0014571,
0.0013349,
0.0012697,
0.0014283,
0.0018071,
0.0013102,
0.0014122,
0.0016571,
0.0021067,
0.0030063,
0.0067838,
0.017049,
0.0039052,
0.00248,
0.0020056,
0.001831,
0.0019179,
0.0023794,
0.0044387,
0.0022522,
0.0026366,
0.0032552,
0.0044044,
0.0067243,
0.015339,
0.054926,
0.010011,
0.0054975,
0.0039058,
0.0030999,
0.0026359,
0.0024381,
0.0029907,
0.0023072,
0.0024624,
0.0029207,
0.0036388,
0.0054557,
0.011682,
0.061664,
0.0086209,
0.0046951,
0.0032285,
0.0026086,
0.0022313,
0.0019631,
0.0030513,
0.0015605,
0.0014767,
0.0014534,
0.0015155,
0.0016246,
0.0025413,
0.021595,
0.0021717,
0.0014274,
0.001276,
0.0010788,
0.0010892,
0.0010319,
0.0016208,
0.0010681,
0.0010111,
0.001003,
0.001123,
0.0012721,
0.0015922,
0.022206,
0.0014876,
0.001356,
0.001529,
0.0022603,
0.003488,
0.0032249
} ; 
*/