#ifndef _CVIRTUALPOTENTIAL_H_
#define _CVIRTUALPOTENTIAL_H_

#include "CRoboBlob.h"


//in pixels
const int WALL_START_X = 50;
const int WALL_END_X  = 550;
const int WALL_START_Y = 100;
const int WALL_END_Y = 350;
const float WALL_THICKNESS_FACTOR = 12.0f;

const float MAX_POTENTIAL_ROBOT = 100.0f;
const float MAX_POTENTIAL_WALL = 100.0f;
const float VARIANCE_X = 60.0f;
const float VARIANCE_Y = 60.0f;

const int ACTIVE_WINDOW_SIZE = 33; //potential neighbourhood of a robot
const int STEERING_FACTOR = 5;

const float MAX_SPEED = 12.88f;
const int BUFSIZE = 256;

class CVirtualPotential
{
private:
	float gradients[8]; // {W,NW,N,NE,E,SE,S,SW}
	float potentialMatrix[WIDTH_PIX][HEIGHT_PIX]; // make dynamic with pixels of camera
	float localPotential[9];
	float activeWindow[ACTIVE_WINDOW_SIZE][ACTIVE_WINDOW_SIZE];
	vector<float> vVirtualForce;

	int meanX;
	int meanY;
	int varianceX;
	int varianceY;

	int wallThickness;


public:
	CVirtualPotential();
	~CVirtualPotential();
	void ComputeGradients(float *localpotential); 
	void CreatePotentialWall();
	float *ExtractLocalPotentialField(float *localpot, int x_pix, int y_pix);
	float GaussianDistribution(int iMeanX, int iMeanY, int x, int y); //the mean values here are determined by the position of the other robots
	void CreateVirtualForceField(int x, int y, vector<CRoboBlob*> &vObstacle); //create potential field around the point x,y
	//void PlotMoveDecision();
	void RestorePotentialField(int x_other_pix, int y_other_pix, int x, int y, float *localPot);

	vector<float> MoveDecision(float stateAngle, float deltaTime, float wheelDistance); // -1: turn left, 0: don't turn, 1: turn right 
	void ComputeTotalField(int x_pix, int y_pix);

};

#endif // _CVIRTUALPOTENTIAL_H_