#ifndef __SETPOINTBYUSER_H__
#define __SETPOINTBYUSER_H__

#include <CRoboBlob.h>
#include <CVirtualPotential.h>
#include <vector>

int TsSetpoint = 1000; //[ms] Period of setpoint updates. Used in RobotSwarmDLL.cpp

vector<float> SetpointByUser(int xClick_pix, int yClick_pix, vector<CRoboBlob*> RoboBlob, int index)
{
	CvMat* blobStates = RoboBlob[index]->GetState(); 
	float xRobPos = blobStates->data.fl[0]/100; //converting centimeters to meters
	float yRobPos = blobStates->data.fl[1]/100; //converting centimeters to meters
	float T = 0.33f;
	//float xRobVel = -blobStates->data.fl[3]*T*cosf(blobStates->data.fl[2]);
	//float yRobVel = -blobStates->data.fl[3]*T*sinf(blobStates->data.fl[2]);
	float xClick = CRoboBlob::PixToWorldX( (float) xClick_pix)/100; //converting pixels to meters
	float yClick = CRoboBlob::PixToWorldY( (float) yClick_pix)/100; //converting pixels to meters

	float d = sqrt(pow((xRobPos-xClick),2)+pow((yRobPos-yClick),2));
	float alpha = atan2(-(yClick-yRobPos),(xClick-xRobPos)) - (PI - blobStates->data.fl[2]);	// y-axis is flipped in Camera's coordinate system (positive y points downwards, point [0,0] is the top left corner of the image); 
																								// blobStates->data.fl[2] is the global orientation angle of the robot, it goes clockwise and 0 angle is pointing in negative x direction

	//vector<float> vVelocity;
	//vVelocity.push_back(0.7*MAX_SPEED);
	//vVelocity.push_back(0.9*MAX_SPEED);
	//return vVelocity;
	
	vector<float> vSetpoint;
	vSetpoint.push_back(d);
	vSetpoint.push_back(alpha);
	return vSetpoint;

}

#endif // __SETPOINTBYUSER_H__