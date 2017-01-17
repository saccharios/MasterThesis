#ifndef __SETPOINTBYUSERGLOB_H__
#define __SETPOINTBYUSERGLOB_H__

#include <CRoboBlob.h>
#include <CVirtualPotential.h>
#include <vector>


vector<float> SetpointByUserGlob(int xClick_pix, int yClick_pix, vector<CRoboBlob*> RoboBlob, int index)
{
	CvMat* blobState = RoboBlob[index]->GetState(); 
	float xRobPos = blobState->data.fl[0]/100; //converting centimeters to meters
	float yRobPos = blobState->data.fl[1]/100; //converting centimeters to meters
	float xClick =  CRoboBlob::PixToWorldX( (float) xClick_pix)/100; //converting pixels to meters
	float yClick =  CRoboBlob::PixToWorldY( (float) yClick_pix)/100; //converting pixels to meters
	
	vector<float> vPosUpdate;
	vPosUpdate.push_back(xRobPos);
	vPosUpdate.push_back(-yRobPos);
	vPosUpdate.push_back(xClick);
	vPosUpdate.push_back(-yClick);
	return vPosUpdate;

}

#endif // __SETPOINTBYUSERGLOB_H__