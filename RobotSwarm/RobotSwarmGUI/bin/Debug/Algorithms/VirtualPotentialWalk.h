#ifndef __VIRTUALPOTENTIALWALK_H__
#define __VIRTUALPOTENTIALWALK_H__

#include <CRoboBlob.h>
#include <CVirtualPotential.h>


vector<float> VirtualPotentialWalk(CVirtualPotential *VirtualPotential, vector<CRoboBlob*> RoboBlobs, int index) //CRoboBlob* roboBlob)
{

	CvMat* blobState = RoboBlobs[index]->GetState(); 
	//int decision;
	//float *localpotential = new float [9];
	float x = blobState->data.fl[0];
	float y = blobState->data.fl[1];
	int x_pix = CRoboBlob::WorldToPixX(x);
	int y_pix = CRoboBlob::WorldToPixY(y);



	if (x_pix > WIDTH_PIX - 1)
	{
		x_pix = WIDTH_PIX - 1;
	}
	else if	(x_pix < 1)
	{
		x_pix = 1;
	}

	if (y_pix > HEIGHT_PIX - 1)
	{
		y_pix = HEIGHT_PIX - 1;
	}
	else if	(y_pix < 1)
	{
		y_pix = 1;
	}

	//localpotential = VirtualPotential->ExtractLocalPotentialField(localpotential, x_pix, y_pix);

	//cout << "blobStates: " << blobStates->data.fl[0] << ", " << blobStates->data.fl[1] << endl;

	vector<CRoboBlob*> vObstacle;

	//cout << "RoboBlob.size(): " << RoboBlob.size() << endl;
	//cout << "index: " << index << endl;

	for(size_t i = 0; i < RoboBlobs.size(); i++)
	{
		//CvMat* states =  RoboBlobs[i]->GetState();
		//float x_obstacle = states->data.fl[0];
		//float y_obstacle = states->data.fl[1];

		if(i != index)
		{
			//int x_obstacle_pix = CRoboBlob::WorldToPixX(x_obstacle);
			//int y_obstacle_pix = CRoboBlob::WorldToPixY(y_obstacle);
			vObstacle.push_back(RoboBlobs[i]);
			//vObstacleX.push_back(x_obstacle_pix);
			//vObstacleY.push_back(y_obstacle_pix);

			//VirtualPotential->ComputeTotalField(x_pix,y_pix);
			
			//cout << "CreatePotential, x: " << x << ", y: " << y << ", xpix: " << x_pix << ", yPix: " << y_pix << endl;
			
			//RoboBlob[i]->SetLocalPotential(localPot);
		}
	}

	VirtualPotential->CreateVirtualForceField(x_pix, y_pix, vObstacle);
	
//	VirtualPotential->ComputeGradients(localpotential);
	float angle = blobState->data.fl[2];

	DWORD startTick = RoboBlobs[index]->GetStartTime();
	DWORD endTick = GetTickCount();
	DWORD diffTick = (endTick - startTick) ;

	float fDiffTick = ((float)diffTick)/1000;
	//cout << "fDiffTick: " << fDiffTick << endl;

	RoboBlobs[index]->SetStartTime(GetTickCount());
	float wheelDistance;

	if(RoboBlobs[index]->GetType() == Epuck)
	{
		wheelDistance = WHEEL_DISTANCE_EPUCK;
	}
	else
	{
		wheelDistance = WHEEL_DISTANCE_KHEPERA;
	}

	vector<float> vVelocity = VirtualPotential->MoveDecision(angle, fDiffTick, wheelDistance);
	//cout << "decision: " << decision << endl;

	/*for(int i = 0; i < RoboBlob.size(); i++)
	{
		CvMat* states =  RoboBlob[i]->GetState();
		float x_other = states->data.fl[0];
		float y_other = states->data.fl[1];
		float x = blobStates->data.fl[0];
		float y = blobStates->data.fl[1];

		if(i != index && RoboBlob[i]->GetComPort() != -1)
		{
			int x_other_pix = CRoboBlob::WorldToPixX(x_other);
			int y_other_pix = CRoboBlob::WorldToPixY(y_other);
			int xPix = CRoboBlob::WorldToPixX(x);
			int yPix = CRoboBlob::WorldToPixY(y);
			float *localPot = RoboBlob[i]->GetLocalPotential();
			VirtualPotential->RestorePotentialField(x_other_pix,y_other_pix,xPix,yPix,localPot);
		}
	}*/
	
	return vVelocity;
}

#endif // __VIRTUALPOTENTIALWALK_H__