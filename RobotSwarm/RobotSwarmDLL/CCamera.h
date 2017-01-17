#ifndef __CCAMERA_H__
#define __CCAMERA_H__

#include "CRoboBlob.h"
#include "CRobot.h"
#include "CVirtualPotential.h"
#include "opencv/cv.h"   
#include "opencv/highgui.h"   
#include "BlobResult.h"   
#include <algorithm> 
#include <CameraConfig.h>
#include <boost/lexical_cast.hpp>
#include <boost/math/special_functions/factorials.hpp>
using namespace std;
using namespace boost::numeric;



class CCamera
{
private:
	vector<CvMat*> vMeasurementCoord; //stores temporary the three coordinates of possible RoboBlobs
	//IplImage* rgbOutputImg_resized;

public:
	
	IplImage *frm,  *rgbOutputImg, *rgbOutputImg_resized;
	vector<CRoboBlob*> vTempBlobs;
	
	vector<int> vComPort; //com ports assigned to a RoboBlob-Filter
	vector<CRoboBlob*> vRoboBlobs;
	vector<CRobot*> vRemovedRobots;
	vector<int> vComPortPriority;
	int lastClickedPos[3];
	DWORD timestampLastClick;

	CCamera();
	CCamera(CvCapture* capt);
	~CCamera();

	int ExtractBlobsEpuckOriBlack(IplImage* input, vector<CRobot*> vActiveRobots);

	void DrawExtractedBlob(IplImage* input, CvMat* states);

	void ReleaseImages();
	void ShowImages(string sAlgName = "");

	int MatchBlobs2();
	void DeleteDataFile();
	void DrawVirtualWall(IplImage* img);
	void DrawMouseClick(IplImage* img);
	void DrawTable(IplImage* img);
	void GetLastClick(int* x, int* y,  DWORD* timestampLastClick);
	void CaptureClick(void);

	void showBoxes( CBlobResult blobs, IplImage* dst);
	 vector<vector<int>> CreatePM(int num_TempBlobs, vector<vector<int>> Possibilites_of_TempBlob);
	void DetectDoubleBlobs(CBlobResult* doubleBlobs, IplImage* imgHSV);
	

};

void mouseEvent(int evt, int x, int y, int flags, void* param);

#endif //__CCAMERA_H__
