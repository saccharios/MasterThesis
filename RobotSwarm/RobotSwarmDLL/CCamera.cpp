#include "CCamera.h"

CCamera::CCamera()
{


	lastClickedPos[0] = 0;
	lastClickedPos[1] = 0;
	lastClickedPos[2] = 0;
	timestampLastClick = 0;
}

CCamera::CCamera(CvCapture* capt)
{
	//capt = cvCaptureFromCAM(-1);
}

CCamera::~CCamera()
{

}

void CCamera::DrawExtractedBlob(IplImage* input, CvMat* state)
{// Draws circle and line (heading angle)


	// Attempt to adapt to the different resolutions
	int thickness = 1;
	int radius = 10;
	float af = ((float) WIDTH_WORLD * HEIGHT_WORLD  / ( WIDTH_PIX * HEIGHT_PIX));
	thickness = (int) (-15.0f*af + 4.0f);
	radius = (int) (-73.0f*af + 23.0f);

	CvPoint center;
	center.x = CRoboBlob::WorldToPixX(state->data.fl[0]);
	center.y = CRoboBlob::WorldToPixY(state->data.fl[1]);

	CvPoint pt2;
	pt2.x = center.x-(int)(cosf((float)state->data.fl[2])*100);
	pt2.y = center.y-(int)(sinf((float)state->data.fl[2])*100);

	cvLine(input, center,  pt2, cvScalar(0,0,255), thickness);
	cvCircle(input, center , radius, cvScalar(255,255,0),thickness);



}

void CCamera::DeleteDataFile()
{
	ofstream file;
	file.open("statesDataSheet.txt");		//open a file

	file.close();	

}

void CCamera::ReleaseImages()
{
	cvReleaseImage(&rgbOutputImg);
	cvReleaseImage(&rgbOutputImg_resized);
}

void CCamera::ShowImages(string sAlgName)
{

	if (sAlgName == "VirtualPotentialWalk")
	{
		DrawVirtualWall(rgbOutputImg);
	} else if (sAlgName == "SetpointByUser" || sAlgName == "SetpointByUserGlob")
	{
		DrawMouseClick(rgbOutputImg);
	} else if (sAlgName == "StochasticLocalization" || sAlgName == "GridSearch" || sAlgName == "LineSearch" || sAlgName == "MetropolisHastings" || sAlgName == "SimulatedAnnealing")
	{
		DrawTable(rgbOutputImg);
	}


	// Display result in resized window
	//rgbOutputImg_resized = cvCreateImage( cvSize( SHOW_XPIX, SHOW_YPIX ),8,3);     
	cvResize(rgbOutputImg , rgbOutputImg_resized , CV_INTER_LINEAR );
	cvShowImage("Result", rgbOutputImg_resized );



	CaptureClick();

	cvWaitKey(1);
}

void CCamera::DrawVirtualWall(IplImage* img)
{
	cvRectangle(img, cvPoint(WALL_START_X, WALL_START_Y), cvPoint(WALL_END_X,WALL_END_Y), cvScalar(255,0,0) );
	//int wallThickness = (int)(min(WALL_END_Y - WALL_START_Y, WALL_END_X - WALL_START_X)/WALL_THICKNESS_FACTOR);
	//cvRectangle(img, cvPoint(WALL_START_X + wallThickness, WALL_START_Y + wallThickness), cvPoint(WALL_END_X - wallThickness, WALL_END_Y - wallThickness), cvScalar(255,0,0) );

}

void CCamera::DrawMouseClick(IplImage* img)
{
	cvCircle(img, cvPoint(lastClickedPos[0],lastClickedPos[1]), 4, cvScalar(0,180,0));
}

void CCamera::DrawTable(IplImage* img)
{// Author: Stefan Frei, 2013

	int x_start, y_start,x_end,y_end;
	x_start = CRoboBlob::WorldToPixX(OFFSET_X);
	x_end = CRoboBlob::WorldToPixX(OFFSET_X + WIDTH_TABLE);
	y_start = CRoboBlob::WorldToPixY(HEIGHT_WORLD - OFFSET_Y - HEIGHT_TABLE ) ;
	y_end = CRoboBlob::WorldToPixY(HEIGHT_WORLD - OFFSET_Y );


	cvRectangle(img, cvPoint(x_start,y_start), cvPoint(x_end,y_end), cvScalar(0,0,255) );

}

void CCamera::CaptureClick(void)
{
	void* ptr = (void*)&(this->lastClickedPos[0]);
	cvSetMouseCallback("Result", mouseEvent, ptr);

	if (this->lastClickedPos[2] == 1)
	{
		timestampLastClick = GetTickCount();
		this->lastClickedPos[2] = 0;
	}
}

void CCamera::GetLastClick(int* x, int* y, DWORD* timestampLastClick)
{
	*x = this->lastClickedPos[0];
	*y = this->lastClickedPos[1];
	*timestampLastClick = this->timestampLastClick;
}

// This is a callback function, which is only intended as input to cvSetMouseCallback()
void mouseEvent(int evt, int x, int y, int flags, void* param)
{			
    if(evt==CV_EVENT_LBUTTONDOWN){
		//DWORD timestamp = GetTickCount();
        ((int*)param)[0] = x;
		((int*)param)[1] = y;
		((int*)param)[2] = 1; //new click flag. Once this click position is read, the flag is set to 0 (see GetLastClick function above)
    }
}

int CCamera::ExtractBlobsEpuckOriBlack(IplImage* input, vector<CRobot*> vActiveRobots)
{// Author: Stefan Frei, 2013
	
	//rgbOutputImg = cvCreateImage(cvGetSize( input ),8,3);
	cvSet(rgbOutputImg,cvScalar(255,255,255));

	CBlobResult blueblobs; 
	CBlobResult blackblobs;
	CBlobResult doubleBlueBlobs;

	CBlob* currentBlueBlob;
	CvMat* pCoord;

	CvRect BlueBoundingBox, BlackBoundingBox;

	int iNumberOfRobotBlobs;



	IplImage *imageROIHSV, *imgBlack;

	// Create the blue treshholded image
	IplImage* imgHSV = cvCreateImage( cvGetSize( input ),8,3);             
	cvCvtColor( input, imgHSV, CV_BGR2HSV );   

	cvSmooth(imgHSV,imgHSV,CV_BLUR,3);

    cvErode(imgHSV,imgHSV,NULL,1);

	cvDilate(imgHSV,imgHSV,NULL,1);

	IplImage* imgBlue = cvCreateImage( cvGetSize( input ),8,1);  

	cvInRangeS( imgHSV,  cvScalar( 80,100,90 ), cvScalar( 120, 255, 252 ),  imgBlue );  

	// Extract blue blobs
	blueblobs = CBlobResult( imgBlue, NULL, 0 );
	doubleBlueBlobs = CBlobResult( imgBlue, NULL, 0 );
	

	//showBoxes(blueblobs, imgBlue);
	//cvShowImage("blue",imgBlue);	


	// Filter the blobs by size, this depends on resolution
	float area_factor = (float) WIDTH_WORLD * HEIGHT_WORLD / (WIDTH_PIX * HEIGHT_PIX);	



	doubleBlueBlobs.Filter( doubleBlueBlobs, B_EXCLUDE,  CBlobGetArea(), B_LESS, (int) (50.0f/area_factor) );  //the area value may need to be adjusted
	doubleBlueBlobs.Filter( doubleBlueBlobs, B_EXCLUDE,  CBlobGetArea(), B_GREATER, (int) (100.0f/area_factor) ); //the area value may need to be adjusted
	//cout << "doubleBlueBlobs.GetNumBlobs(): " << doubleBlueBlobs.GetNumBlobs() << endl;
	blueblobs.Filter( blueblobs, B_EXCLUDE,  CBlobGetArea(), B_LESS, (int) (10.0f/area_factor) );   
	blueblobs.Filter( blueblobs, B_EXCLUDE,  CBlobGetArea(), B_GREATER, (int) (50.0f/area_factor) );
	//cout << "blueblobs.GetNumBlobs(): " << blueblobs.GetNumBlobs() << endl;
	// Adjust to the case when to TWO blobs are touching each other (one big blob will be detected)

	if(doubleBlueBlobs.GetNumBlobs() > 0)
	{
		DetectDoubleBlobs(&doubleBlueBlobs, imgHSV);
	}




	int num_blueblobs = blueblobs.GetNumBlobs();

	int num_blackblobs;
		
	float distance_x, distance_y, distance_sqr;
	float alpha;


	//showBoxes(blueblobs, rgbOutputImg);

	float bluecenter_x, bluecenter_y, blackcenter_x, blackcenter_y;
	int	counter = 0;

	for(int i=0; i < num_blueblobs; i++)
	{
		currentBlueBlob = blueblobs.GetBlob(i);
		currentBlueBlob->FillBlob(rgbOutputImg, CV_RGB(0,0,255));
		BlueBoundingBox = currentBlueBlob->GetBoundingBox(); 
		
	

		bluecenter_x = CRoboBlob::PixToWorldX(BlueBoundingBox.x + ((float)BlueBoundingBox.width)/2); //convert to real world coordinates, this needs to be a float
		bluecenter_y = CRoboBlob::PixToWorldY(BlueBoundingBox.y + ((float)BlueBoundingBox.height)/2);

		// Enlarge BlueboundingBox for ROI
		BlueBoundingBox.x -= 2;
		BlueBoundingBox.y -= 2;
		BlueBoundingBox.width += 4;
		BlueBoundingBox.height += 4;

		imageROIHSV = cvCloneImage( imgHSV );
		cvSetImageROI(imageROIHSV, BlueBoundingBox);
		imgBlack = cvCreateImage( cvGetSize( imageROIHSV ),8,1);  
		cvInRangeS( imageROIHSV,  cvScalar( 0, 0, 0 ), cvScalar( 180, 255, 50),  imgBlack );

		//cvShowImage("black",imgBlack);
	
		blackblobs = CBlobResult( imgBlack, NULL, 0 );

		blackblobs.Filter( blackblobs, B_EXCLUDE, CBlobGetArea(),B_LESS,(int) 0.5/area_factor);
		blackblobs.Filter( blackblobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, (int) 8.0/area_factor );

		num_blackblobs = blackblobs.GetNumBlobs();


		//cout << "num_blackblobs: " << num_blackblobs << endl;
		for( int k = 0; k < num_blackblobs; k++)
		{

			BlackBoundingBox = (blackblobs.GetBlob(k))->GetBoundingBox();

			blackcenter_x = CRoboBlob::PixToWorldX((float)(BlackBoundingBox.x) + ((float)BlackBoundingBox.width)/2.0f + (float)(BlueBoundingBox.x)); //convert to real world coordinates, this needs to be a float
			blackcenter_y = CRoboBlob::PixToWorldY((float)(BlackBoundingBox.y) + ((float)BlackBoundingBox.height)/2.0f + (float)(BlueBoundingBox.y));

			distance_x = bluecenter_x - blackcenter_x;
			distance_y = bluecenter_y - blackcenter_y;

			distance_sqr = distance_x*distance_x + distance_y*distance_y;
			
			//cout << "distance_sqr: "  << distance_sqr << endl;
			if( 4 <= distance_sqr && distance_sqr <= 20) 
			{
				
				pCoord = cvCreateMat( 3, 1, CV_32FC1 );// always create a new pointer

				alpha = atan2(distance_y, distance_x);

				pCoord->data.fl[0] = bluecenter_x;
				pCoord->data.fl[1] = bluecenter_y;
				pCoord->data.fl[2] = alpha;
				vMeasurementCoord.push_back( pCoord );//save the pointer to the three coordinates for matching
	

				//iNumberOfRobotBlobs++; // Counter of actual valid robot blobs
				//UpdateMeasurement(x_world, y_world, alpha, Epuck,rgbOutputImg, iNumberOfRobotBlobs);	

				 //cout << x_world <<", " << y_world<<", " << alpha << endl;
				 //cout << "inloop " << vMeasurementCoord.back().data.fl[0] << ", "<< vMeasurementCoord.back().data.fl[1] << ", "<< vMeasurementCoord.back().data.fl[2] << ", "  << endl;
			} 


		}
		cvReleaseImage( &imageROIHSV ); 
		cvReleaseImage( &imgBlack ); 
	}
	//cout << endl;

	cvReleaseImage( &imgBlue ); 
	cvReleaseImage( &imgHSV ); 
	


	//time_t T = clock();
	iNumberOfRobotBlobs = MatchBlobs2(); // matches detected blobs to temporary blobs (Note: camera may detect more blobs when epucks are close)
	//T = clock()-T;
	//cout << "Elapsed time [ms] = " << T << endl;


	//cout << "iNumberOfRobotBlobs " << iNumberOfRobotBlobs << endl;


	
	for(size_t i = 0; i < vTempBlobs.size(); i++)
	{
		int tempComPort = -2;

			if (vTempBlobs[i]->GetStep() == MAX_STEPS)
			{
				for(size_t j = 0; j < vComPort.size(); j++)
				{
					if(vComPort[j] == vTempBlobs[i]->GetComPort())
					{
						tempComPort = vComPort[j];
						vComPort.erase(vComPort.begin()+j); 
						break;
					}

				}
				for (size_t j = 0; j<vRoboBlobs.size(); j++)
				{
					if(vRoboBlobs[j]->GetComPort() == tempComPort)
					{
						
						for(size_t k = 0; k < vActiveRobots.size(); k++)
						{

							if(vActiveRobots[k]->GetComPort() == tempComPort)
							{
								vRemovedRobots.push_back(vActiveRobots[k]);
								break;
							}
						}
						
						
						for(int k = vRemovedRobots.size() - 1; k > 0; k--)
						{
							if(vRemovedRobots[k]->GetPriority() < vRemovedRobots[k - 1]->GetPriority())
							{
								CRobot *tempRobot = vRemovedRobots[k - 1];
								vRemovedRobots[k - 1] = vRemovedRobots[k];
								vRemovedRobots[k] = tempRobot;
								continue;
							}
							break;
						}

						vRoboBlobs.erase(vRoboBlobs.begin()+j);
						break;
					}
				}

				vTempBlobs.erase(vTempBlobs.begin()+i); 
				i--;
			}
	}
	//cout << "-----------------------------------------------------------------------------------" << endl;
	return iNumberOfRobotBlobs;
}

int CCamera::MatchBlobs2()
{// Author: Stefan Frei, 2013
	int num_TempBlobs = vTempBlobs.size();
	int num_FoundBlobs = vMeasurementCoord.size();
	//cout << "Nr of Coord-Candidates: " << vMeasurementCoord.size()  <<  endl;

	CvMat* x_k;
	CvMat* x_p;
	CvMat* x_m;// = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* u_k;

	double min_sumTwoNorm = 999999; 
	double sumTwoNorm;
	float distance_sqr, x , y ,alpha;
	float filterradius_sqr = 600; //defines radius where blobs are looked in (in cm^2)

	vector<CvMat*> vPredictionCoord;

	vector<vector<int>> Possibilites_of_TempBlob; // stores information of possibilities of each blob
	vector<vector<int>> PermutedPossibilities; // are all allowed solution, the best one will be chosen
	vector<int> vcurrentRow; // currentRow vector
	vector<int> solution; // solution vector (is one row of PermutedPossibilities);


	float Ts;

	//Predict states ahead and store them
	for (int i = 0; i < num_TempBlobs; i++)
	{
		//SAMPLING_TIME_CAMERA = ((float) clock())/1000.0f - vTempBlobs[i]->oldTime;
		x_p = cvCreateMat( 3, 1, CV_32FC1 );
		x_k = vTempBlobs[i]->GetState();
		Ts = vTempBlobs[i]->GetEKFsamplingTime();
		if( EKFtype == 2 )
		{
			u_k = vTempBlobs[i]->GetInput();
			x_p->data.fl[0]  = x_k->data.fl[0] - u_k->data.fl[0]*Ts*cosf(x_k->data.fl[2]);
			x_p->data.fl[1]  = x_k->data.fl[1] - u_k->data.fl[0]*Ts*sinf(x_k->data.fl[2]); 
			x_p->data.fl[2]  = CRoboBlob::SetAngleInRange(x_k->data.fl[2] + u_k->data.fl[1]*Ts);
		}else
		{	
			x_p->data.fl[0]  = x_k->data.fl[0] - x_k->data.fl[3]*Ts*cosf(x_k->data.fl[2]);
			x_p->data.fl[1]  = x_k->data.fl[1] - x_k->data.fl[3]*Ts*sinf(x_k->data.fl[2]); 
			x_p->data.fl[2]  = CRoboBlob::SetAngleInRange(x_k->data.fl[2] + x_k->data.fl[4]*Ts);
		}





		vPredictionCoord.push_back( x_p ) ;

	}
	//cout << "num_TempBlobs: " << num_TempBlobs << ", num_FoundBlobs: " << num_FoundBlobs << endl;


	for (int i = 0; i < num_TempBlobs; i++) // Delete all the permutations that are not possible (i.e. blobs are too far away)
	{
		x_p = vPredictionCoord[i];
		for (int k = 0; k < num_FoundBlobs; k++)
		{	
			x_m = vMeasurementCoord[k];

			distance_sqr = (x_m->data.fl[0] - x_p->data.fl[0])*(x_m->data.fl[0] - x_p->data.fl[0]) + (x_m->data.fl[1] - x_p->data.fl[1])*(x_m->data.fl[1] - x_p->data.fl[1]);
			//cout << "i = " << i << ", k = " << k << "  distance_sqr = " << distance_sqr << endl;
			if( distance_sqr < filterradius_sqr)
			{
				if(num_TempBlobs == 1 && num_FoundBlobs ==1) // there is only one tempblob, one foundblob, and they are in the range -> take a shortcut
				{
					x = x_m->data.fl[0];
					y = x_m->data.fl[1];
					alpha = x_m->data.fl[2];

					vTempBlobs[0]->SetMeasurement(x,y,alpha);
					vTempBlobs[0]->bLost = false;
					vMeasurementCoord.clear();
					return vTempBlobs.size();
				}else
				{
					vcurrentRow.push_back(k);
				}
			}
		}
		vcurrentRow.push_back(-1); // add -1 at beginning, this represents a lost blob
		Possibilites_of_TempBlob.push_back( vcurrentRow );
		vcurrentRow.clear();
	}
		//cout << "Possibilites_of_TempBlob:" << endl;
		//for( size_t i = 0; i < Possibilites_of_TempBlob.size(); i++)
		//{
		//	for( size_t k = 0; k < Possibilites_of_TempBlob[i].size(); k++)
		//	{
		//		cout << Possibilites_of_TempBlob[i][k] << "  ";
		//	}
		//	cout << endl;
		//}
		//cout << endl << endl;

	bool* FoundBlobIsAssigned = new bool[num_FoundBlobs];

	memset(FoundBlobIsAssigned, false, num_FoundBlobs);
	//for (int i = 0; i < num_FoundBlobs; i++) { FoundBlobIsAssigned[i] = false;}

	if ( num_TempBlobs > 0) // Cant match FoundBlobs to nonexisting  TempBlobs (i.e when num_TempBlobs = 0)
	{
		PermutedPossibilities = CreatePM(num_TempBlobs, Possibilites_of_TempBlob); // Create Possibility matrix, each row represents a configuration

		//cout << "PermutedPossibilities, " << PermutedPossibilities.size() <<":" << endl;
		//for( size_t i = 0; i < PermutedPossibilities.size(); i++)
		//{
		//	for( size_t k = 0; k < PermutedPossibilities[i].size(); k++)
		//	{
		//		cout << PermutedPossibilities[i][k] << "  ";
		//	}
		//	cout << endl;
		//}

		//cout << endl << PermutedPossibilities.size() << endl;

		// Evaluate Possibility matrix

		float add;

		for(size_t cRow = 0; cRow < PermutedPossibilities.size(); cRow++)
		{
			sumTwoNorm = 0;
			
			//cout << "possibility: " ;
			for(int cCol = 0; cCol < num_TempBlobs; cCol++)
			{
				if(PermutedPossibilities[cRow][cCol] == -1)
				{
					add = 9999;
					//sumTwoNorm += add; //Huge penalty when loosing blobs
				} else {
					//cvSub(vPredictionCoord[cRow],vMeasurementCoord[cCol],x_k);
					x = vPredictionCoord[cCol]->data.fl[0] - vMeasurementCoord[PermutedPossibilities[cRow][cCol]]->data.fl[0] ;
					y = vPredictionCoord[cCol]->data.fl[1] - vMeasurementCoord[PermutedPossibilities[cRow][cCol]]->data.fl[1] ;
					alpha = CRoboBlob::SetAngleInRange(vPredictionCoord[cCol]->data.fl[2] -vMeasurementCoord[PermutedPossibilities[cRow][cCol]]->data.fl[2]) ;
					add = x*x  + y*y + alpha*alpha*100; //two norm weighted on angle
					//cout << "x*x = " << x*x << " y*y = " << y*y << " alpha*alpha*400 = " << alpha*alpha*100 ;
					
				}
				sumTwoNorm += add;
				//cout << " adding: " << add << "\t";

				
				//cout << PermutedPossibilities[cRow][cCol] << " " ;

			}
			//cout << endl << "sumTwoNorm: " << sumTwoNorm << endl;
			if( sumTwoNorm < min_sumTwoNorm)
			{
				min_sumTwoNorm = sumTwoNorm;
				solution = PermutedPossibilities[cRow]; 

			}
		}
		//cout << "min Cost: " << min_sumTwoNorm << endl;

		//Allocate solution
		//cout << "num_FoundBlobs " << num_FoundBlobs << endl;
		//cout << "solution: " ;
		//for(size_t i = 0 ; i < solution.size(); i++)
		//{
		//	cout << solution[i] << " " ;
		//}
		//cout << endl ;

		for(size_t i = 0 ; i < solution.size(); i++)
		{
			
			if(solution[i] != -1)
			{
				x = vMeasurementCoord[solution[i]]->data.fl[0];
				y = vMeasurementCoord[solution[i]]->data.fl[1];
				alpha = vMeasurementCoord[solution[i]]->data.fl[2];

				vTempBlobs[i]->SetMeasurement(x,y,alpha);
				vTempBlobs[i]->bLost = false;

				//mark ghost blobs and mark assigned blobs
				for(int k = 0 ; k < num_FoundBlobs; k++)
				{
					//cout << x << "  " << vMeasurementCoord[k]->data.fl[0] << " " << y << " " << vMeasurementCoord[k]->data.fl[1]  << endl;
					if( (x - vMeasurementCoord[k]->data.fl[0])*(x - vMeasurementCoord[k]->data.fl[0]) +  (y - vMeasurementCoord[k]->data.fl[1] )*(y - vMeasurementCoord[k]->data.fl[1] ) < 30.0f)
					{
						FoundBlobIsAssigned[k] = true; //indicate if blobs are assigned (needed to create new ones, but not ghosts)
					}
				}
			} 
		}
	}


	//cout << "FoundBlobIsAssigned: ";
	for(int i = 0; i < num_FoundBlobs; i++)
	{
		//cout << FoundBlobIsAssigned[i] << " ";
		if( FoundBlobIsAssigned[i] == false )
		{
			//cout << "create newblob" << endl;
			CRoboBlob *newBlob = new CRoboBlob(Epuck);

			x = vMeasurementCoord[i]->data.fl[0];
			y = vMeasurementCoord[i]->data.fl[1];
			alpha = vMeasurementCoord[i]->data.fl[2];


			newBlob->InitState(x,y,alpha); // x_world, y_world, alpha
			newBlob->SetMeasurement(x,y,alpha);

			vTempBlobs.push_back(newBlob);
			FoundBlobIsAssigned[i] = true;
		}
		//mark ghost blobs
		for(int k = i ; k < num_FoundBlobs; k++)
		{
			//cout << "Blob: " << i << " Candidate: " << k << endl;
			//cout << x << "  " << vMeasurementCoord[k]->data.fl[0] << " " << y << " " << vMeasurementCoord[k]->data.fl[1]  << endl;
			//cout << "Distance: " << (x - vMeasurementCoord[k]->data.fl[0])*(x - vMeasurementCoord[k]->data.fl[0]) +  (y - vMeasurementCoord[k]->data.fl[1] )*(y - vMeasurementCoord[k]->data.fl[1]) << endl;

			if((x - vMeasurementCoord[k]->data.fl[0])*(x - vMeasurementCoord[k]->data.fl[0]) +  (y - vMeasurementCoord[k]->data.fl[1] )*(y - vMeasurementCoord[k]->data.fl[1] ) < 30.0f)
			{
				FoundBlobIsAssigned[k] = true; //indicate if blobs are assigned (needed to create new ones, but not ghosts)
			}
		}


	}
	//cout << endl;

	vMeasurementCoord.clear();
	delete[] FoundBlobIsAssigned;

	//cout << "vTempBlobs.size() " << vTempBlobs.size() << endl;
	return vTempBlobs.size();

}

void CCamera::showBoxes( CBlobResult blobs, IplImage* dst)
{// Author: Stefan Frei, 2013
	CBlob *currentBlob;    
	CvPoint pt1, pt2;   
	CvRect cvRect;  
	int num_blobs = blobs.GetNumBlobs(); 
	//cout << num_blobs << ": ";
    for ( int i = 0; i < num_blobs; i++ )   
	{
		currentBlob = blobs.GetBlob( i ); 
		cvRect = currentBlob->GetBoundingBox();   
		pt1.x = cvRect.x-3;   
		pt1.y = cvRect.y-3;   
		pt2.x = cvRect.x + cvRect.width+3;   
		pt2.y = cvRect.y + cvRect.height+3;   
		// Attach bounding rect to blob in orginal video input   
		cvRectangle( dst,  pt1, pt2,  cvScalar(255, 0, 255, 0),  1,  8,  0 );  
		//cout << cvRect.width * cvRect.height << ", ";

    }  
	//cout << endl;
 }

 vector<vector<int>> CCamera::CreatePM(int num_TempBlobs, vector<vector<int>> Possibilites_of_TempBlob)
{// Author: Stefan Frei, 2013
	// Creates PossibilityMatrix 

	vector<vector<int>> Permutations;
	vector<int> vcurrentRow;

	if( num_TempBlobs == 1)
	{
		for(size_t k = 0; k < Possibilites_of_TempBlob[0].size(); k++)
		{
			vcurrentRow.push_back(Possibilites_of_TempBlob[0][k]);
			Permutations.push_back(vcurrentRow);
			vcurrentRow.clear();
		}

		return Permutations;
	}
	
	vector<vector<int>> LowerPermutations;
	vector<vector<int>> LowerPossibilites_of_TempBlob(Possibilites_of_TempBlob.begin()+1,Possibilites_of_TempBlob.end());


	LowerPermutations = CreatePM(num_TempBlobs-1, LowerPossibilites_of_TempBlob);

	for( size_t i = 0; i < Possibilites_of_TempBlob[0].size(); i++)
	{
		for(size_t k = 0; k < LowerPermutations.size(); k++)
		{
			vcurrentRow = LowerPermutations[k];
			if (find(vcurrentRow.begin(), vcurrentRow.end(), Possibilites_of_TempBlob[0][i]) == vcurrentRow.end() || Possibilites_of_TempBlob[0][i] == -1) //element does not exist yet or is equal -1, add it
			{
				vcurrentRow.insert(vcurrentRow.begin(),Possibilites_of_TempBlob[0][i]);
				Permutations.push_back( vcurrentRow );
				vcurrentRow.clear();
			}

		}

	}
	return Permutations;
}

void CCamera::DetectDoubleBlobs(CBlobResult* doubleBlobs, IplImage* imgHSV)
{//Splits up robots that are close together. Adds valid possibilites to vMeasurementCoord 

	CBlob *currentDoubleBlob, *currentBlackBlob;
	CvRect DoubleBoundingBox, ROIBox, BlackBoundingBox;
	IplImage *imgHSVblack, *imgBlack ;

	CBlobResult blackblobs;

	
	float area_factor = (float) WIDTH_WORLD * HEIGHT_WORLD / (WIDTH_PIX * HEIGHT_PIX);
	float botRadius = 4.0f;
	float x_center[4], y_center[4];
	float distance_sqr, blackcenter_x, blackcenter_y, alpha, distance_x, distance_y;
	CvMat* pCoord;

	size_t num_doubleBlobs = doubleBlobs->GetNumBlobs() ;

	for(size_t i = 0; i < num_doubleBlobs; i++)
	{// the implementation is a bit cumbersome, but one cannot add manually blobs to CBlobResult

		currentDoubleBlob = doubleBlobs->GetBlob(i);
		DoubleBoundingBox = currentDoubleBlob->GetBoundingBox();

		ROIBox.x = DoubleBoundingBox.x - 2;
		ROIBox.y = DoubleBoundingBox.y - 2;
		ROIBox.width = DoubleBoundingBox.width  + 4; 
		ROIBox.height = DoubleBoundingBox.height + 4;
	
		//cout<< "ROIBox.x " << ROIBox.x << " ROIBox.y " << ROIBox.y << " BROIBox.width  " << ROIBox.width << " ROIBox.height  " << ROIBox.height << endl;
		imgHSVblack = cvCloneImage( imgHSV); 
		cvSetImageROI(imgHSVblack, ROIBox);
		imgBlack = cvCreateImage( cvGetSize( imgHSVblack ),8,1);
		cvInRangeS( imgHSVblack, cvScalar( 0, 0, 0 ), cvScalar( 180, 255, 50),    imgBlack );


		blackblobs = CBlobResult( imgBlack , NULL, 0 ); 
		blackblobs.Filter( blackblobs, B_EXCLUDE, CBlobGetArea(),B_LESS,(int) 0.5/area_factor);
		blackblobs.Filter( blackblobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, (int) 5.0/area_factor );

		cvReleaseImage(&imgHSVblack);
		cvReleaseImage(&imgBlack);


		//cout << "blackblob in ROI: " << blackblobs.GetNumBlobs() << endl;
		for( int i = 0 ;i < blackblobs.GetNumBlobs(); i++)
		{
			currentBlackBlob = blackblobs.GetBlob(i);
			BlackBoundingBox = currentBlackBlob->GetBoundingBox();
			//cout<< "BlackBoundingBox.x " << BlackBoundingBox.x << " BlackBoundingBox.y " << BlackBoundingBox.y << " BlackBoundingBox.width  " << BlackBoundingBox.width << " BlackBoundingBox.height  " << BlackBoundingBox.height << endl;
			blackcenter_x = CRoboBlob::PixToWorldX((float)(BlackBoundingBox.x) + ((float)BlackBoundingBox.width)/2.0f + (float)(ROIBox.x));
			blackcenter_y = CRoboBlob::PixToWorldY((float)(BlackBoundingBox.y) + ((float)BlackBoundingBox.height)/2.0f + (float)(ROIBox.y));

			//first quadrant
			x_center[0] = CRoboBlob::PixToWorldX((float) DoubleBoundingBox.x) + botRadius;
			y_center[0] = CRoboBlob::PixToWorldY((float) DoubleBoundingBox.y) + botRadius;
			//fsecond quadrant
			x_center[1] = CRoboBlob::PixToWorldX((float) DoubleBoundingBox.x + (float) DoubleBoundingBox.width) - botRadius;
			y_center[1] = CRoboBlob::PixToWorldY((float) DoubleBoundingBox.y) + botRadius;
			//third quadrant
			x_center[2] = CRoboBlob::PixToWorldX((float) DoubleBoundingBox.x + (float) DoubleBoundingBox.width) - botRadius;
			y_center[2] = CRoboBlob::PixToWorldY((float) DoubleBoundingBox.y + (float) DoubleBoundingBox.height) - botRadius;
			//fourth quadrant
			x_center[3] = CRoboBlob::PixToWorldX((float) DoubleBoundingBox.x) + botRadius;
			y_center[3] = CRoboBlob::PixToWorldY((float) DoubleBoundingBox.y + (float) DoubleBoundingBox.height) - botRadius;

			for(int k = 0; k< 4; k++)
			{
				distance_x = x_center[k] - blackcenter_x;
				distance_y = y_center[k] - blackcenter_y;
				
				distance_sqr = distance_x*distance_x + distance_y*distance_y;
				//cout << "distance_sqr: " << distance_sqr << endl;
				if( 4 <= distance_sqr && distance_sqr <= 20)
				{
					pCoord = cvCreateMat( 3, 1, CV_32FC1 );// always create a new pointer

					alpha = atan2(distance_y, distance_x);

					pCoord->data.fl[0] = x_center[k];
					pCoord->data.fl[1] = y_center[k];
					pCoord->data.fl[2] = alpha;
					vMeasurementCoord.push_back( pCoord );//save the pointer to the three coordinates for matching

				} 

			}
		}
	}
}

