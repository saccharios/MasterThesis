#include "CVirtualPotential.h"
#include <iostream>
using namespace std;

CVirtualPotential::CVirtualPotential()
{
	//wallThickness = min(WALL_END_Y - WALL_START_Y, WALL_END_X - WALL_START_X)/WALL_THICKNESS_FACTOR;
	wallThickness = 30;

	for (int i=0; i<WIDTH_PIX; i++)
	{
		for (int j=0; j<HEIGHT_PIX; j++)
		{
			 potentialMatrix[i][j] = 0;
		}
	}

	CreatePotentialWall();

	varianceX = CRoboBlob::WorldToPixX(VARIANCE_X); 
	varianceY = CRoboBlob::WorldToPixY(VARIANCE_Y); 
	
}

CVirtualPotential::~CVirtualPotential()
{

}


float *CVirtualPotential::ExtractLocalPotentialField(float *localpotential, int x_pix, int y_pix)
{
	
	localpotential[0] = potentialMatrix[x_pix - 1][y_pix - 1]; //nw
	localpotential[1] = potentialMatrix[x_pix][y_pix - 1]; //n
	localpotential[2] = potentialMatrix[x_pix + 1][y_pix - 1]; //ne
	localpotential[3] = potentialMatrix[x_pix - 1][y_pix]; //w
	localpotential[4] = potentialMatrix[x_pix][y_pix]; //c
	localpotential[5] = potentialMatrix[x_pix + 1][y_pix]; //e
	localpotential[6] = potentialMatrix[x_pix - 1][y_pix + 1]; //sw
	localpotential[7] = potentialMatrix[x_pix][y_pix + 1]; //s
	localpotential[8] = potentialMatrix[x_pix + 1][y_pix + 1]; //se

	return localpotential;
	

}

void CVirtualPotential::ComputeGradients(float *localpotential)
{
	/*float x_world = states->data.fl[0];
	float y_world = states->data.fl[1];

	int x_pix = CRoboBlob::WorldToPixX(x_world);
	int y_pix = CRoboBlob::WorldToPixY(y_world);

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

	cout << "ComputeGradients, x: " << x_world << ", y: " << y_world << ", xpix: " << x_pix << ", yPix: " << y_pix << endl;
	*/

	float nw = localpotential[0];
	float n = localpotential[1];
	float ne = localpotential[2];
	float w = localpotential[3];
	float c = localpotential[4];
	float e = localpotential[5];
	float sw = localpotential[6];
	float s = localpotential[7];
	float se = localpotential[8];

	//cout << "potential: " << nw << ", " << n << ", " << ne << ", " << w << ", " << c << ", " << e << ", " << sw << ", " << s << ", " << se << endl;

	gradients[0] = w - c;
	gradients[1] = nw - c;
	gradients[2] = n - c;
	gradients[3] = ne - c;
	gradients[4] = e - c;
	gradients[5] = se - c;
	gradients[6] = s - c;
	gradients[7] = sw - c;

	/*for (int i=0; i<8; i++)
	{
		cout << "Gradient " << i << ": " << gradients[i] << endl;
	}*/
}

void CVirtualPotential::CreatePotentialWall()
{
	//int m = 2;
	float m = MAX_POTENTIAL_WALL/(float)wallThickness;

	for (int i=0; i<WIDTH_PIX; i++)
	{
		for (int j=0; j<HEIGHT_PIX; j++)
		{
			

				if(i < WALL_START_X || i > WALL_END_X)  
				{
					potentialMatrix[i][j] = MAX_POTENTIAL_WALL;
				}
				else if(i < WALL_START_X + wallThickness)
				{
					if (i - WALL_START_X <= j - WALL_START_Y && i - WALL_START_X <= WALL_END_Y - j)
					{
						 potentialMatrix[i][j] = -m*i + MAX_POTENTIAL_WALL + m * WALL_START_X;
					}
				}
				else if(i > WALL_END_X - wallThickness)
				{
					if(WALL_END_X - i <= j - WALL_START_Y && WALL_END_X - i <= WALL_END_Y - j)
					{
						potentialMatrix[i][j] =  m*i + MAX_POTENTIAL_WALL - m * WALL_END_X;
					}
				}


			if(!(i < WALL_START_X || i > WALL_END_X))
			{
				if(j < WALL_START_Y || j > WALL_END_Y)
				{
					potentialMatrix[i][j] = MAX_POTENTIAL_WALL;
				}
				else if(j < WALL_START_Y + wallThickness)
				{
					if(i - WALL_START_X > j - WALL_START_Y && WALL_END_X - i > j - WALL_START_Y)
					{
						potentialMatrix[i][j] = -m*j + MAX_POTENTIAL_WALL + m * WALL_START_Y;
					}
				}
				else if(j > WALL_END_Y - wallThickness)
				{
					if(i - WALL_START_X > WALL_END_Y - j && WALL_END_X - i > WALL_END_Y - j)
					{
						potentialMatrix[i][j] = m*j + MAX_POTENTIAL_WALL - m * WALL_END_Y;
					}
				}
			}
			/*if (potentialMatrix[i][j] > MAX_POTENTIAL_WALL)
			{
				potentialMatrix[i][j] = MAX_POTENTIAL_WALL;
			}*/

			//CRoboBlob::WriteDataToFile(i,j,potentialMatrix[i][j]);

		}
	}
}

float CVirtualPotential::GaussianDistribution(int iMeanX, int iMeanY, int x, int y) //all parameters are in world coordinates
{
	float potential;

	meanX = iMeanX; // CRoboBlob::WorldToPixX(iMeanX);
	meanY = iMeanY; // CRoboBlob::WorldToPixY(iMeanY);
	int xPix = x; //= CRoboBlob::WorldToPixX(x);
	int yPix = y; //CRoboBlob::WorldToPixY(y);

	float tempX = (float) (xPix - meanX)*(xPix - meanX);
	float tempY = (float) (yPix - meanY)*(yPix - meanY);

	potential = MAX_POTENTIAL_ROBOT*exp(-(tempX/(2*varianceX) + tempY/(2*varianceY)));

	//cout << "Pot: " << potential << endl;

	return potential;

}

void CVirtualPotential::CreateVirtualForceField(int x_pix, int y_pix, vector<CRoboBlob*> &vObstacle)
{
	
	float potential;
	int iStart = max(0,x_pix - ACTIVE_WINDOW_SIZE/2);
	int iEnd = min(WIDTH_PIX, x_pix + ACTIVE_WINDOW_SIZE/2);
	int jStart = max(0,y_pix - ACTIVE_WINDOW_SIZE/2);
	int jEnd = min(HEIGHT_PIX,  y_pix + ACTIVE_WINDOW_SIZE/2);

	vVirtualForce.clear();
	vVirtualForce.push_back(0);
	vVirtualForce.push_back(0);
	/*cout << "x_pix: " <<  x_pix << ", y_pix: " << y_pix << endl;
	cout << "iStart: " <<  iStart << ", iEnd: " << iEnd << endl;
	cout << "jStart: " <<  jStart << ", jEnd: " << jEnd << endl;*/
	//cout << "x_pix_obstacle.size(): " <<  x_pix_obstacle.size() << endl;

	
			//vObstacle.push_back(RoboBlob[i]);
			//vObstacleX.push_back(x_obstacle_pix);
			//vObstacleY.push_back(y_obstacle_pix);
	

	for(int i = iStart; i < iEnd ; i++)
	{
		for(int j = jStart; j < jEnd; j++)
		{
		
			//cout << "Obstacle.size: " << vObstacle.size() << endl;

			/*if(vObstacle.size() > 0)
			{
				activeWindow[i - iStart][j - jStart] = vObstacle.size()*potentialMatrix[i][j];
			}
			else*/
			{
				activeWindow[i - iStart][j - jStart] = potentialMatrix[i][j];
			}

			//cout << "activeWindow[i - iStart][j - jStart]: " << activeWindow[i - iStart][j - jStart] << endl;

			for(size_t k = 0; k < vObstacle.size(); k++)
			{
				float x_obstacle = vObstacle[k]->GetState()->data.fl[0];
				float y_obstacle = vObstacle[k]->GetState()->data.fl[1];
				int x_obstacle_pix = CRoboBlob::WorldToPixX(x_obstacle);
				int y_obstacle_pix = CRoboBlob::WorldToPixY(y_obstacle);

				if(vObstacle[k]->GetType() == Khepera)
				{
					varianceX = CRoboBlob::WorldToPixX(2*VARIANCE_X);
					varianceY = CRoboBlob::WorldToPixX(2*VARIANCE_Y);
				}
				else
				{
					varianceX = CRoboBlob::WorldToPixX(VARIANCE_X);
					varianceY = CRoboBlob::WorldToPixX(VARIANCE_Y);
				}

				activeWindow[i - iStart][j - jStart] += GaussianDistribution(x_obstacle_pix, y_obstacle_pix, i,j);

			}

			float distanceSquared = (float) (i - x_pix)*(i - x_pix) + (j - y_pix)*(j - y_pix);
			float distance = sqrt(distanceSquared);

				//cout << "x_pix_obstacle[k]: " <<  x_pix_obstacle[k] << ", y_pix_obstacle[k]: " << y_pix_obstacle[k] << endl;

			if(distance != 0)
			{
				potential = activeWindow[i - iStart][j - jStart];
				vVirtualForce[0] = vVirtualForce[0] + potential/distanceSquared*(i - x_pix)/distance;
				vVirtualForce[1] = vVirtualForce[1] + potential/distanceSquared*(j - y_pix)/distance; 
			}

			/*if(j == WALL_START_Y)
			{
				cout << "WALL_START_Y Potential: " << activeWindow[i - iStart][j - jStart] << endl;
			}
			else if(j == WALL_END_Y)
			{
				cout << "WALL_END_Y Potential: " << activeWindow[i - iStart][j - jStart] << endl;
			}
			else if(i == WALL_START_X)
			{
				cout << "WALL_START_X Potential: " << activeWindow[i - iStart][j - jStart] << endl;
			}
			else if(i == WALL_END_X)
			{
				cout << "WALL_END_X Potential: " << activeWindow[i - iStart][j - jStart] << endl;
			}*/

			/*if(i != iStart && i != iEnd - 1)
			{
				j = j + ACTIVE_WINDOW_SIZE - 3; 
			}*/
		}
	}

	vVirtualForce[0] = -vVirtualForce[0];
	vVirtualForce[1] = -vVirtualForce[1];

	//cout << "vVirtualForce[0]: " << vVirtualForce[0] << ", vVirtualForce[1]: " << vVirtualForce[1] << endl;

	if(abs(vVirtualForce[0]) < 0.00001)
	{
		vVirtualForce[0] = 0;
		//cout << "vVirtualForce[0] changed" << endl;
	}

	if(abs(vVirtualForce[1]) < 0.00001)
	{
		vVirtualForce[1] = 0;
		//cout << "vVirtualForce[1] changed" << endl;
	}

	/*potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix - 1, y_pix - 1);
	//localPotential[0] = potential;
	localpotential[0] += potential;
	//cout << "Pot: " << localpotential[0] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix, y_pix - 1);
	//localPotential[1] = potential;
	localpotential[1] += potential;
	//cout << "Pot: " <<localpotential[1] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix + 1, y_pix - 1);
	//localPotential[2] = potential;
	localpotential[2] += potential;
	//cout << "Pot: " << localpotential[2] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix - 1, y_pix);
	//localPotential[3] = potential;
	localpotential[3] += potential;
	//cout << "Pot: " << localpotential[3] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix, y_pix);
	//localPotential[4] = potential;
	localpotential[4] += potential;
	//cout << "Pot: " << localpotential[4] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix + 1, y_pix);
	//localPotential[5] = potential;
	localpotential[5] += potential;
	//cout << "Pot: " << localpotential[5] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix - 1, y_pix + 1);
	//localPotential[6] = potential;
	localpotential[6] += potential;
	//cout << "Pot: " << localpotential[6] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix, y_pix + 1);
	//localPotential[7] = potential;
	localpotential[7] += potential;
	//cout << "Pot: " << localpotential[7] << endl;

	potential = GaussianDistribution(x_other_pix, y_other_pix, x_pix + 1, y_pix + 1);
	//localPotential[8] = potential;
	localpotential[8] += potential;
	//cout << "Pot: " << localpotential[8] << endl;

	cout << endl;

	return localpotential;*/

}

/*double LinearFunction(int x, double m, int q)
{
	double y = m*x + q;
	return y;
}*/

/*void CVirtualPotential::PlotMoveDecision()
{
	Engine *ep;
	mxArray *T = NULL, *result = NULL;
	mxArray *Zmatrix = NULL;
	char buffer[BUFSIZE+1];
	double time[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
	const int data = 4;
	const int data2 = 2;
	double xData[data];
	double yData[data2][data];

	mxArray *Z_Matrix = NULL;
	const int iDataSize = 33;
	double dActiveWindow[iDataSize][iDataSize];
	double m = 1;
	int q = 0;
	char ch;
	


	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		//return EXIT_FAILURE;
	}
	
	while((ch = getch()) != 27) 
	{
		for(int i = 0; i<iDataSize; i++)
		{
			for(int j = 0; j<iDataSize; j++)
			{
				//dActiveWindow[i][j] = LinearFunction(j,m,q);
			}
		}
		m = m*1.5;
												
		engPutVariable(ep, "Z_Matrix", Z_Matrix);
		engEvalString(ep,"X = linspace(1,33,33);");
		engEvalString(ep,"X = X';");
		engEvalString(ep,"Y = linspace(1,33,33);");
		engEvalString(ep,"Y = Y';");
		engEvalString(ep,"Z=zeros(size(Y,1),size(X,1));");
		 engEvalString(ep,"for i=1:33; \
							for j =1:33;\
								Z(j,i)=Z_Matrix(j,i); \
							end; \
						   end;");
	
		engEvalString(ep,"surf(X,Y,Z);");
		engEvalString(ep,"arrow([X(1,1) Y(1,1) Z(1,1)],[X(33,33) Y(33,33) Z(33,33)]);");
	
	}

	
	printf("Hit return to continue\n\n");
	fgetc(stdin);
	/*
	 * We're done for Part I! Free memory, close MATLAB figure.
	 */
/*	printf("Done for Part I.\n");
	mxDestroyArray(T);
	engEvalString(ep, "close;");

}
*/


vector<float> CVirtualPotential::MoveDecision(float stateAngle, float deltaTime, float wheelDistance)
{
	//int decision;

	// 1: turn right
	// 0: don't turn
	// -1 turn left

	//float minGradient = MAX_POTENTIAL;
	//float minDirection = 0;
	//float minAngleDiff = 7; // 7 > 2*PI

	float x = vVirtualForce[0];
	float y = vVirtualForce[1];

	float absForce = sqrt(x*x + y*y);

	float forceAngle; 

	if(x == 0 && y == 0)
	{

		forceAngle = stateAngle;
	}
	else
	{
		forceAngle = atan2(y,x) + PI;
	}
	

	float angleDiff = stateAngle - forceAngle;

	//cout << "forceAngle: " << forceAngle*180/PI << endl;

	//cout << "stateAngle: " << stateAngle*180/PI << endl;

	//for(int i = 0; i < 8; i++)
	//{
		//float angleOfGradient = i*PI/4;
		//float tempAngleDiff = angle - angleOfGradient;

		if (abs(angleDiff) > PI)
		{
			if (angleDiff >= 0)
			{
				angleDiff = -(2*PI - angleDiff);
			}
			else
			{
				angleDiff = 2*PI - abs(angleDiff);
			}
		}

		/*if(gradients[i] < minGradient || (gradients[i] == minGradient && abs(tempAngleDiff) < abs(minAngleDiff)))
		{
			minGradient = gradients[i];
			minAngleDiff = tempAngleDiff;
			minDirection = i;
		}
	}*/

	//cout << "minAngle: " << minDirection*45 << endl; 
	//cout << "minAngleDiff: " << minAngleDiff*180/PI << endl; 

	float vl;
	float vr;

	//cout << "angleDiff: " << angleDiff << endl;

	if (abs(angleDiff) <= PI/4)
	{
		vl =  MAX_SPEED;
		vr =  MAX_SPEED;
		//decision = 0;
	}
	else
	{
	
		//vl = (MAX_SPEED / 2 - STEERING_FACTOR*angleDiff*absForce);
		//vr = (MAX_SPEED / 2 + STEERING_FACTOR*angleDiff*absForce);

		float omega = angleDiff/deltaTime;
		float omegaMax = 2.5;

		cout << "delta time: " << deltaTime << endl;
		cout << "omega: " << omega << endl;
		cout << "absForce: " << absForce << endl;
		/*cout << "1/2*omega*wheelDistance: " << 0.5*omega*wheelDistance << endl;*/

		if (absForce > 10)
		{
			absForce = 1;
		}
		else
		{
			absForce /= 10; 
		}

		if (omega > omegaMax)
		{
			omega = omegaMax;
		}
		else if (omega < -omegaMax)
		{
			omega = -omegaMax;
		}

		//vl = (/*STEERING_FACTOR**/0.5*absForce*MAX_SPEED - 0.5*omega*wheelDistance);
		//vr = (/*STEERING_FACTOR**/0.5*absForce*MAX_SPEED + 0.5*omega*wheelDistance);

		vl =  (float) (0.5*MAX_SPEED  - 0.25*omega*wheelDistance); //0.5*0.5*d*omega
		vr =  (float) (0.5*MAX_SPEED  + 0.25*omega*wheelDistance); //6*absForce + 3.5

		cout << "omega: " << omega << endl;
		cout << "absForce: " << absForce << endl;
		cout << "vl: " << vl << endl;
	    cout << "vr: " << vr << endl << endl;

		if(vl > MAX_SPEED)
		{
			vl =  MAX_SPEED;
		}
		else if(vl < 0)
		{
			vl = 0;
		}

		if(vr > MAX_SPEED)
		{
			vr =  MAX_SPEED;
		}
		else if(vr < 0)
		{
			vr = 0;
		}
		
	}

	//cout << "STEERING_FACTOR*absForce*MAX_SPEED/2: " << STEERING_FACTOR*absForce*MAX_SPEED/2 << endl;
	//cout << "absForce*MAX_SPEED: " << absForce*MAX_SPEED << endl;

	

	/*else if (angleDiff > PI/8)
	{
		decision = -1;
	}
	else
	{
		decision =  1;
	}*/
	
	vector<float> vVelocity;
	vVelocity.push_back(vl);
	vVelocity.push_back(vr);

	return vVelocity;
}

void CVirtualPotential::ComputeTotalField(int x_pix, int y_pix)
{
	float potential;
	float tempX;
	float tempY;


	for(int i = 0; i < WIDTH_PIX; i++)
	{
		for(int j = 0; j < HEIGHT_PIX; j++)
		{
			tempX = (float) ((i - x_pix)*(i - x_pix));
			tempY = (float) ((j - y_pix)*(j - y_pix));

			potential = MAX_POTENTIAL_ROBOT*exp(-(tempX/(2*varianceX) + tempY/(2*varianceY)));
			potentialMatrix[i][j] += potential;
			//CRoboBlob::WriteDataToFile(i,j,potentialMatrix[i][j]);
	
		}

	}
	
	
}

