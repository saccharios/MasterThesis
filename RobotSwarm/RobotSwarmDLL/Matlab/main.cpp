/*#include <iostream>
#include <engine.h>


http://www.umiacs.umd.edu/~jsp/Downloads/MatlabEngine/MatlabEngine.pdf

using namespace std;


void main()
{
	Engine *en = engOpen(NULL);
	mxArray *z_array = mxCreateDoubleMatrix(1,10,mxREAL);
	//double *pz = mxGetPr(z_array);
	double arr[10];
	int k = 0;

	//for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			arr[j] = j;
			//cout << pz[k] << endl;
			k++;
		}
	}
	k = 0;
	memcpy((void *)mxGetPr(z_array), (void *)arr, sizeof(arr));
	engPutVariable(en,"z",z_array);

	engEvalString(en,"gaussian");
	//engClose(en);

}
*/
/* $Revision: 1.1.6.4 $ */
/*
 *	engdemo.cpp
 *
 *	A simple program to illustrate how to call MATLAB
 *	Engine functions from a C++ program.
 *
 * Copyright 1984-2011 The MathWorks, Inc.
 * All rights reserved
 */
#include <stdlib.h>
#include <cmath>
#include <conio.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"
#include <iostream>
using namespace std;
#define  BUFSIZE 256

double GaussianPotential(int xPix, int yPix, int xObstaclePix, int yObstaclePix)
{
	int variance = 100;

	int tempX = (xObstaclePix - xPix)*(xObstaclePix - xPix);
	int	tempY = (yObstaclePix - yPix)*(yObstaclePix - yPix);

	double potential = exp((double)(-(tempX/(2*variance) + tempY/(2*variance))));
	return potential;
}

double LinearFunction(int x, double m, int q)
{
	double y = m*x + q;
	return y;
}

int main()

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
	

	//int *iData = 0;
	//iData = new int[data];
	//xData = new double[data];
	





	//printf("iData[2] %d",iData[2]);

	

	/*for(double i = 0; i<data; i++)
	{
		xData[(int)i] = i;
	}

	for(double i = 0; i<data2; i++)
	{
		for(double j = 0; j<data; j++)
		{
			yData[(int)i][(int)j] = i*j;
		}
		
	}
	*/
	/*for(int i = 0; i<480; i++)
	{
		yData[i] = i;
	}
	*/
	/*
	 * Call engOpen with a NULL string. This starts a MATLAB process 
     * on the current host using the command "matlab".
	 */
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}
	
	while((ch = getch()) != 27) 
	{
		for(int i = 0; i<iDataSize; i++)
		{
			for(int j = 0; j<iDataSize; j++)
			{
				dActiveWindow[i][j] = LinearFunction(j,m,q);
			}
		}
		m = m*1.5;
														/*
	 * PART I
	 *
	 * For the first half of this demonstration, we will send data
	 * to MATLAB, analyze the data, and plot the result.
	 */

	/* 
	 * Create a variable for our data
	 */
	//T = mxCreateDoubleMatrix(1, 10, mxREAL);
	//memcpy((void *)mxGetPr(T), (void *)time, sizeof(time));

	/*Zmatrix = mxCreateDoubleMatrix(data2, data, mxREAL);
	memcpy((void *)mxGetPr(Zmatrix), (void *)yData, sizeof(yData));*/

		Z_Matrix = mxCreateDoubleMatrix(iDataSize, iDataSize, mxREAL);
		memcpy((void *)mxGetPr(Z_Matrix), (void *)dActiveWindow, sizeof(dActiveWindow));
						/*
	 * Place the variable T into the MATLAB workspace
	 */
	//engPutVariable(ep, "T", T);
	//engPutVariable(ep, "Zmatrix", Zmatrix);
		engPutVariable(ep, "Z_Matrix", Z_Matrix);
	
							/*
	 * Evaluate a function of time, distance = (1/2)g.*t.^2
	 * (g is the acceleration due to gravity)
	 */
	//engEvalString(ep, "D = .5.*(-9.8).*T.^2;");
	//engEvalString(ep, "D = T;");
		engEvalString(ep,"X = linspace(1,33,33);");
		engEvalString(ep,"X = X';");
		engEvalString(ep,"Y = linspace(1,33,33);");
		engEvalString(ep,"Y = Y';");
		engEvalString(ep,"Z=zeros(size(Y,1),size(X,1));");
		//engEvalString(ep,"Z(1,1) = TEST(1,2);");
		 engEvalString(ep,"for i=1:33; \
							for j =1:33;\
								Z(j,i)=Z_Matrix(j,i); \
							end; \
						   end;");
	
		engEvalString(ep,"surf(X,Y,Z);");
		engEvalString(ep,"arrow([X(1,1) Y(1,1) Z(1,1)],[X(33,33) Y(33,33) Z(33,33)]);");
	
	}

	/*
	 * Plot the result
	 */
	/*engEvalString(ep, "plot(T,D);");
	engEvalString(ep, "title('Position vs. Time for a falling object');");
	engEvalString(ep, "xlabel('Time (seconds)');");
	engEvalString(ep, "ylabel('Position (meters)');");*/

	/*
	 * use fgetc() to make sure that we pause long enough to be
	 * able to see the plot
	 */
	printf("Hit return to continue\n\n");
	fgetc(stdin);
	/*
	 * We're done for Part I! Free memory, close MATLAB figure.
	 */
	printf("Done for Part I.\n");
	mxDestroyArray(T);
	engEvalString(ep, "close;");


	/*
	 * PART II
	 *
	 * For the second half of this demonstration, we will request
	 * a MATLAB string, which should define a variable X.  MATLAB
	 * will evaluate the string and create the variable.  We
	 * will then recover the variable, and determine its type.
	 */
	  
	/*
	 * Use engOutputBuffer to capture MATLAB output, so we can
	 * echo it back.  Ensure first that the buffer is always NULL
	 * terminated.
	 */

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);
	while (result == NULL) {
	    char str[BUFSIZE+1];
	    /*
	     * Get a string input from the user
	     */
	    printf("Enter a MATLAB command to evaluate.  This command should\n");
	    printf("create a variable X.  This program will then determine\n");
	    printf("what kind of variable you created.\n");
	    printf("For example: X = 1:5\n");
	    printf(">> ");

	    fgets(str, BUFSIZE, stdin);
	  
	    /*
	     * Evaluate input with engEvalString
	     */
	   engEvalString(ep, str);
	    
	    /*
	     * Echo the output from the command.  
	     */
	    printf("%s", buffer);
	    
	    /*
	     * Get result of computation
	     */
	    printf("\nRetrieving X...\n");
	    if ((result = engGetVariable(ep,"X")) == NULL)
	      printf("Oops! You didn't create a variable X.\n\n");
	    else {
		printf("X is class %s\t\n", mxGetClassName(result));
	    }
	}

	/*
	 * We're done! Free memory, close MATLAB engine and exit.
	 */
	printf("Done!\n");
	mxDestroyArray(result);
	engClose(ep);
	
	return EXIT_SUCCESS;
}







