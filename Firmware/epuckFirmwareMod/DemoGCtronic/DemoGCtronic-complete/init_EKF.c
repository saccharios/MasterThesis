//#include "init_EKF.h"
#include "utility.h"
#include <uart/e_uart_char.h> //Communication
#define uart1_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
extern int MSIZE_MAX;


void Tr_MatrixMultiply(unsigned int nrowA,unsigned int ncolArowB, unsigned int ncolB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX])
{
  //Res = A*B;

  int r, c, k; //iteration variables
  float sum = 0.0; 

  //Validity checks
/*
	if (nrowA>MSIZE_MAX || ncolArowB>MSIZE_MAX || ncolB>MSIZE_MAX)
	{
		uart1_send_static_text("Tr_MatrixMultiply: Error: Trying to mutliply a matrix larger than MSIZE_MAX\r\n");
		return; //wrong matrix size
	}
*/
  for ( r = 0 ; r < nrowA ; r++ )
  {
    for ( c = 0 ; c < ncolB ; c++ )
    {
      for ( k = 0 ; k < ncolArowB ; k++ )
      {
        sum = sum + A[ncolArowB*r+k]*B[ncolB*k+c];
      }
      Res[ncolB*r+c] = sum;
      sum = 0.0;
    }
  }
}

void Tr_MatrixAdd(unsigned int nrowAB,unsigned int ncolAB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX])
// CAN BE CALCULATED IN PLACE, i.e. "A = A+B" or "A = B+A" is allowed 
{
  //Res = A+B;

  int r, c; //iteration variables

  //Validity checks
/*
	if (nrowAB>MSIZE_MAX || ncolAB>MSIZE_MAX)
	{
		uart1_send_static_text("Tr_MatrixAdd: Error: Trying to add a matrices larger than MSIZE_MAX\r\n");
		return; //wrong matrix size
	}
*/
  for ( r = 0 ; r < nrowAB ; r++ )
  {
    for ( c = 0 ; c < ncolAB ; c++ )
    {
      Res[ncolAB*r+c] = A[ncolAB*r+c]+B[ncolAB*r+c];
    }
  }
}

void Tr_MatrixSubtract(unsigned int nrowAB,unsigned int ncolAB, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX], float B[MSIZE_MAX*MSIZE_MAX])
// CAN BE CALCULATED IN PLACE, i.e. "A = A-B"  is allowed 
{
  //Res = A-B;

  int r, c; //iteration variables

  //Validity checks
/*
	if (nrowAB>MSIZE_MAX || ncolAB>MSIZE_MAX)
	{
		uart1_send_static_text("Tr_MatrixAdd: Error: Trying to add a matrices larger than MSIZE_MAX\r\n");
		return; //wrong matrix size
	}
*/
  for ( r = 0 ; r < nrowAB ; r++ )
  {
    for ( c = 0 ; c < ncolAB ; c++ )
    {
      Res[ncolAB*r+c] = A[ncolAB*r+c]-B[ncolAB*r+c];
    }
  }
}

void Tr_MatrixTranspose(unsigned int nrowA, unsigned int ncolA, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX])
{
  //Res = A';

  int r, c; //iteration variables

  //Validity checks
/*
	if (nrowA>MSIZE_MAX || ncolA>MSIZE_MAX)
	{
		uart1_send_static_text("Tr_MatrixTranspose: Error: Trying to transpose a matrix larger than MSIZE_MAX\r\n");
		return; //wrong matrix size
	}
*/
  for ( r = 0 ; r < nrowA ; r++ )
  {
    for ( c = 0 ; c < ncolA ; c++ )
    {
      Res[nrowA*c+r] = A[ncolA*r+c];
    }
  }
}

void Tr_MatrixSubtractFromId (unsigned int nrowAcolA, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX])
// CAN BE CALCULATED IN PLACE, i.e. "A = (I-A)" is allowed
{
  //Res = I - A; , where I is identity matrix of the same size as A (A must be a square matrix)

  int r, c; //iteration variables

  for ( r = 0 ; r < nrowAcolA ; r++ )
  {
    for ( c = 0 ; c < nrowAcolA ; c++ )
    {
	  if (c==r)
	  {
      	Res[nrowAcolA*r+c] = 1.0-A[nrowAcolA*r+c];
	  } else {
		Res[nrowAcolA*r+c] = -A[nrowAcolA*r+c];
	  }
    }
  }
}

int Tr_MatrixInverse3x3(float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX])
{
  //Res = inv(A); , where A is must be a 3x3 matrix

float Det = A[0]*(A[8]*A[4]-A[7]*A[5])-A[3]*(A[8]*A[1]-A[7]*A[2])+A[6]*(A[5]*A[1]-A[4]*A[2]);
	if (Det!=0.0)
	{
		Res[0] = (A[8]*A[4]-A[7]*A[5])/Det;
		Res[1] = -(A[8]*A[1]-A[7]*A[2])/Det;
		Res[2] = (A[5]*A[1]-A[4]*A[2])/Det;
		Res[3] = -(A[8]*A[3]-A[6]*A[5])/Det;
		Res[4] = (A[8]*A[0]-A[6]*A[2])/Det;
		Res[5] = -(A[5]*A[0]-A[3]*A[2])/Det;	
		Res[6] = (A[7]*A[3]-A[6]*A[4])/Det;
		Res[7] = -(A[7]*A[0]-A[6]*A[1])/Det;
		Res[8] = (A[4]*A[0]-A[3]*A[1])/Det;		
		return 0;
	} else {
		uart1_send_static_text("Tr_MatrixInverse3x3: Error: det(A)=0\r\n");
		Res[0] = 0.0;
		Res[1] = 0.0;
		Res[2] = 0.0;
		Res[3] = 0.0;
		Res[4] = 0.0;
		Res[5] = 0.0;
		Res[6] = 0.0;
		Res[7] = 0.0;
		Res[8] = 0.0;
		return 1;
	}
}

int Tr_MatrixInverse2x2(float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX])
{
  //Res = inv(A); , where A is must be a 2x2 matrix

float Det = A[0]*A[3] - A[1]*A[2];
	if (Det!=0.0)
	{
		Res[0] = A[3]/Det;
		Res[1] = -A[1]/Det;
		Res[2] = -A[2]/Det;
		Res[3] = A[0]/Det;
		return 0;	
	} else {
		uart1_send_static_text("Tr_MatrixInverse2x2: Error: det(A)=0\r\n");
		Res[0] = 0.0;
		Res[1] = 0.0;
		Res[2] = 0.0;
		Res[3] = 0.0;
		return 1;
	}
}

void Tr_MatrixTimeShift(unsigned int nrowA, unsigned int ncolA, unsigned int N_TimeSteps, float Res[MSIZE_MAX*MSIZE_MAX], float A[MSIZE_MAX*MSIZE_MAX])
{
  //Shifts matrix A back one time step, i.e. the oldest entry is destroyed and all others are shifted one step back. The newest entry stays the same.
  //Res = A[k+1];

  int r, c, k; //iteration variables

	if (N_TimeSteps > 1)
	{
		for (k = (N_TimeSteps-1); k>0; k--)
		{
		  for ( r = 0 ; r < nrowA ; r++ )
		  {
		    for ( c = 0 ; c < ncolA ; c++ )
		    {
		      Res[(nrowA*ncolA)*k + nrowA*c+r] = A[(nrowA*ncolA)*(k-1) + nrowA*c+r];
		    }
		  }
		}
	}
}

