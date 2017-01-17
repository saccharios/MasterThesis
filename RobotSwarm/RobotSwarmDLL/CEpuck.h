#ifndef __CEPUCK_H__
#define __CEPUCK_H__

#include "CRobot.h"



class CEpuck : public CRobot
{

public:

	void Calibrate();
	void ObstacleAvoidanceOn();
	void ObstacleAvoidanceOff();


};

#endif // __CEPUCK_H__