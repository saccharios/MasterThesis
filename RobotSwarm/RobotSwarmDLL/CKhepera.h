#ifndef __CKHEPERA_H__
#define __CKHEPERA_H__
#include "CRobot.h"


class CKhepera : public CRobot{
private:
	
	void SetCom();

public:
	
	CKhepera();
	~CKhepera();

	vector<int> vProximity;

	void Stop();
	void Reset();
	void SetMotorSpeed(int iLeft, int iRight);
	void RandomWalk(bool bCollisionAvoidance);
	void CollisionAvoidance();
	
	void SetTargetProfile(int iLeft, int iRight);
	void GetUS_PeriferalMeasure(int i);
	void GetBatteryState(int iMeasurement);


	int Type();
	
};

#endif // __CKHEPERA_H__