#ifndef __CELISA_H__
#define __CELISA_H__

#include "CRobot.h"
#include <boost\property_tree\ptree.hpp>
#include <boost\property_tree\xml_parser.hpp>
#include <boost\foreach.hpp>
//#include "boost_1_45_0\boost\property_tree\ptree.hpp"
//#include "boost_1_45_0\boost\property_tree\xml_parser.hpp"
//#include "boost_1_45_0\boost\foreach.hpp"

// macro for handling flags byte
#define FRONT_IR_ON(x) ((x) |= (1 << 1))
#define BACK_IR_ON(x) ((x) |= (1 << 0))
#define ALL_IR_ON(x) ((x) |= (1<<0) | (1 << 1))
#define TV_REMOTE_ON(x) ((x) |= (1<<2))
#define SLEEP_ON(x) ((x) = 0x08)
#define CALIBRATION_ON(x) ((x) |= (1<<4))
#define OBSTACLE_AVOID_ON(x) ((x) |= (1<<6))
#define CLIFF_AVOID_ON(x) ((x) |= (1<<7))
#define FRONT_IR_OFF(x) ((x) &= ~(1 << 1))
#define BACK_IR_OFF(x) ((x) &= ~(1 << 0))
#define ALL_IR_OFF(x) ((x) &= ~(1 << 0) & ~(1 << 1))
#define TV_REMOTE_OFF(x) ((x) &= ~(1 << 2))
#define SLEEP_OFF(x) ((x) &= ~(1 << 3))
#define CALIBRATION_OFF(x) ((x) &= ~(1 << 4))
#define OBSTACLE_AVOID_OFF(x) ((x) &= ~(1 << 6))
#define CLIFF_AVOID_OFF(x) ((x) &= ~(1 << 7))

#define RAD_2_DEG 57.2957796

#define NUM_ROBOTS 4
#define PAYLOAD_SIZE 13
#define ADDR_SIZE 2
#define ROBOT_PACKET_SIZE (PAYLOAD_SIZE+ADDR_SIZE)
#define PACKETS_SIZE 64
#define OVERHEAD_SIZE (2*NUM_ROBOTS+1)
#define UNUSED_BYTES 3

class CElisa : public CRobot{
private:
	 
	//
	int idevh;
	int current_address;
    int r;
	unsigned int i,j,k,delayCounter;
	
    unsigned int proxValue[8];
    unsigned int proxValueAmbient[8];
	unsigned int groundValue[4];
    unsigned int groundValueAmbient[4];
    unsigned int batteryLevel, batteryPercent;
    signed int accX, accY, accZ;
    int angle;
    unsigned char selector, tvRemote, flags;
    unsigned long int counter;
    char current_speed;
	char redLed, greenLed, blueLed;
	unsigned char smallLeds;
	unsigned char flagsTx;
    int ch;
    unsigned char obstacleAvoid, cliffAvoid, irOn, tvOn, sleepOn, exitProg, smallLedsOn;
    signed long int leftMotSteps, rightMotSteps;
	

	static int find_nrf_device(void);
	
	int USB_receive(char* data, int nbytes);
	int USB_send(char* data, int nbytes);
	void Transmit();

	void SetMotorSpeed(int iLeft, int iRight);
	
	

public:
	
	
	//static struct libusb_device_handle *devh;
	CElisa();
	~CElisa();
	bool Connect();
	void SetComPort(int iAddress);
	bool Disconnect();

	vector<int> LoadXMLFile();
	int Type();
	
	void RandomWalk(bool bCollisionAvoidance);
	int GetComPort();
	void Stop();
	void Exit();
	void Reset();

	void Calibrate();
	void ObstacleAvoidance_On_Off();

	void SetGreenLED();
	void GetBatteryState();
	vector<int> GetProximitySensorData();
	void GetGroundValue();
	//void GetSelectorPosition();
	void Accelerometer();
	void MotorSteps();
	
	
	
	//void Stop();
	//void Reset();
	//void SetMotorSpeed(string com, int iLeft, int iRight);
	//void RandomWalk();
	
};




#endif // __CELISA_H__