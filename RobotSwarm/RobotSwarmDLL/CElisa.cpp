#include "CElisa.h"
#include "libusb.h"
#include <iostream>
#include <set>

static struct libusb_device_handle *devh = NULL;

char RX_buffer[64] = {0};         // Last packet received from base station
char TX_buffer[64] = {0};          // Next packet to send to base station
BOOL bConnected = false;


CElisa::CElisa()
{
	//

	i,j,k,delayCounter=0;
	//RX_buffer[64]= 0;         // Last packet received from base station
	//TX_buffer[64]=0;         // Next packet to send to base station
    proxValue[8] = 0;
    proxValueAmbient[8] = 0;
    groundValue[4] = 0;
    groundValueAmbient[4] = 0;
    batteryLevel = 0, batteryPercent = 0;
    accX=0, accY=0, accZ=0;
    angle = 0;
    selector=0, tvRemote=0, flags=0;
    counter = 0;
    current_speed=0;
	redLed=0, greenLed=0, blueLed=0;
	smallLeds=0;
	flagsTx = 0x00;
    ch=0;
    obstacleAvoid=0, cliffAvoid=0, irOn=0, tvOn=0, sleepOn=0, exitProg=0, smallLedsOn=0;
    leftMotSteps=0, rightMotSteps=0;
	maxSpeed = 100;//speed in percentage, 100% correspond to 60cm/s
	speedFactor = maxSpeed/60.0f;


}

CElisa::~CElisa()
{
	
}

vector<int> CElisa::LoadXMLFile()
{
	using boost::property_tree::ptree;
	ptree pt;

	read_xml("ElisaAddress.xml",pt);
	//read_xml("\\\\tardis-second.ee.ethz.ch\\homes\\MSDocuments\\Visual Studio 2010\\Projects\\RobotSwarm\\RobotSwarmDLL\\ElisaAddress.xml",pt);
	vector<int> addresses;

	std::set<string> m_modules; 
	BOOST_FOREACH(ptree::value_type &v,
            pt.get_child("ElisaAddress"))
        m_modules.insert(v.second.data());

	
		/* TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (m_modules[i]>>8)&0xFF;     // address of the robot
		 TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;*/
		set<string>::iterator it;
		
  
  for ( it=m_modules.begin() ; it != m_modules.end(); it++ )
  {
	  stringstream ss; 
	  ss << *it;
	  ss >> current_address;
	  TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
	  TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	  
	
	  r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }
		
        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;

        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }
		
	  
		if((int)((unsigned char)RX_buffer[0]) == 2 ) { // if something goes wrong skip the data
            cout << endl;
			//printf("Elisa not found: %d", RX_buffer[0]);
            //cout << ", With address: " << current_address;
			//cout << endl;
        }
		else
		{
			//cout << "Elisa found with address: " << current_address << endl;
			addresses.push_back(current_address);
			//addresses[elementCount] = current_address;
			//elementCount++;
		}
	 
  }

  return addresses;

}

int CElisa::find_nrf_device()
{
	
	devh = (libusb_device_handle*)libusb_open_device_with_vid_pid(NULL, 0x1915, 0x0101);
	return devh ? 0 : -1;
	
	
}

int CElisa::USB_send(char* data, int nbytes)
{
	int transferred = 0;
	int r = 0;
	//static struct libusb_device_handle *udevh = NULL;
 	r = libusb_bulk_transfer(devh, 0x01, reinterpret_cast<unsigned char*>(data), 64, &transferred, 5000); // address 0x01
	if (r < 0) {
		fprintf(stderr, "bulk write error %d\n", r);
		return r;
	}
	if (transferred < nbytes) {
		fprintf(stderr, "short write (%d)\n", r);
		return -1;
	}
	
	return 0;

}

int CElisa::USB_receive(char* data, int nbytes)
{
	int received = 0;
	int r = 0;

	r = libusb_bulk_transfer(devh, 0x81, reinterpret_cast<unsigned char*>(data), nbytes, &received, 5000);
	if (r < 0) {
		fprintf(stderr, "bulk read error %d\n", r);
		return r;
	}
	if (received < nbytes) {
		fprintf(stderr, "short read (%d)\n", r);
		return -1;
	}

    return 0;
}

void CElisa::Transmit()
{
	cout << "Elisa leftSpeed: " << iLeftSpeed << " rightSpeed: " << iRightSpeed << endl;
	TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = redLed;                        // R
    TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = blueLed;				        // B
    TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = greenLed;                      // G
    TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTx;                       // activate IR remote control
    TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = iRightSpeed;                 // speed right (in percentage)
    TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = iLeftSpeed;                 // speed left (in percentage)
    TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = smallLeds;                     // small green leds
    TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	       // second robot
           /* TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = redLed;                        // R
            TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = blueLed;				        // B
            TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = greenLed;                      // G
            TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = flagsTx;                       // activate IR remote control
            TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = iRightSpeed;                  // speed right (in percentage)
            TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = iLeftSpeed;                 // speed left (in percentage)
            TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = smallLeds;                     // small green leds
            TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = ((current_address+1)>>8)&0xFF; // address of the robot
            TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = (current_address+1)&0xFF;

            // third robot
            TX_buffer[(2*ROBOT_PACKET_SIZE)+1] = redLed;                        // R
            TX_buffer[(2*ROBOT_PACKET_SIZE)+2] = blueLed;				        // B
            TX_buffer[(2*ROBOT_PACKET_SIZE)+3] = greenLed;                      // G
            TX_buffer[(2*ROBOT_PACKET_SIZE)+4] = flagsTx;                       // activate IR remote control
            TX_buffer[(2*ROBOT_PACKET_SIZE)+5] = iRightSpeed;                  // speed right (in percentage)
            TX_buffer[(2*ROBOT_PACKET_SIZE)+6] = iLeftSpeed;                 // speed left (in percentage)
            TX_buffer[(2*ROBOT_PACKET_SIZE)+7] = smallLeds;                     // small green leds
            TX_buffer[(2*ROBOT_PACKET_SIZE)+14] = ((current_address+2)>>8)&0xFF; // address of the robot
            TX_buffer[(2*ROBOT_PACKET_SIZE)+15] = (current_address+2)&0xFF;

            // fourth robot
            TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = redLed;                        // R
            TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = blueLed;				        // B
            TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = greenLed;                      // G
            TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = flagsTx;                       // activate IR remote control
            TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = iRightSpeed;                 // speed right (in percentage)
            TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = iLeftSpeed;                // speed left (in percentage)
            TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = smallLeds;                     // small green leds
            TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = ((current_address+3)>>8)&0xFF; // address of the robot
            TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = (current_address+3)&0xFF;

			*/

	
	// transfer the data to the base-station
        r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }
		
        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;
        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }
		
        // the base-station returns this "error" codes:
        // - 0 => transmission succeed (no ack received though)
        // - 1 => ack received (should not be returned because if the ack is received, then the payload is read)
        // - 2 => transfer failed
        /*if((int)((unsigned char)RX_buffer[0]) <= 2 ) { // if something goes wrong skip the data
            //printf("Communication problems: %d\n", RX_buffer[0]);
            continue;
        }
		*/
        // extract the sensors data for the first robot based on the packet id (first byte):
        // id=3 | prox0         | prox1         | prox2         | prox3         | prox5         | prox6         | prox7         | flags
        // id=4 | prox4         | gound0        | ground1       | ground2       | ground3       | accX          | accY          | tv remote
        // id=5 | proxAmbient0  | proxAmbient1  | proxAmbient2  | proxAmbient3  | proxAmbient5  | proxAmbient6  | proxAmbient7  | selector
        // id=6 | proxAmbient4  | goundAmbient0 | goundAmbient1 | goundAmbient2 | goundAmbient3 | accZ          | battery       | free byte

      /*  switch((int)((unsigned char)RX_buffer[0])) {

            case 3:
                proxValue[0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                proxValue[1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxValue[2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxValue[3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxValue[5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxValue[6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxValue[7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                flags = (unsigned char)RX_buffer[15];
                break;

            case 4:
                proxValue[4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundValue[0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundValue[1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundValue[2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundValue[3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accX = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                accY = (int)((RX_buffer[14]<<8)|(RX_buffer[13]));
                tvRemote = (unsigned char)RX_buffer[15];
                break;

            case 5:
                proxValueAmbient[0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                proxValueAmbient[1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxValueAmbient[2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxValueAmbient[3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxValueAmbient[5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxValueAmbient[6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxValueAmbient[7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                selector = (unsigned char)RX_buffer[15];
                break;

            case 6:
                proxValueAmbient[4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundValueAmbient[0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundValueAmbient[1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundValueAmbient[2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundValueAmbient[3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accZ = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                batteryLevel = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                // RX_buffer[15] is free
                break;

            case 7:
                leftMotSteps = ((signed long)((unsigned char)RX_buffer[4]<<24)| ((unsigned char)RX_buffer[3]<<16)| ((unsigned char)RX_buffer[2]<<8)|((unsigned char)RX_buffer[1]));
                rightMotSteps = ((signed long)((unsigned char)RX_buffer[8]<<24)| ((unsigned char)RX_buffer[7]<<16)| ((unsigned char)RX_buffer[6]<<8)|((unsigned char)RX_buffer[5]));
                //rightMotSteps = (long int)((RX_buffer[8]<<24)| (RX_buffer[7]<<16)| (RX_buffer[6]<<8)|(RX_buffer[5]));
                break;

        }*/
}


void CElisa::SetComPort(int iAddress)
{
	current_address = iAddress;
	iComPort = iAddress;
}

bool CElisa::Connect(){

	/*cout << "Enter the adress of Elisa:" << endl;
    cin >> current_address;
	cout << endl;*/
if(!bConnected)
{
	r = libusb_init(NULL);
	if (r < 0)
	{
		printf("libusb not initialized\n");
		//return;
		return false;
	}

	r = find_nrf_device();
	if (r < 0) {
		fprintf(stderr, "Could not find/open device\n");
		return false;
	}
	//printf("device found\n");
	
	r = libusb_claim_interface(devh, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
		return false;

	}
	bConnected = true;
}
	printf("Connected\n");
	//iComPort = iAddress; //current_address;
	

    TX_buffer[0]=0x27;  // Set command: change robots state

    //lspeed = 0;
    //rspeed = 0;
	
    i=0;

	/*if(LoadXMLFile().size() != NULL)
	{
		return true;
	}
	else
	{
		cout << endl << "No Elisa Addresses found" << endl;
		return false;
	}*/
	return true;

}

void CElisa::SetMotorSpeed(int iLeft, int iRight)
{ 
	//iLeft = SpeedConverter( (float)iLeft);
	//iRight = SpeedConverter((float)iRight);
	iLeftSpeed = iLeft;
	iRightSpeed = iRight;
	  
	  if(iLeft >= 0) {
			iLeftSpeed  = iLeft|0x80;
			cout << "SetMotorSpeed iLeft>=0: " << iLeftSpeed << ", iLeft: " << iLeft;
		} 
	  else {
			iLeftSpeed = (-iLeft)&0x7F;
			cout << "SetMotorSpeed iLeft<0: " << iLeftSpeed << ", iLeft: " << iLeft;
    }

	  
	  if(iRight >= 0) {
			iRightSpeed = iRight|0x80;
			cout << "SetMotorSpeed iRight>=0: " << iRightSpeed << ", iRight: " << iRight;

		} 
	  else {
        iRightSpeed =(-iRight)&0x7F;
		cout << "SetMotorSpeed iRight<0: " << iRightSpeed << ", iRight: " << iRight;
		
    }
	  cout << endl;
}

void CElisa::RandomWalk(bool bCollisionAvoidance)
{
	SetMotorSpeed(25,25);
	Transmit();
}

int CElisa::GetComPort()
{
	return current_address;
}

void CElisa::Stop()
{
	SetMotorSpeed(0,0);
	Transmit();
}

void CElisa::Calibrate()
{
	CALIBRATION_ON(flagsTx);
	TX_buffer[0]=0x27;
	Transmit();
}

void CElisa::ObstacleAvoidance_On_Off()
{
	if(!obstacleAvoid) 
	{
		obstacleAvoid = 1;
        OBSTACLE_AVOID_ON(flagsTx);
     } 
	
	TX_buffer[0]=0x27;
	Transmit();

}

void CElisa::SetGreenLED()
{

}

void CElisa::GetBatteryState()
{
    TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;
	//TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTx;

	

		while((int)((unsigned char)RX_buffer[0]) != 6)
		{
			 r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }
		
        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;

        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

		if((int)((unsigned char)RX_buffer[0]) <= 2 ) { // if something goes wrong skip the data
            printf("Communication problems: %d\n", RX_buffer[0]);
            
        }
		}

        // extract the sensors data for the first robot based on the packet id (first byte):
        // id=3 | prox0         | prox1         | prox2         | prox3         | prox5         | prox6         | prox7         | flags
        // id=4 | prox4         | gound0        | ground1       | ground2       | ground3       | accX          | accY          | tv remote
        // id=5 | proxAmbient0  | proxAmbient1  | proxAmbient2  | proxAmbient3  | proxAmbient5  | proxAmbient6  | proxAmbient7  | selector
        // id=6 | proxAmbient4  | goundAmbient0 | goundAmbient1 | goundAmbient2 | goundAmbient3 | accZ          | battery       | free byte

        if((int)((unsigned char)RX_buffer[0]) == 6) {

                batteryLevel = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
               if(batteryLevel >= 934) {           // 934 is the measured adc value when the battery is charged
            batteryPercent = 100;
        } else if(batteryLevel <= 780) {    // 780 is the measrued adc value when the battery is discharged
            batteryPercent = 0;
        } else {
            batteryPercent = (unsigned int)((float)((batteryLevel-780.0)/(934.0-780.0))*100.0);
        }

		printf("BatteryLevel: %4d (~%3d%%)\n\r\n", batteryLevel, batteryPercent);
		}

       



	//batteryLevel = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
	//printf("BatteryLevel: %4d (~%3d%%)\n\r\n", batteryLevel, batteryPercent);
}

vector<int> CElisa::GetProximitySensorData()
{
	vector<int> vProximityData;
	TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	for(int i = 0;i < 5; i++)
	{
		// transfer the data to the base-station
        r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }

        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;
        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

	if((int)((unsigned char)RX_buffer[0]) == 3) {
		proxValue[0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
		proxValue[1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
		proxValue[2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
		proxValue[3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
		proxValue[5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
		proxValue[6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
		proxValue[7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
	}
	else if((int)((unsigned char)RX_buffer[0]) == 4)
	{
		proxValue[4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
	}
	else if((int)((unsigned char)RX_buffer[0]) == 5)
	{
		proxValueAmbient[0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
        proxValueAmbient[1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
        proxValueAmbient[2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
        proxValueAmbient[3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
        proxValueAmbient[5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
        proxValueAmbient[6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
        proxValueAmbient[7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
	}
	else if((int)((unsigned char)RX_buffer[0]) == 6)
	{
		proxValueAmbient[4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
	}
	}

	/*printf("PROXIMITY\r\n");
    printf("Prox0\t Prox1\t Prox2\t Prox3\t Prox4\t Prox5\t Prox6\t Prox7\r\n");
    printf("%4d\t %4d\t %4d\t %4d\t %4d\t %4d\t %4d\t %4d\t\n\r\n", proxValue[0], proxValue[1], proxValue[2], proxValue[3], proxValue[4], proxValue[5], proxValue[6], proxValue[7]);
    printf("PROXIMITY AMBIENT\r\n");
    printf("Prox0\t Prox1\t Prox2\t Prox3\t Prox4\t Prox5\t Prox6\t Prox7\r\n");
    printf("%4d\t %4d\t %4d\t %4d\t %4d\t %4d\t %4d\t %4d\t\n\r\n", proxValueAmbient[0], proxValueAmbient[1], proxValueAmbient[2], proxValueAmbient[3], proxValueAmbient[4], proxValueAmbient[5], proxValueAmbient[6], proxValueAmbient[7]);
	*/
	for(int i=0; i<8; i++)
	{
		vProximityData.push_back(proxValue[i]);
	}

	return vProximityData;
}

void CElisa::GetGroundValue()
{
	TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	for(int i = 0;i < 5; i++)
	{
		// transfer the data to the base-station
        r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }

        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;
        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

	if((int)((unsigned char)RX_buffer[0]) == 4)
	{
		groundValue[0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
        groundValue[1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
        groundValue[2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
        groundValue[3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
	}
	else if((int)((unsigned char)RX_buffer[0]) == 6)
	{
		groundValueAmbient[0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
        groundValueAmbient[1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
        groundValueAmbient[2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
        groundValueAmbient[3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
	}
	}

	printf("GROUND\r\n");
    printf("ground0\t ground1\t ground2\t ground3\r\n");
    printf("%4d\t %4d\t\t %4d\t\t %4d\t\n\r\n", groundValue[0], groundValue[1], groundValue[2], groundValue[3]);
    printf("GROUND AMBIENT\r\n");
    printf("ground0\t ground1\t ground2\t ground3\r\n");
    printf("%4d\t %4d\t\t %4d\t\t %4d\t\n\r\n", groundValueAmbient[0], groundValueAmbient[1], groundValueAmbient[2], groundValueAmbient[3]);
}

void CElisa::Accelerometer()
{
	TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	for(int i = 0;i < 5; i++)
	{
		// transfer the data to the base-station
        r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }

        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;
        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

	if((int)((unsigned char)RX_buffer[0]) == 4)
	{
		accX = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
        accY = (int)((RX_buffer[14]<<8)|(RX_buffer[13]));
	}
	else if((int)((unsigned char)RX_buffer[0]) == 6)
	{
		accZ = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
	}

	}

	printf("ACCELEROMETER\r\n");
    printf("X:%4d\t Y:%4d\t Z:%4d\t\r\n", accX, accY, accZ);



}

/*void CElisa::GetSelectorPosition()
{

	while((int)((unsigned char)RX_buffer[0]) != 5)
		{
			 r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }
		
        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;

        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

		if((int)((unsigned char)RX_buffer[0]) <= 2 ) { // if something goes wrong skip the data
            printf("Communication problems: %d\n", RX_buffer[0]);
            
        }
		}
	selector = (unsigned char)RX_buffer[15];
	printf("SELECTOR\r\n");
    printf("%.2d\n\r\n", selector);
}*/


void CElisa::MotorSteps()
{
	TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (current_address>>8)&0xFF;     // address of the robot
    TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address&0xFF;

	while((int)((unsigned char)RX_buffer[0]) != 7)
		{
			 r = USB_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
        if(r < 0) {
            printf("send error!\n");
        }
		
        RX_buffer[0] = 0;
        RX_buffer[1] = 0;
        RX_buffer[2] = 0;
        RX_buffer[3] = 0;

        r = USB_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }

		if((int)((unsigned char)RX_buffer[0]) <= 2 ) { // if something goes wrong skip the data
            printf("Communication problems: %d\n", RX_buffer[0]);
            
        }
	}

	 leftMotSteps = ((signed long)((unsigned char)RX_buffer[4]<<24)| ((unsigned char)RX_buffer[3]<<16)| ((unsigned char)RX_buffer[2]<<8)|((unsigned char)RX_buffer[1]));
     rightMotSteps = ((signed long)((unsigned char)RX_buffer[8]<<24)| ((unsigned char)RX_buffer[7]<<16)| ((unsigned char)RX_buffer[6]<<8)|((unsigned char)RX_buffer[5]));
	 printf("MOTORS ENCODERS\r\n");
     printf("l: %+.10ld r: %+.10ld\n\r\n", leftMotSteps, rightMotSteps);
}
void CElisa::Reset()
{
	Stop();
}

bool CElisa::Disconnect()
{
	if(bConnected = true)
	{
		libusb_release_interface(devh, 0);
		libusb_close(devh);
		libusb_exit(NULL);
		bConnected = false;
	}
	else{
		cout << "Elisa Channel is already closed" << endl;
	}
	return true;
}

int CElisa::Type(){

	return 2;
}

void CElisa::Exit()
{
	//Stop();
	Disconnect();
}