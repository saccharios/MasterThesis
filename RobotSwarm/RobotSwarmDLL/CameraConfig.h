#ifndef _CAMERACONFIG_H_
#define _CAMERACONFIG_H_
// Author: Stefan Frei, 2013

//const int WIDTH_PIX = 1920; //in pixel = 265 cm
//const int HEIGHT_PIX = 1080; //in pixel = 195 cm
//
//const float WIDTH_WORLD = 330; //in cm - dependant on resolution and your camera - this is what the camera sees (given constant mapping from pix to cm)
//const float HEIGHT_WORLD = 200; //in cm - dependant on resolution and your camera - this is what the camera sees 
 //260 , 200
//
//// Define available region on the table. 
//const float OFFSET_X = 185; // in cm
//const float OFFSET_Y = 15; //in cm
////35, 15
//
//const float WIDTH_TABLE = 90;// in cm - effective available space on table, depends on resolution
//const float HEIGHT_TABLE = 175; //in cm - effective available space on table, depends on resolution
////260, 175
//// make sure that OFFSET_X + WIDTH_TABLE  <= WIDTH_WORLD and OFFSET_Y + HEIGHT_TABLE <= HEIGHT_WORLD 
//
//const float SAMPLING_TIME_CAMERA = 0.2f;

const int WIDTH_PIX = 640; //in pixel = 265 cm
const int HEIGHT_PIX = 480; //in pixel = 195 cm

const float WIDTH_WORLD = 260; //in cm - dependant on resolution and your camera - this is what the camera sees (given constant mapping from pix to cm)
const float HEIGHT_WORLD = 195; //in cm - dependant on resolution and your camera - this is what the camera sees 
 //260 , 195

// Define available region on the table. 
const float OFFSET_X = 10; // in cm, 10
const float OFFSET_Y = 10; //in cm, 10

const float WIDTH_TABLE = 225;// in cm - effective available space on table, depends on resolution
const float HEIGHT_TABLE = 175; //in cm - effective available space on table, depends on resolution
//240, 175
// make sure that OFFSET_X + WIDTH_TABLE  <= WIDTH_WORLD and OFFSET_Y + HEIGHT_TABLE <= HEIGHT_WORLD 

const float SAMPLING_TIME_CAMERA = 0.048f;


// Define Size of output windows (only display)
const int SHOW_XPIX = 640;
const int SHOW_YPIX = 480;



// Add userspecific configuration for outputs
// 1 = Alex
// 2 = Stefan
const int USER_NAME = 2;

// Choose extenden kalman filter type
// 1, inputs estimated
// 2, give inputs

const int EKFtype = 2;

#endif