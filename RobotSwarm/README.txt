--------------------------------------------------
README
--------------------------------------------------


HOW TO PROCEED TO ADD A NEW ALGORITHM TO THE CODE
--------------------------------------------------

1. Write your code into a header file "NameOfYourAlgorithm.h"

2. The header file has to be stored under ..\RobotSwarm\RobotSwarmGUI\bin\Debug\Algorithms

3. Go to the RobotSwarmDLL Project, open the file RobotSwarmDLL.cpp and include your header file.
   Now your algorithm will appear in the Algorithm-List of the GUI
 
4. There is a function called "Navigate(...)" in the file RobotSwarmDLL.cpp.
   Add the name of the header file (which is also the name of the algorithm) to it like it was done for the already
   existing algorithms "Walk" and "VirtualPotentialWalk". As you see you get the com port of the robot that is entering the
   "Navigate" - Function and you can also make use of its proximity sensor data, camera measurements etc. 


Take a look at the algorithm called "VirtualPotentialWalk", there it was done similarily. The header file can be found
at the same location as mentioned in step 2.



USING THE CAMERA
------------------

Since the camera tends to deliver unsharp images in the Autofocus - Mode should be turned off.

1. Open the folder "MyCam" (which can be found in the RobotSwarm folder) and start the .exe

2. Go to "Device" -> "Options" -> "Camera Control"

3. Disable "Auto" for "Focus" and set it manually to the maximum

Settings: Brigthness 150, Contrast: 160, Saturation: 255, Sharpness : 170, White Balance: yes


UPDATED:

HOW TO PROCEED TO ADD A NEW ALGORITHM TO THE CODE -OR-
--------------------------------------------------

1. Create an object that starts a thread (thread runns in parallel to main thread (i.e. camera & blob detection) )

2. The thread needs to have access to all robots & blobs, to send commands and retrieve new state updates

3. There are 2 types of Kalman filter implemented. The first runs on its own wihtout inputs, the second relies on inputs
(and is thus much more accurate). The filter can be switched in CamerConfig.h.

An example is CStochasticLocalization. 
Note that this is a complete different way than described above.


UPDATED:

ADDITIONLA LIBRARIES
----------------------
	
The code need following librarys to run:
1. opencv 2.4.3
2. boost 1.53
3. cvblobslib

Don't forget to set up the paths in the project settings properly.
On autwin164, they are already installed in C:\Apps\CppLib

