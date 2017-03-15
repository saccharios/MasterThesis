Welcome!
This repsitory contains the code which I and my predecessors wrote for my master thesis in electrical engineering at ETH Zürich.
There are two programs in the repo. 
The first is the firmware for an e-puck (the e-puck is a small robot with two wheels). 
To the supplier firmware I have added a special mode of navigaionin the file epuckFirmwareMod/DemoGCtronic/DemoMASF/rundevice.c.
The second program is RobotSwarm. 
It runs on the computer and steers several e-pucks over bluetooth with a camera as positioning system.
The camera generates pictures of the robots of which we extract their position and head angle. 
To faciliate this, the robots have a blue and black sticker on their top surface.
Because the measurement is quite noisy, and problems arise when two robots are close together (i.e. you cannot distinguish the individual robots anymore by looking at the picture), we use a measurement assignment algorithm to assign to each robot the best matching measurment.
We then run an Extended Kalman Filter (EKF) for each robot to obtain a good estimate of its positions, head angle and velocities.
The robots can either be freely moving around in a virtual force field, or some special localization algorithms can be chosen.
My thesis focus on these localization algorithms. 
The board on which the robots drive has a main virtual source, which the robots try to find.
The final report is also added in this repo, in case you are interested. 
If you look at the source files, I apologize in advance for the mess. 
I based my project on a previous project, so the architecture was already given.
Also, as this is my first major programming exercise the style and design is quite poor.
Here is a link (https://youtu.be/2cl1_fTFdQk) to a video on youtube where seven robots are moving according to "stochastic localization of sources".

