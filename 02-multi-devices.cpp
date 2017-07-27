/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.
* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
\author    <http://www.chai3d.org>
\author    Francois Conti
\version   3.1.1 $Rev: 1869 $
*/
//==============================================================================
#include <fstream>
#include <string>
#include <math.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
//------------------------------------------------------------------------------
#include "chai3d.h"


//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif


extern "C" {
#include "extApi.h"
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// maximum number of devices supported by this application
const int MAX_DEVICES = 16;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// labels to display each haptic device model
cLabel* labelHapticDeviceModel[MAX_DEVICES];

// labels to display the position [m] of each haptic device
cLabel* labelHapticDevicePosition[MAX_DEVICES];

// global variable to store the position [m] of each haptic device
cVector3d hapticDevicePosition[MAX_DEVICES];

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// small spheres (cursor) representing position of each haptic device
cShapeSphere* cursor[MAX_DEVICES];

// lines representing the velocity vector of each haptic device
cShapeLine* velocity[MAX_DEVICES];

// flag for using damping (ON/OFF)
bool useDamping = false;

// flag for using force field (ON/OFF)
bool useForceField = false;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// Global variables
double fmax = 15;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics();


// Our functions 
bool HysteresisDirection(float a, float b);
double DoHysteresis(double distance, bool direction, std::string &last_state, std::string &state, std::vector<double> radius, double m, std::vector<double> b, int index);
double ComputeForce(double m, double b, double dist);
void ComputeSlopesParameters(double R1, double R2, double delta_x, double &m, std::vector<double> &b);
bool graspBall(std::vector<float> distance, double R, bool attached);

//flag for stopping motion when crossing a position boundary from the torso frame
bool stop = false;
bool attachball = false;
// global variables
double dist = 1.0;

//Remote v-rep api connection variable
int clientID;
//Ball Handle for remote v-rep
int b_handle;


//==============================================================================
/*
DEMO:   02-multi-devices.cpp
This application illustrates how to program forces, torques and gripper
forces on multiple haptic device.
In this example the application opens an OpenGL window and displays a
3D cursor for each device connected to your computer. If the user presses
onto the user button (if available on your haptic device), the color of
the cursor changes from blue to green.
This example is very similar to 01-devices, but extends support for multiple
haptic devices
*/
//==============================================================================

int main(int argc, char* argv[])
{
	clientID = simxStart((simxChar*)"192.168.137.42", 19997, true, true, 2000, 5);


	if (clientID != -1)
	{
		printf("Connected to remote API server\n");

		//simxGetObjectHandle(clientID, "Sphere", &b_handle, simx_opmode_blocking);//simx_opmode_oneshot_wait);
		//float position[3];
		////Get position of the ball
		//simxGetObjectPosition(clientID, b_handle, -1, position, simx_opmode_blocking);
		//printf("Ball's position is : %f, \t %f, \t %f\n ", position[0], position[1], position[2]);


		// Now send some data to V-REP in a non-blocking fashion:
		simxAddStatusbarMessage(clientID, "Hello V-REP!", simx_opmode_oneshot);


		//--------------------------------------------------------------------------
		// INITIALIZATION
		//--------------------------------------------------------------------------

		cout << endl;
		cout << "-----------------------------------" << endl;
		cout << "CHAI3D" << endl;
		cout << "Demo: 02-multi-devices" << endl;
		cout << "Copyright 2003-2016" << endl;
		cout << "-----------------------------------" << endl << endl << endl;
		cout << "Keyboard Options:" << endl << endl;
		cout << "[1] - Enable/Disable potential field" << endl;
		cout << "[2] - Enable/Disable damping" << endl;
		cout << "[f] - Enable/Disable full screen mode" << endl;
		cout << "[m] - Enable/Disable vertical mirroring" << endl;
		cout << "[x] - Exit application" << endl;
		cout << endl << endl;


		//--------------------------------------------------------------------------
		// OPENGL - WINDOW DISPLAY
		//--------------------------------------------------------------------------

		// initialize GLUT
		glutInit(&argc, argv);

		// retrieve  resolution of computer display and position window accordingly
		screenW = glutGet(GLUT_SCREEN_WIDTH);
		screenH = glutGet(GLUT_SCREEN_HEIGHT);
		windowW = 0.8 * screenH;
		windowH = 0.5 * screenH;
		windowPosY = (screenH - windowH) / 2;
		windowPosX = windowPosY;

		// initialize the OpenGL GLUT window
		glutInitWindowPosition(windowPosX, windowPosY);
		glutInitWindowSize(windowW, windowH);

		if (stereoMode == C_STEREO_ACTIVE)
			glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
		else
			glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);


		// create display context and initialize GLEW library
		glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
		// initialize GLEW
		glewInit();
#endif

		// setup GLUT options
		glutDisplayFunc(updateGraphics);
		glutKeyboardFunc(keySelect);
		glutReshapeFunc(resizeWindow);
		glutSetWindowTitle("CHAI3D");

		// set fullscreen mode
		if (fullscreen)
		{
			glutFullScreen();
		}


		//--------------------------------------------------------------------------
		// WORLD - CAMERA - LIGHTING
		//--------------------------------------------------------------------------

		// create a new world.
		world = new cWorld();

		// set the background color of the environment
		world->m_backgroundColor.setBlack();

		// create a camera and insert it into the virtual world
		camera = new cCamera(world);
		world->addChild(camera);

		// position and orient the camera
		camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
			cVector3d(0.0, 0.0, 0.0),    // look at position (target)
			cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

										 // set the near and far clipping planes of the camera
		camera->setClippingPlanes(0.01, 10.0);

		// set stereo mode
		camera->setStereoMode(stereoMode);

		// set stereo eye separation and focal length (applies only if stereo is enabled)
		camera->setStereoEyeSeparation(0.01);
		camera->setStereoFocalLength(0.5);

		// set vertical mirrored display mode
		camera->setMirrorVertical(mirroredDisplay);

		// create a directional light source
		light = new cDirectionalLight(world);

		// insert light source inside world
		world->addChild(light);

		// enable light source
		light->setEnabled(true);

		// define direction of light beam
		light->setDir(-1.0, 0.0, 0.0);

		// create a font
		cFont *font = NEW_CFONTCALIBRI20();


		//--------------------------------------------------------------------------
		// HAPTIC DEVICES
		//--------------------------------------------------------------------------

		// create a haptic device handler
		handler = new cHapticDeviceHandler();

		// get number of haptic devices
		numHapticDevices = handler->getNumDevices();

		// setup each haptic device
		for (int i = 0; i < numHapticDevices; i++)
		{
			// get a handle to the first haptic device
			handler->getDevice(hapticDevice[i], i);

			// open a connection to haptic device
			hapticDevice[i]->open();

			// calibrate device (if necessary)
			hapticDevice[i]->calibrate();

			// retrieve information about the current haptic device
			cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

			// create a sphere (cursor) to represent the haptic device
			cursor[i] = new cShapeSphere(0.01);

			// insert cursor inside world
			world->addChild(cursor[i]);

			// create small line to illustrate the velocity of the haptic device
			velocity[i] = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));

			// insert line inside world
			world->addChild(velocity[i]);


			// display a reference frame if haptic device supports orientations
			if (info.m_sensedRotation == true)
			{
				// display reference frame
				cursor[i]->setShowFrame(true);

				// set the size of the reference frame
				cursor[i]->setFrameSize(0.05);
			}

			// if the device has a gripper, enable the gripper to simulate a user switch
			hapticDevice[i]->setEnableGripperUserSwitch(true);

			// create a label to display the haptic device model
			labelHapticDeviceModel[i] = new cLabel(font);
			camera->m_frontLayer->addChild(labelHapticDeviceModel[i]);
			labelHapticDeviceModel[i]->setText(info.m_modelName);

			// create a label to display the position of haptic device
			labelHapticDevicePosition[i] = new cLabel(font);
			camera->m_frontLayer->addChild(labelHapticDevicePosition[i]);
		}

		// create a label to display the haptic rate of the simulation
		labelHapticRate = new cLabel(font);
		camera->m_frontLayer->addChild(labelHapticRate);


		//--------------------------------------------------------------------------
		// START SIMULATION
		//--------------------------------------------------------------------------
		//string ip = "192.168.19.101";
		//AL::ALMotionProxy motion("192.168.137.56", 9559);
		//motion.wakeUp();
		//motion.move(1.0f, 0.0f, 0.0f);



		// create a thread which starts the main haptics rendering loop
		cThread* hapticsThread = new cThread();
		hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

		// setup callback when application exits
		atexit(close);

		// start the main graphics rendering loop
		glutTimerFunc(50, graphicsTimer, 0);
		glutMainLoop();

		// Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
		int pingTime;
		simxGetPingTime(clientID, &pingTime);

		// Now close the connection to V-REP:   
		simxFinish(clientID);
	}

	// exit
	return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
	windowW = w;
	windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
	// option ESC: exit
	if ((key == 27) || (key == 'x'))
	{
		exit(0);
	}

	// option 1: enable/disable force field
	if (key == '1')
	{
		useForceField = !useForceField;
		if (useForceField)
			cout << "> Enable force field     \r";
		else
			cout << "> Disable force field    \r";
	}

	// option 2: enable/disable damping
	if (key == '2')
	{
		useDamping = !useDamping;
		if (useDamping)
			cout << "> Enable damping         \r";
		else
			cout << "> Disable damping        \r";
	}

	// option f: toggle fullscreen
	if (key == 'f')
	{
		if (fullscreen)
		{
			windowPosX = glutGet(GLUT_INIT_WINDOW_X);
			windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
			windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
			windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
			glutPositionWindow(windowPosX, windowPosY);
			glutReshapeWindow(windowW, windowH);
			fullscreen = false;
		}
		else
		{
			glutFullScreen();
			fullscreen = true;
		}
	}

	// option m: toggle vertical mirroring
	if (key == 'm')
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	for (int i = 0; i < numHapticDevices; i++)
	{
		hapticDevice[i]->close();
	}
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
	if (simulationRunning)
	{
		glutPostRedisplay();
	}

	glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	int step = 40;
	for (int i = 0; i < numHapticDevices; i++)
	{
		// update position of label
		labelHapticDeviceModel[i]->setLocalPos(20, windowH - step, 0);
		step += 20;

		// display new position data
		labelHapticDevicePosition[i]->setText(hapticDevicePosition[i].str(3));

		// update position of label
		labelHapticDevicePosition[i]->setLocalPos(20, windowH - step, 0);
		step += 25;
	}

	// display haptic rate data
	if (numHapticDevices == 0)
	{
		labelHapticRate->setText("no haptic device detected");
	}
	else
	{
		labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");
	}

	// update position of label
	labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(windowW, windowH);

	// swap buffers
	glutSwapBuffers();

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics()
{
	// initialize frequency counter
	frequencyCounter.reset();

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// Creating the Motion constructor
	AL::ALMotionProxy motion("192.168.137.42", 9559);
	motion.wakeUp();

	//Moving hand-effector variables
	std::string chainNameLeft = "LArm";
	std::string chainNameRight = "RArm";
	int space = 0;

	float fractionMaxSpeed = 0.2f;
	int axisMask = 7;

	//Remote v-rep information
	simxGetObjectHandle(clientID, "Sphere", &b_handle, simx_opmode_blocking);
	int h_handle;
	int t_handle;
	simxGetObjectHandle(clientID, "imported_part_36_sub", &h_handle, simx_opmode_blocking);
	simxGetObjectHandle(clientID, "torso_respondable1cyl0", &t_handle, simx_opmode_blocking);

	float hand_position_vrep[3];
	float ball_position[3];
	//float ball_orientation[3];

	//Get position of the ball
	simxGetObjectPosition(clientID, b_handle, t_handle, ball_position, simx_opmode_blocking);
	//	simxGetObjectOrientation(clientID, b_handle, h_handle, ball_orientation, simx_opmode_blocking);

	printf("Ball's position is : %f, \t %f, \t %f\n ", ball_position[0], ball_position[1], ball_position[2]);
	float original_ball_position[3];
	original_ball_position[0] = -0.40315;
	original_ball_position[1] = 0.2;
	original_ball_position[2] = 0.23713;
	//	simxGetObjectPosition(clientID, h_handle, t_handle, hand_position_vrep, simx_opmode_blocking);
	//	printf("Hand's position is : %f, \t %f, \t %f\n ", hand_position_vrep[0], hand_position_vrep[1], hand_position_vrep[2]);


	//motion.move(1.0f, 0.0f, 0.0f);
	std::vector<float> command_pose(6, 0.0f); // Absolute Position
	std::vector<float> nao_pose(6, 0.0f);
	std::vector<float> ball_pose(6, 0.0f);
	ball_pose[0] = ball_position[0];// 0.179675; //
	ball_pose[1] = ball_position[1];// -0.131535; //
	ball_pose[2] = ball_position[2];// 0.00880959;  //

									//// Hello, my name is Alessandra, and I am not a robotcist ! That's why I don't compute force profiles. Grazie. Ciao.
									//float large_threshold = 0.08;
									//float medium_threshold = 0.06;
									//float ball_radius = 0.05;


	float small_threshold = 0.05;

	bool movement_allowed = false;
	bool apply_force = false;

	//Distances 
	// Initialize far from the center of the sphere
	std::vector <float> distance_t0(2, 10);
	std::vector <float> distance_t1(2, 0);

	// Initialize the force gains
	std::vector <float> Kp(2, 0);


	// Compute the slopes
	double m = 0.0;
	std::vector <double> b(2, 0);
	double delta_x = 0.04;
	double R1 = 0.15;
	double R2 = 0.09;
	double R1p = R1 + delta_x;
	double R2p = R2 - delta_x;

	std::vector <std::string> last_distance_state(2, "outside");
	std::vector <std::string> distance_state(2, "outside");

	std::vector <double> sphere_radius(4, 0);
	sphere_radius[0] = R1;
	sphere_radius[1] = R1p;
	sphere_radius[2] = R2;
	sphere_radius[3] = R2p;

	ComputeSlopesParameters(R1p, R2, delta_x, m, b);

	// Open the text file 
	std::ofstream SaveFileHaptics("haptics.txt");
	//SaveFileHaptics.open("haptics.txt",std::ios::in);
	// Initalizing the data tables
	

	if (SaveFileHaptics.is_open()) {
		std::cout << "Log file opened correctly" << std::endl;
		SaveFileHaptics << "Distance 1" << " , " << "Force 1" << " , " << "Distance 2" << " , " << "Force 2" << "\n";
		//SaveFileHaptics << std::to_string(static_cast<long double>(distance_t1[0])) << " , " << std::to_string(static_cast<long double>(Kp[0])) << std::to_string(static_cast<long double>(distance_t1[1])) << " , " << std::to_string(static_cast<long double>(Kp[1])) << "\n";
	}
	else
		std::cout << "Error by opening the log file" << std::endl;

	// Samples counter for saving data
	int count_samples = 1;

	// main haptic simulation loop
	while (simulationRunning)
	{

		

		for (int i = 0; i < numHapticDevices; i++){
			/////////////////////////////////////////////////////////////////////
			// READ HAPTIC DEVICE
			/////////////////////////////////////////////////////////////////////

			std::vector<float> nao_pose_right = motion.getPosition(chainNameRight, space, true);
			std::vector<float> nao_pose_left = motion.getPosition(chainNameLeft, space, true);
			// Position of the sphere
			//cVector3d spherePosition;
			//spherePosition.set(0.0849656, -0.0505495, 0.0394306);

			// read position 
			cVector3d position;
			cVector3d position_world;
			hapticDevice[i]->getPosition(position);

			// read orientation 
			cMatrix3d rotation;
			hapticDevice[i]->getRotation(rotation);

			// read gripper position
			double gripperAngle;
			hapticDevice[i]->getGripperAngleRad(gripperAngle);

			// read linear velocity 
			cVector3d linearVelocity;
			hapticDevice[i]->getLinearVelocity(linearVelocity);

			// read angular velocity
			cVector3d angularVelocity;
			hapticDevice[i]->getAngularVelocity(angularVelocity);

			// read gripper angular velocity
			double gripperAngularVelocity;
			hapticDevice[i]->getGripperAngularVelocity(gripperAngularVelocity);

			// read user-switch status (button 0)
			bool button0, button1, button2, button3;
			button0 = false;
			button1 = false;
			button2 = false;
			button3 = false;


			hapticDevice[i]->getUserSwitch(0, button0);
			hapticDevice[i]->getUserSwitch(1, button1);
			hapticDevice[i]->getUserSwitch(2, button2);
			hapticDevice[i]->getUserSwitch(3, button3);


			/////////////////////////////////////////////////////////////////////
			// UPDATE 3D CURSOR MODEL
			/////////////////////////////////////////////////////////////////////

			// update arrow
			velocity[i]->m_pointA = position;
			velocity[i]->m_pointB = cAdd(position, linearVelocity);

			// update position and orientation of cursor
			cursor[i]->setLocalPos(position);
			cursor[i]->setLocalRot(rotation);

			nao_pose = motion.getPosition(chainNameRight, space, true);

			if (button0 &&  i == 0)
			{

				cout << "Haptics 1" << endl;
				cout << "x: " << ball_position[0] << "  y: " << ball_position[1] << " zx:" << ball_position[2] << endl;
				std::string handName = "RHand";
				motion.openHand(handName);

				command_pose[0] = position.x() * 2;
				command_pose[1] = position.y() * 2;
				command_pose[2] = position.z() * 2;
				/*command_pose[0] = ball_position[0];
				command_pose[1] = ball_position[1];
				command_pose[2] = ball_position[2];*/
				command_pose[3] = 0;
				command_pose[4] = 0;
				command_pose[5] = 0;


				//	float one_shot_check = sqrt(pow((command_pose[0] - ball_pose[0]), 2) + pow((command_pose[1] - ball_pose[1]), 2) + pow((command_pose[2] - ball_pose[2]), 2));
				//	float comparison_distance = sqrt(pow((nao_pose_right[0] - ball_pose[0]), 2) + pow((nao_pose_right[1] - ball_pose[1]), 2) + pow((nao_pose_right[2] - ball_pose[2]), 2));

				motion.setPosition(chainNameRight, space, command_pose, fractionMaxSpeed, axisMask);
				cursor[i]->m_material->setGreenMediumAquamarine();
			}
			if (button0 &&  i == 1)
			{

				cout << "Haptics 2" << endl;
				std::string handName = "LHand";
				motion.openHand(handName);

				command_pose[0] = position.x() * 2;
				command_pose[1] = position.y() * 2;
				command_pose[2] = position.z() * 2;
				command_pose[3] = 0;
				command_pose[4] = 0;
				command_pose[5] = 0;


				//	float one_shot_check = sqrt(pow((command_pose[0] - ball_pose[0]), 2) + pow((command_pose[1] - ball_pose[1]), 2) + pow((command_pose[2] - ball_pose[2]), 2));
				//	float comparison_distance = sqrt(pow((nao_pose_right[0] - ball_pose[0]), 2) + pow((nao_pose_right[1] - ball_pose[1]), 2) + pow((nao_pose_right[2] - ball_pose[2]), 2));

				motion.setPosition(chainNameLeft, space, command_pose, fractionMaxSpeed, axisMask);
				cursor[i]->m_material->setGreenMediumAquamarine();
			}
			else if (button1 && i == 0)
			{
				// Open the right hand
				motion.closeHand("RHand");
				/*if (attachball)
				attachball = false;
				else
				attachball = true;*/
				cursor[i]->m_material->setYellowGold();
			}
			else if (button1 && i == 1)
			{
				// Open the left hand
				motion.closeHand("LHand");
				cursor[i]->m_material->setYellowGold();
			}
			else if (button2)
			{
				cursor[i]->m_material->setOrangeCoral();
			}
			else if (button3)
			{

				//attachball = true;
				cursor[i]->m_material->setPurpleLavender();
			}
			else
			{

				cursor[i]->m_material->setBlueRoyal();
			}


			//		simxGetObjectPosition(clientID, h_handle, t_handle, hand_position_vrep, simx_opmode_blocking);
			//	printf("Hand's position is : %f, \t %f, \t %f\n ", hand_position_vrep[0], hand_position_vrep[1], hand_position_vrep[2]);
			//		printf("Naoqi's position is : %f, \t %f, \t %f\n ", nao_pose[0], nao_pose[1], nao_pose[2]);
			//		printf("Difference : %f \t %f \t \%f \n", (hand_position_vrep[0] - nao_pose[0]), (hand_position_vrep[1] - nao_pose[1]), (hand_position_vrep[2] - nao_pose[2]));

			//float current_distance_right = sqrt(pow((nao_pose_right[0] - ball_pose[0]), 2) + pow((nao_pose_right[1] - ball_pose[1]), 2) + pow((nao_pose_right[2] - ball_pose[2]), 2));
			//float current_distance_left = sqrt(pow((nao_pose_left[0] - ball_pose[0]), 2) + pow((nao_pose_left[1] - ball_pose[1]), 2) + pow((nao_pose_left[2] - ball_pose[2]), 2));

			if (i == 0)
				distance_t1[i] = sqrt(pow((nao_pose_right[0] - ball_pose[0]), 2) + pow((nao_pose_right[1] - ball_pose[1]), 2) + pow((nao_pose_right[2] - ball_pose[2]), 2));
			else
				distance_t1[i] = sqrt(pow((nao_pose_left[0] - ball_pose[0]), 2) + pow((nao_pose_left[1] - ball_pose[1]), 2) + pow((nao_pose_left[2] - ball_pose[2]), 2));

			attachball = graspBall(distance_t1, R2, attachball);

			
			bool dir = HysteresisDirection(distance_t0[i], distance_t1[i]);

			if (attachball)
				Kp[i] = fmax;
			else
				Kp[i] = DoHysteresis(distance_t1[i], dir, last_distance_state[i], distance_state[i], sphere_radius, m, b, i);
			distance_t0[i] = distance_t1[i];

			// update global variable for graphic display update
			hapticDevicePosition[i] = position;


			/////////////////////////////////////////////////////////////////////
			// COMPUTE AND APPLY FORCES
			/////////////////////////////////////////////////////////////////////

			// desired position
			cVector3d desiredPosition;
			desiredPosition.set(0.0, 0.0, 0.0);

			// desired orientation
			cMatrix3d desiredRotation;
			desiredRotation.identity();

			// variables for forces    
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;


			// apply force in the direction of the movement
			cVector3d ball_pose_3d(ball_pose[0], ball_pose[1], ball_pose[2]);

			cVector3d forceField = Kp[i] * (ball_pose_3d - position);
			force.add(forceField);

			// compute angular torque
			double Kr = 0.05; // [N/m.rad]
			cVector3d axis;
			double angle;
			cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
			deltaRotation.toAxisAngle(axis, angle);
			torque = rotation * ((Kr * angle) * axis);


			// apply force field
			//if (useForceField)
			//{
			//	// compute linear force
			//	double Kp = 25; // [N/m]
			//	cVector3d forceField = Kp * (desiredPosition - position);
			//	force.add(forceField);

			//	// compute angular torque
			//	double Kr = 0.05; // [N/m.rad]
			//	cVector3d axis;
			//	double angle;
			//	cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
			//	deltaRotation.toAxisAngle(axis, angle);
			//	torque = rotation * ((Kr * angle) * axis);
			//}

			//// apply damping term
			//if (useDamping)
			//{
			//	cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

			//	// compute linear damping force
			//	double Kv = 1.0 * info.m_maxLinearDamping;
			//	cVector3d forceDamping = -Kv * linearVelocity;
			//	force.add(forceDamping);

			//	// compute angular damping force
			//	double Kvr = 1.0 * info.m_maxAngularDamping;
			//	cVector3d torqueDamping = -Kvr * angularVelocity;
			//	torque.add(torqueDamping);

			//	// compute gripper angular damping force
			//	double Kvg = 1.0 * info.m_maxGripperAngularDamping;
			//	gripperForce = gripperForce - Kvg * gripperAngularVelocity;
			//}

			// send computed force, torque, and gripper force to haptic device
			hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

			if (attachball) {
				//cout << "attached" << endl;
				ball_position[0] = 0.05;
				ball_position[1] = 0.0;
				ball_position[2] = 0.00;
				simxSetObjectPosition(clientID, b_handle, h_handle, ball_position, simx_opmode_blocking);
				//	simxSetObjectOrientation(clientID, b_handle,h_handle, ball_orientation, simx_opmode_blocking);
			}
			else {
				//simxSetObjectPosition(clientID, b_handle, -1, original_ball_position, simx_opmode_blocking);
				//cout << " not attached" << endl;
			}
		}

		// update frequency counter
		frequencyCounter.signal(1);

		// Save the data for both haptics
		//
		// && 
		if (SaveFileHaptics.is_open() && count_samples > 10){
			count_samples = 0;
			SaveFileHaptics << std::to_string(static_cast<long double>(distance_t1[0])) << " , " << std::to_string(static_cast<long double>(Kp[0])) << " , " << distance_state[0] << " , "<< std::to_string(static_cast<long double>(distance_t1[1])) << " , " << std::to_string(static_cast<long double>(Kp[1])) << " , "<< distance_state[1] << " , "<< attachball <<"\n";
			//std::cout << "Saving" << std::endl;
		}
		count_samples++;
	}

	// Close the data file
	SaveFileHaptics.close();
	std::cout << "Closing the text file" << std::endl;

	simulationFinished = true;
}

// ------------ Our functions --------------

bool HysteresisDirection(float distance_t0, float distance_t1) {
	bool goInside;
	double force_magn;
	float delta = distance_t1 - distance_t0;
	if (delta < 0) {
		//Hand is going "inside"->check
		//std::cout << "Inside:  " << delta << "  to:  " << distance_t0<<"  t1:  "<< distance_t1<< std::endl;
		goInside = true;
	}
	if (delta > 0) {
		//std::cout << "Outside: " << delta << "  to:  " << distance_t0 << "  t1:  " << distance_t1 << std::endl;
		goInside = false;
	}
	////Check interval variables
	//if (goInside) {
	//	force_magn=25;
	//}
	//else {
	//	force_magn=10;
	//}
	return goInside;
}



double ComputeForce(double m, double b, double dist) {

	double f;
	f = m*dist + b;

	if (f >= fmax) {
		cout << "f> fmax : " << f << endl;
		f = fmax;

	}

	if (f <= 0)
		f = 0.0;
	return f;
}

double DoHysteresis(double distance, bool goInside, std::string &last_state, std::string &state, std::vector<double> radius, double m, std::vector<double> b, int index) {

	//Convention
	//left = 0
	//center = 1
	//rigth = 2
	double f = 12.17;
	std::string aux_state = state;
	double R1 = radius[0];
	double R1p = radius[1];
	double R2 = radius[2];
	double R2p = radius[3];

	if (state == "outside") {

		if (distance > R1p) {
			f = 0.0;
			state = "outside";
		}
		else {
			if (distance <= R1p && distance >= R1) {
				state = "ramp_outside";
				f = 0.0;
			}
			//else
			//	state = "inside";
		}
	}
	else if (state == "ramp_outside") {
		if (distance <= R1p && distance >= R1) {
			state = "ramp_outside";
			if (last_state == "center") {
				f = ComputeForce(m, b[0], distance);
			}
			else if (last_state == "outside")
				f = 0.0;
		}
		else {
			f = 0.0;
			if (distance < R1)
				state = "center";
			else
				state = "outside";
		}

	}
	else if (state == "center") {
		if (distance < R1 && distance > R2) {
			state = "center";
			if (last_state == "ramp_outside")
				f = 0.0;
			else if (last_state == "ramp_inside")
				f = fmax;
		}
		else {
			if (distance >= R1) {
				state = "ramp_outside";
				if (last_state == "ramp_inside")
					f = ComputeForce(m, b[0], distance);
				else
					f = 0.0;
			}
			else {
				state = "ramp_inside";
				if (last_state == "ramp_outside")
					f = ComputeForce(m, b[1], distance);
				else
					f = fmax;
			}
		}
	}
	else if (state == "ramp_inside") {
		if (distance <= R2 && distance >= R2p) {
			state = "ramp_inside";
			if (last_state == "center")
				f = ComputeForce(m, b[1], distance);
			else if (last_state == "inside")
				f = fmax;
		}
		else {

			f = fmax;
			if (distance < R2p) {
				state = "inside";
			}
			else {
				state = "center";
				//if (last_state == "center")
				//	f = 0.0;
				//else
				//	f = fmax;
			}
		}
	}
	// State "inside"
	else {

		if (distance < R2p) {
			state = "inside";
			f = fmax;
		}
		else {
			if (distance >= R2p  && distance <= R2)
				state = "ramp_inside";
			f = fmax;
		}
	}
	//cout << "last state:" << last_state << endl;

	if (aux_state != state) {
		last_state = aux_state;
		cout << "Haptics: " << index << "  Force " << f << "  distance  " << distance << "  State: " << state << endl;
	}
	return f;
}

void ComputeSlopesParameters(double R1p, double R2, double delta_x, double &m, std::vector<double> &b) {

	//The slope
	m = -fmax / delta_x;
	std::cout << "The slope of the system is:  " << m << std::endl;
	// Interesection with y axes
	b[0] = -m*R1p;//JUAN-m*R1;
	b[1] = -m*R2;//JUAN fmax - m*R2;

				 //% Creates funtcion 1 (y1) and function 2 (y2)
				 //x = 0:0.01 : 2 * R2;
				 //y1 = m*x + b1;
				 //y2 = m*x + b2;
				 //
				 //xlimit = R2 - fmax / m;

	return;
}
bool graspBall(std::vector<float> distance, double R, bool attached) {

	bool prev_attached = attached;
	if (distance[0] < R &&  distance[1] < R)
	//if (distance[0] <= R &&  distance[1] <= R)
		attached = true;
	else
		attached = false;

	if (prev_attached != attached)
		std::cout << "Attached:" << attached << std::endl;

	return attached;
}