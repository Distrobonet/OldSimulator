//------------------------------------------------------------------
// Description:     This program tests the robot cell simulator.
//------------------------------------------------------------------

// preprocessor directives
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

// Used for non-blocking user input
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <Simulator/Environment.h>
#include <Simulator/Robot.h>

#include"Simulator/FormationIndex.h"

#define SUBSCRIBER 0
#define PUBLISHER 1

using namespace std;
class Robot;
class OverLord;


// define SIGPIPE if not defined (compatibility for win32)
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

// simulation environment function prototypes
void terminate(int retVal);
void displayMenu();
void keyboardInput();
void clearScreen();
bool initEnv(const int nRobots, const int formationIndex);
bool deinitEnv();
bool changeFormation(const int index, const Vector gradient = Vector());
bool changeFormationSim(const int index, const Vector gradient = Vector());
const char  CHAR_ESCAPE             = char(27);    // 'ESCAPE' character key


// OpenGL function prototypes
void display();
void keyboardPress(unsigned char keyPressed, int mouseX, int mouseY);
//void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY);
//void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY);

// formation-testing function prototypes
float  line(const float x);
float  x(const float x);
float  absX(const float x);
float  negHalfX(const float x);
float  negAbsHalfX(const float x);
float  negAbsX(const float x);
float  parabola(const float x);
float  cubic(const float x);
float  condSqrt(const float x);
float  sine(const float x);
float  xRoot3(const float x);
float  negXRoot3(const float x);
Function formations[] = {line,        x,       absX,     negHalfX,
						negAbsHalfX, negAbsX, parabola, cubic,
						condSqrt,    sine,    xRoot3,   negXRoot3};


// Menu Global variable
int CURRENT_SELECTION = -1;

// simulation global constants
const float   SELECT_RADIUS     = 1.5f * DEFAULT_ROBOT_RADIUS;
const int     N_CELLS           = 0;
const int     MIDDLE_CELL       = 0;//(N_CELLS - 1) / 2;

// A formation is a vector of Functions, which are functions that take floats and return floats
const Formation DEFAULT_FORMATION = Formation(formations[0], DEFAULT_ROBOT_RADIUS * FACTOR_COLLISION_RADIUS, Vector(), MIDDLE_CELL, 0,  90.0f);


// simulation global variables
Environment *g_environment       = NULL;
int        g_nRobots             = 7;
float      g_formationRadius     = DEFAULT_FORMATION.getRadius();
int        g_seedID              = DEFAULT_FORMATION.getSeedID();
int        g_formationID         = DEFAULT_FORMATION.getFormationID();
float      g_formationHeading    = DEFAULT_FORMATION.getHeading();
int        g_formationIndex      = 0;
int        g_selectedIndex       = g_seedID;
int        g_dTime               = 50;    // time interval (in milliseconds)
bool       g_prop_toggle         = false;

// Global variables
double distanceToTarget = 0.0l;
double angleChange = 0.0l;

double distanceToTarget1 = 0.0l;
double angleChange1 = 0.0l;

float xValue = 0.0l;
float yValue = 0.0l;

double getYValue(double xValue);
bool initEnv(const int nRobots, const int formationIndex);



// Service utility function to set the formationIndex being served to the CURRENT_SELECTION
bool setFormation(Simulator::FormationIndex::Request  &req,
		Simulator::FormationIndex::Response &res )
{
  //res.sum = req.a + req.b;
	res.formationIndex = CURRENT_SELECTION;
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: [%ld]", (long int)res.formationIndex);
	return true;
}






int main(int argc, char **argv)
{
	// Service
	ros::init(argc, argv, "formation_index_server");
	ros::NodeHandle serverNode;
	ros::ServiceServer service = serverNode.advertiseService("formation_index", setFormation);
	ROS_INFO("Now serving the formation index.");
	//ros::spin();
	ros::spinOnce();

	// Service Client - blank request, gets formationIndex response
	ros::init(argc, argv, "formation_index_client");
	ros::NodeHandle clientNode;
	ros::ServiceClient client = clientNode.serviceClient<Simulator::FormationIndex>("formation_index");
	Simulator::FormationIndex srv;
//	srv.request.a = atoll(argv[1]);
//	srv.request.b = atoll(argv[2]);
	ROS_INFO("Trying to access the formationIndex");

	// Uses an asynchronous spinner to account for the blocking service client call
	ros::AsyncSpinner spinner(1);
	spinner.start();

	if (client.call(srv))
		ROS_INFO("formation index: %ld", (long int)srv.response.formationIndex);
	else
		ROS_ERROR("Failed to call service formation_index");

	spinner.stop();

	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle aNode;
	ros::Rate loop_rate(10);

	displayMenu();

	// Only continue program once a selection has been made
	while(CURRENT_SELECTION == -1)
	{
		keyboardInput();
	}

	// initialize and execute the robot cell environment
	if (!initEnv(g_nRobots, CURRENT_SELECTION))
	{
		cerr << ">> ERROR: Unable to initialize simulation environment...\n\n";
		return 1;
	}

	// Primary ROS loop
	while(ros::ok())
	{
		keyboardInput();

		std_msgs::String msg;
		std::stringstream ss;
		ss << CURRENT_SELECTION;
		msg.data = ss.str();

		ros::Publisher formation_pub = aNode.advertise<std_msgs::String>("formation", 1000);
		formation_pub.publish(msg);

		for(int robotNum = 1; robotNum < g_nRobots; robotNum++)
		{
			Robot *robot1 = g_environment->getRobot(robotNum);
			Robot *robot2 = g_environment->getRobot(robotNum + 1);

			// A robot
			xValue = robot1->robotX +1;
			yValue = getYValue(robot1->robotY +1);

		}

		//g_environment->getRobot(1)->commandVelocity.linear.x = 5;
		//cout << "distance from 0,0 to robot(1): " << g_environment->distanceToRobot(0, 0, g_environment->getRobot(1)) << endl;

		// update the robot cell environment
		g_environment-> step();
		ros::spinOnce();
		loop_rate.sleep();
	}

  deinitEnv();

  return 0;
}


// Used by keyboardInput() to catch keystrokes without blocking
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}


// Catches keyboard input and sets CURRENT_SELECTION based on user input, redisplays the menu
void keyboardInput()
{
	char keyPressed;

	if(kbhit())
	{
		keyPressed=getchar();

		int keyNum = atoi(&keyPressed);
		cout << "\nKey pressed: " << keyPressed;

		if(keyNum >= 0 && keyNum <= 9)
		{
			cout << " - Setting to " << keyPressed <<endl;
			CURRENT_SELECTION = keyNum;
		}
		else
			cout << " - Not a valid input.";

		displayMenu();
	}

}

// A simple and basic way to clear the screen for the menu refresh
void clearScreen()
{
	std::cout << "\n\n\n\n\n";
}


// Displays the selection menu to the screen
void displayMenu()
{
	clearScreen();

	cout << endl << endl << "Use the '0-9' keys to "
		<< "change to a formation seeded at the selected robot."
		<< endl << endl
		<< "PRESET FORMATIONS\n-----------------"            << endl
		<< "0) f(x) = 0"                                     << endl
		<< "1) f(x) = x"                                     << endl
		<< "2) f(x) = |x|"                                   << endl
		<< "3) f(x) = -0.5 x"                                << endl
		<< "4) f(x) = -|0.5 x|"                              << endl
		<< "5) f(x) = -|x|"                                  << endl
		<< "6) f(x) = x^2"                                   << endl
		<< "7) f(x) = x^3"                                   << endl
		<< "8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0}"  << endl
		<< "9) f(x) = 0.05 sin(10 x)"                        << endl << endl
		<< "Use the mouse to select a robot."                << endl
		<< "Use ctrl+C to exit."                                << endl << endl
		<< "Please enter your selection: ";
}


// Terminates the program on interrupt (i.e., ^C).
void terminate(int retVal)
{
  signal(SIGINT, SIG_IGN);
  deinitEnv();
  signal(SIGINT, SIG_DFL);
  exit(retVal);
}


// Attempts to initialize the environment based on
// the parameterized values, returning true if successful,
// false otherwise.
//
// Parameters:
//      nRobots       in      the number of robots
//      formationIndex        in      the index of the initial formation
bool initEnv(const int nRobots, const int formationIndex)
{
  if (g_environment != NULL)
  {
    delete g_environment;
    g_environment = NULL;
  }

  Formation f(formations[formationIndex], g_formationRadius, Vector(),
		  	  	  g_seedID, ++g_formationID, g_formationHeading);

  if ((g_environment = new Environment(nRobots, f)) == NULL)
  {
	  return false;
  }
  return true;
}


// Attempts to deinitialize the environment,
// returning true if successful, false otherwise.
bool deinitEnv()
{
  if (g_environment != NULL)
  {
    delete g_environment;
    g_environment = NULL;
  }
  return g_environment == NULL;
}


// Attempts to change the current formation,
// returning true if successful, false otherwise.
//
// Parameters:
//      index   in      the index of the formation to change to
bool changeFormation(const int index, const Vector gradient)
{
  g_formationIndex = index;
  if (!g_environment->startFormation)
  {
    g_environment->startFormation = true;
  }
  // determine if a new seed has been selected
  if (g_selectedIndex != -1)
  {
    g_environment->getCell(g_seedID)->setColor(DEFAULT_CELL_COLOR);
    g_seedID = g_selectedIndex;
  }
  Formation f(formations[index], g_formationRadius,     gradient,
      g_seedID,           ++g_environment->formationID, g_formationHeading);

  return g_environment->changeFormation(f);
}


// called by environment, passes the location of a new calculated cell index.
// parameters:   index in the index of the formation to change to
bool changeFormationSim(const int index, const Vector gradient)
{
  if(g_selectedIndex > -1)
  {
    g_environment->getCell(g_seedID)->setColor(DEFAULT_CELL_COLOR);
    g_selectedIndex = index;
    return changeFormation(g_formationIndex,gradient);
  }
  else return false;
}


bool sendNCellRequest()
{
  PropMsg *ncell = new PropMsg();
  return g_environment->sendMsg(ncell, g_seedID,ID_OPERATOR, NCELL_REQUEST);
}


bool sendFcntrRequest()
{
  PropMsg *fcntr = new PropMsg();
  return g_environment->sendMsg(fcntr, g_seedID,ID_OPERATOR, FCNTR_REQUEST);
}


bool sendFRadRequest()
{
  PropMsg *frad = new PropMsg();
  return g_environment->sendMsg(frad, g_seedID,ID_OPERATOR, FRAD_REQUEST);
}


bool sendFSeedRequest()
{
  PropMsg *fseed = new PropMsg();
  return g_environment->sendMsg(fseed, g_seedID,ID_OPERATOR, FSEED_REQUEST);
}


// Clears the frame buffer and draws the simulated cells within the window.
void display()
{
  // draws environment robot cells
  if (g_environment->getCells().size() > 0)
  {
    g_environment->getCell(g_seedID)->setColor(GREEN);
    for(int i = 0; i < g_environment->getNumberOfCells(); ++i)
    {
      if(g_environment->getCell(i) != g_environment->getCell(g_seedID))
      {
        if(g_environment->getCell(i) == g_environment->getCell(g_selectedIndex))
          g_environment->getCell(i)->setColor(RED);
        else
          g_environment->getCell(i)->setColor(DEFAULT_CELL_COLOR);
      }
    }
  }
  //g_environment->draw();
}


// Handles the keyboard input (ASCII Characters).
void keyboardPress(unsigned char keyPressed, int mouseX, int mouseY)
{
  if ((keyPressed >= '0') && (keyPressed <= '9'))
  {
    if (g_environment->getNumberOfCells() > 0)
    {
      char cIndex = keyPressed;
      changeFormation(atoi(&cIndex));
    }
  }
  else switch (keyPressed)
  {

    // change formation heading
    case '<': case ',':
      if (g_environment->getNumberOfCells() > 0)
      {
        g_formationHeading += g_environment->getCell(g_seedID)->maxAngSpeed();
        changeFormation(g_formationIndex);
        g_environment->getCell(g_seedID)->rotateRelative(
            g_environment->getCell(g_seedID)->maxAngSpeed());
        //min(1.0f, g_environment->getCell(g_sID)->maxAngSpeed()));
      }
      break;

    case '>': case '.':
      if (g_environment->getNumberOfCells() > 0)
      {
        g_formationHeading -= g_environment->getCell(g_seedID)->maxAngSpeed();
        changeFormation(g_formationIndex);
        g_environment->getCell(g_seedID)->rotateRelative(
            -g_environment->getCell(g_seedID)->maxAngSpeed());
        //-min(1.0f, g_environment->getCell(g_sID)->maxAngSpeed()));
      }
      break;

      // change formation scale
    case '+': case '=':
      if (g_environment->getNumberOfCells() > 0)
      {
        g_formationRadius += 0.01f;
        changeFormation(g_formationIndex);
      }
      break;

    case '-': case '_':
      if (g_environment->getNumberOfCells() > 0)
      {
        g_formationRadius -= 0.01f;
        g_formationRadius  = max(g_formationRadius,
            g_environment->getCell(g_seedID)->collisionRadius());
        changeFormation(g_formationIndex);
      }
      break;

    case 'h': case 'H':
      if (g_environment->getNumberOfCells() > 0)
        g_environment->showHeading(!g_environment->getCell(g_seedID)->showHeading);
      break;

    case 'l': case 'L':
      if (g_environment->getNumberOfCells() > 0)
        g_environment->showLine(!g_environment->getCell(g_seedID)->heading.showLine);
      break;

    case 'p': case 'P':
      if (g_environment->getNumberOfCells() > 0)
        g_environment->showPos(!g_environment->getCell(g_seedID)->showPos);

      break;

    case 't': case 'T':
      if (g_environment->getNumberOfCells() > 0)
        g_environment->showHead(!g_environment->getCell(g_seedID)->heading.showHead);
      break;

    case 'n': case 'N':
      if (g_environment->getNumberOfCells() > 0)
        //g_prop_toggle = !g_prop_toggle;
        sendNCellRequest();
      break;

    case 'c': case 'C':
      if (g_environment->getNumberOfCells() > 0)
        sendFcntrRequest();
      break;

    case 'r': case 'R':
      if (g_environment->getNumberOfCells() > 0)
        sendFRadRequest();
      break;

    case 's': case 'S':
      if (g_environment->getNumberOfCells()> 0)
        sendFSeedRequest();
      break;

    case CHAR_ESCAPE: 
    	deinitEnv(); 
    	exit(0);
  }
}


// Handles the keyboard input (non-ASCII Characters).
//void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY)
//{
//  switch (keyPressed)
//  {
//    case GLUT_KEY_LEFT:
//      if (g_environment->getNumberOfCells() > 0)
//      {
//        g_environment->getCell(g_seedID)->rotError =
//          -(1.0001f * g_environment->getCell(g_seedID)->angThreshold());
//        //g_environment->getCell(g_seedID)->rotateRelative(
//        //    min(1.0f, g_environment->getCell(g_sID)->maxAngSpeed()));
//      }
//      break;
//
//    case GLUT_KEY_UP:
//      if (g_environment->getNumberOfCells() > 0)
//      {
//        g_environment->getCell(g_seedID)->transError.x =
//          1.0001f * g_environment->getCell(g_seedID)->threshold();
//        //g_environment->getCell(g_seedID)->translateRelative(
//        //    min(0.001f, g_environment->getCell(g_sID)->maxSpeed()));
//      }
//      break;
//
//    case GLUT_KEY_RIGHT:
//      if (g_environment->getNumberOfCells() > 0)
//      {
//        g_environment->getCell(g_seedID)->rotError =
//          1.0001f * g_environment->getCell(g_seedID)->angThreshold();
//        //g_environment->getCell(g_seedID)->rotateRelative(
//        //    -min(1.0f, g_environment->getCell(g_sID)->maxAngSpeed()));
//      }
//      break;
//
//    case GLUT_KEY_DOWN:
//      if (g_environment->getNumberOfCells() > 0)
//      {
//        g_environment->getCell(g_seedID)->transError.x =
//          -(1.0001f * g_environment->getCell(g_seedID)->threshold());
//        //g_environment->getCell(g_seedID)->translateRelative(
//        //    -min(0.001f, g_environment->getCell(g_sID)->maxSpeed()));
//      }
//      break;
//
//    default: break;
//  }
//}


// Handles the keyboard input (non-ASCII Characters).
//void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY)
//{
//  switch (keyReleased)
//  {
//    case GLUT_KEY_LEFT: case GLUT_KEY_RIGHT:
//      if (g_environment->getNumberOfCells() > 0)
//        g_environment->getCell(g_seedID)->rotError = 0.0f;
//      break;
//
//    case GLUT_KEY_UP: case GLUT_KEY_DOWN:
//      if (g_environment->getNumberOfCells() > 0)
//        g_environment->getCell(g_seedID)->transError.x = 0.0f;
//      break;
//
//    default:
//    	break;
//  }
//}


// This function does all the actual computation depending on which function the user has selected
double getYValue(double xValue)
{
	double yValue = -9999.0l;
	switch(CURRENT_SELECTION)
	{
		case 0:
			yValue = 0.0l;
		break;
		case 1:
			yValue = xValue;
		break;
		case 2:
			yValue = abs(xValue);
		break;
		case 3:
			yValue = -0.5f * xValue;
		break;
		case 4:
			yValue = -abs(0.5f * xValue);
		break;
		case 5:
			yValue = -abs(xValue);
		break;
		case 6:
			yValue = xValue*xValue;
		break;
		case 7:
			yValue = xValue*xValue*xValue;
		break;
		case 8:
			yValue = sqrt(abs(0.5f * xValue)) * ((xValue >= 0) ? 1.0f : -1.0f);
		break;
		case 9:
			yValue = 0.05f * sin(10.0f * xValue);;
		break;
	}
	return yValue;
}


// <test formation functions>

// Returns formation function definition f(x) = 0.
float line(const float x)
{
  return 0.0f;
}


// Returns formation function definition f(x) = x.
float x(const float x)
{
  return x;
}


// Returns formation function definition f(x) = |x|.
float absX(const float x)
{
  return abs(x);
}


// Returns formation function definition f(x) = -0.5 x.
float negHalfX(const float x)
{
  return -0.5f * x;
}


// Returns formation function definition f(x) = -|0.5 x|.
float negAbsHalfX(const float x)
{
  return -abs(0.5f * x);
}


// Returns formation function definition f(x) = -|x|.
float negAbsX(const float x)
{
  return -abs(x);
}


// Returns formation function definition f(x) = x^2.
float parabola(const float x)
{
  return x * x;
  //return pow(x, 2.0f);
}


// Returns formation function definition f(x) = x^3.
float cubic(const float x)
{
  return x * x * x;
  //return pow(x, 3.0f);
}


// Returns formation function definition
// f(x) = {sqrt(x),  x = 0 | -sqrt|x|, x < 0}.
float condSqrt(const float x)
{
  return sqrt(abs(0.5f * x)) * ((x >= 0) ? 1.0f : -1.0f);
}


// Returns formation function definition f(x) = 0.05 sin(10 x).
float sine(const float x)
{
  return 0.2f * sin(10.0f * x);
}


// Returns formation function definition f(x) = x sqrt(3).
float xRoot3(const float x)
{
  return x * sqrt(3.0f);
}


// Returns formation function definition f(x) = -x sqrt(3).
float negXRoot3(const float x)
{
  return -x * sqrt(3.0f);
}
