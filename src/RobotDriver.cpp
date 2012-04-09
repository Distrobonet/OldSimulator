//------------------------------------------------------------------
// Description:     This program tests the robot cell simulator.
//------------------------------------------------------------------

// preprocessor directives
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

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

#include <Simulator/RobotDriver.h>
#include <Simulator/Environment.h>
#include <Simulator/Robot.h>

// HUH?
#define SUBSCRIBER 0
#define PUBLISHER 1

using namespace std;
class Robot;


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
void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY);
void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY);
void timerFunction(int value);


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
int currentSelection;

// simulation global constants
const float   SELECT_RADIUS     = 1.5f * DEFAULT_ROBOT_RADIUS;
const int     N_CELLS           = 0;
const int     MIDDLE_CELL       = 0;//(N_CELLS - 1) / 2;

// A formation is a vector of Functions, which are functions that take floats and return floats
const Formation DEFAULT_FORMATION = Formation(formations[0], DEFAULT_ROBOT_RADIUS * FACTOR_COLLISION_RADIUS, Vector(), MIDDLE_CELL, 0,  90.0f);


// simulation global variables
Environment *g_env               = NULL;
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

vector<Robot> robots;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle aNode;
	ros::Rate loop_rate(10);

	for(int robotNum = 0; robotNum < g_nRobots; robotNum++)
	{
		Robot *temp = new Robot(0, 0, 0, 0);
		robots.push_back(*temp);
	}


	// Primary ROS loop
	while(ros::ok())
	{
		displayMenu();
		keyboardInput();

		for(int robotNum = 0; robotNum < g_nRobots; robotNum++)
		{
			// A robot
			xValue = velocityX0 + 1;
			yValue = getYValue(velocityY0 + 1);
			if(yValue != -9999)
			{
				distanceToTarget = getDistance(xValue, yValue, velocityX1, velocityY1);
				angleChange = getAngle(xValue, yValue, velocityX1, velocityY1, theta1);

				if (distanceToTarget < 0.00001)
				{
					distanceToTarget = 0;
					angleChange = getAngle(xValue, velocityY0 + 2, velocityX1, velocityY1, theta1);
				}
				robots.at(robotNum).commandVelocity.linear.x = 0;
				robots.at(robotNum).commandVelocity.linear.y = 0;
				robots.at(robotNum).commandVelocity.angular.z = angleChange;
				robots.at(robotNum).pub_cmd_vel.publish(robots.at(robotNum).commandVelocity);

				if((angleChange < 0.1 && angleChange > 0) || (angleChange > -0.1 && angleChange < 0))
				{
					robots.at(robotNum).commandVelocity.linear.x = distanceToTarget;
					robots.at(robotNum).commandVelocity.linear.y = distanceToTarget;
					robots.at(robotNum).commandVelocity.angular.z = 0;
					robots.at(robotNum).pub_cmd_vel.publish(robots.at(robotNum).commandVelocity);
				}
			}
		}
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


// Catches keyboard input and sets currentSelection based on user input, redisplays the menu
void keyboardInput()
{
	char keyPressed;

	if(kbhit())
	{
		keyPressed=getchar();

		cout << "\nKey pressed: " << keyPressed;

		if(keyPressed >= '0' && keyPressed <= '9')
		{
			cout << " - Setting to " << keyPressed;
			currentSelection = keyPressed;
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
  if (g_env != NULL)
  {
    delete g_env;
    g_env = NULL;
  }

  Formation f(formations[formationIndex], g_formationRadius, Vector(),
      g_seedID,            ++g_formationID,     g_formationHeading);
  if ((g_env = new Environment(nRobots, f)) == NULL) return false;
  return true;
}


// Attempts to deinitialize the environment,
// returning true if successful, false otherwise.
bool deinitEnv()
{
  if (g_env != NULL)
  {
    delete g_env;
    g_env = NULL;
  }
  return g_env == NULL;
}


// Attempts to change the current formation,
// returning true if successful, false otherwise.
//
// Parameters:
//      index   in      the index of the formation to change to
//
bool changeFormation(const int index, const Vector gradient)
{
  g_formationIndex = index;
  if (!g_env->startFormation)
  {
    g_env->startFormation = true;
  }
  // determine if a new seed has been selected
  if (g_selectedIndex != -1)
  {
    g_env->getCell(g_seedID)->setColor(DEFAULT_CELL_COLOR);
    g_seedID = g_selectedIndex;
  }
  Formation f(formations[index], g_formationRadius,     gradient,
      g_seedID,           ++g_env->formationID, g_formationHeading);

  return g_env->changeFormation(f);
}


// called by environment, passes the location of a new calculated cell
// index.
//
// parameters:
//         index in the index of the formation to change to
bool changeFormationSim(const int index, const Vector gradient)
{
  if(g_selectedIndex > -1)
  {
    g_env->getCell(g_seedID)->setColor(DEFAULT_CELL_COLOR);
    g_selectedIndex = index;
    return changeFormation(g_formationIndex,gradient);
  }
  else return false;
}


bool sendNCellRequest()
{
  PropMsg *ncell = new PropMsg();
  return g_env->sendMsg(ncell, g_seedID,ID_OPERATOR, NCELL_REQUEST);
}


bool sendFcntrRequest()
{
  PropMsg *fcntr = new PropMsg();
  return g_env->sendMsg(fcntr, g_seedID,ID_OPERATOR, FCNTR_REQUEST);
}


bool sendFRadRequest()
{
  PropMsg *frad = new PropMsg();
  return g_env->sendMsg(frad, g_seedID,ID_OPERATOR, FRAD_REQUEST);
}


bool sendFSeedRequest()
{
  PropMsg *fseed = new PropMsg();
  return g_env->sendMsg(fseed, g_seedID,ID_OPERATOR, FSEED_REQUEST);
}


// Clears the frame buffer and draws the simulated cells within the window.
void display()
{
//  glClear(GL_COLOR_BUFFER_BIT);   // clear background color
//  glMatrixMode(GL_MODELVIEW);     // modeling transformation

  // draws environment robot cells
  if (g_env->getCells().size() > 0)
  {
    g_env->getCell(g_seedID)->setColor(GREEN);
    for(int i = 0; i < g_env->getNCells(); ++i)
    {
      if(g_env->getCell(i) != g_env->getCell(g_seedID))
      {
        if(g_env->getCell(i) == g_env->getCell(g_selectedIndex))
          g_env->getCell(i)->setColor(RED);
        else
          g_env->getCell(i)->setColor(DEFAULT_CELL_COLOR);
      }
    }
  }
  g_env->draw();

//  glFlush();                      // force the execution of OpenGL commands
//  glutSwapBuffers();              // swap visible buffer and writing buffer
}


// Handles the keyboard input (ASCII Characters).
//
// Parameters:
//      keyPressed  in      the ASCII key that was pressed
//      mouseX      in      the x-coordinate of the mouse position
//      mouseY      in      the y-coordinate of the mouse position
void keyboardPress(unsigned char keyPressed, int mouseX, int mouseY)
{
  if ((keyPressed >= '0') && (keyPressed <= '9'))
  {
    if (g_env->getNCells() > 0)
    {
      char cIndex = keyPressed;
      changeFormation(atoi(&cIndex));
    }
  }
  else switch (keyPressed)
  {

    // change formation heading
    case '<': case ',':
      if (g_env->getNCells() > 0)
      {
        g_formationHeading += g_env->getCell(g_seedID)->maxAngSpeed();
        changeFormation(g_formationIndex);
        g_env->getCell(g_seedID)->rotateRelative(
            g_env->getCell(g_seedID)->maxAngSpeed());
        //min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;

    case '>': case '.':
      if (g_env->getNCells() > 0)
      {
        g_formationHeading -= g_env->getCell(g_seedID)->maxAngSpeed();
        changeFormation(g_formationIndex);
        g_env->getCell(g_seedID)->rotateRelative(
            -g_env->getCell(g_seedID)->maxAngSpeed());
        //-min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;

      // change formation scale
    case '+': case '=':
      if (g_env->getNCells() > 0)
      {
        g_formationRadius += 0.01f;
        changeFormation(g_formationIndex);
      }
      break;

    case '-': case '_':
      if (g_env->getNCells() > 0)
      {
        g_formationRadius -= 0.01f;
        g_formationRadius  = max(g_formationRadius,
            g_env->getCell(g_seedID)->collisionRadius());
        changeFormation(g_formationIndex);
      }
      break;

    case 'h': case 'H':
      if (g_env->getNCells() > 0)
        g_env->showHeading(!g_env->getCell(g_seedID)->showHeading);
      break;

    case 'l': case 'L':
      if (g_env->getNCells() > 0)
        g_env->showLine(!g_env->getCell(g_seedID)->heading.showLine);
      break;

    case 'p': case 'P':
      if (g_env->getNCells() > 0)
        g_env->showPos(!g_env->getCell(g_seedID)->showPos);

      break;

    case 't': case 'T':
      if (g_env->getNCells() > 0)
        g_env->showHead(!g_env->getCell(g_seedID)->heading.showHead);
      break;

    case 'n': case 'N':
      if (g_env->getNCells() > 0)
        //g_prop_toggle = !g_prop_toggle;
        sendNCellRequest();
      break;

    case 'c': case 'C':
      if (g_env->getNCells() > 0)
        sendFcntrRequest();
      break;

    case 'r': case 'R':
      if (g_env->getNCells() > 0)
        sendFRadRequest();
      break;

    case 's': case 'S':
      if (g_env->getNCells()> 0)
        sendFSeedRequest();
      break;

    case 'a': case 'A':
      g_env->addObject(randSign() * frand(),
                       randSign() * frand(),
                       0.0f);
/*
      g_env->addRobot(randSign() * frand(),
                      randSign() * frand(),
                      0.0f,
                      randSign() * frand(0.0f, 180.0f)); */
      break;

    case 'd': case 'D':
      g_env->removeObject();
/*
      if ((g_selectedIndex >= 0) && (g_selectedIndex < g_env->getNCells()))
      {
        g_env->removeCell(g_env->getCell(g_selectedIndex));
		g_selectedIndex = g_sID;
        //g_env->removeRobot();
      } */
      break;

    case CHAR_ESCAPE: 
    	deinitEnv(); 
    	exit(0);
  }
}


// Handles the keyboard input (non-ASCII Characters).
//
// Parameters:
//      keyPressed  in      the non-ASCII key that was pressed
//      mouseX      in      the x-coordinate of the mouse position
//      mouseY      in      the y-coordinate of the mouse position
void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY)
{
  switch (keyPressed)
  {
    case GLUT_KEY_LEFT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->rotError =
          -(1.0001f * g_env->getCell(g_seedID)->angThreshold());
        //g_env->getCell(g_seedID)->rotateRelative(
        //    min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;

    case GLUT_KEY_UP:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->transError.x =
          1.0001f * g_env->getCell(g_seedID)->threshold();
        //g_env->getCell(g_seedID)->translateRelative(
        //    min(0.001f, g_env->getCell(g_sID)->maxSpeed()));
      }
      break;

    case GLUT_KEY_RIGHT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->rotError =
          1.0001f * g_env->getCell(g_seedID)->angThreshold();
        //g_env->getCell(g_seedID)->rotateRelative(
        //    -min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;

    case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->transError.x =
          -(1.0001f * g_env->getCell(g_seedID)->threshold());
        //g_env->getCell(g_seedID)->translateRelative(
        //    -min(0.001f, g_env->getCell(g_sID)->maxSpeed()));
      }
      break;

    default: break;
  }
}


// Handles the keyboard input (non-ASCII Characters).
//
// Parameters:
//      keyReleased  in      the non-ASCII key that was released
//      mouseX       in      the x-coordinate of the mouse position
//      mouseY       in      the y-coordinate of the mouse position
void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY)
{
  switch (keyReleased)
  {
    case GLUT_KEY_LEFT: case GLUT_KEY_RIGHT:
      if (g_env->getNCells() > 0)
        g_env->getCell(g_seedID)->rotError = 0.0f;
      break;

    case GLUT_KEY_UP: case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
        g_env->getCell(g_seedID)->transError.x = 0.0f;
      break;

    default:
    	break;
  }
}


// Updates the environment and redraws.
//
// Parameters:
//      value   in      the value of the timer
void timerFunction(int value)
{
  // sendFcntrRequest();
  // sendFRadRequest();
  // if(g_prop_toggle)
  // 	sendNCellRequest();

  g_env->step();          // update the robot cell environment
}


// This function does all the actual computation depending on which function the user has selected
double getYValue(double xValue)
{
	double yValue = -9999.0l;
	switch(currentSelection)
	{
		case '0':
			yValue = 0.0l;
		break;
		case '1':
			yValue = xValue;
		break;
		case '2':
			yValue = abs(xValue);
		break;
		case '3':
			yValue = -0.5f * xValue;
		break;
		case '4':
			yValue = -abs(0.5f * xValue);
		break;
		case '5':
			yValue = -abs(xValue);
		break;
		case '6':
			yValue = xValue*xValue;
		break;
		case '7':
			yValue = xValue*xValue*xValue;
		break;
		case '8':
			yValue = sqrt(abs(0.5f * xValue)) * ((xValue >= 0) ? 1.0f : -1.0f);
		break;
		case '9':
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

//void printUsage(int argc, char **argv);
//bool parseArguments(int   argc,
//    char  **argv,
//    int   &nRobots,
//    int   &formationIndex,
//    float &fRadius,
//    float &fHeading,
//    int   &dt);
//bool validateParameters(const int   nRobots,
//    const int   formationIndex,
//    const float fRadius,
//    const float fHeading,
//    const int   dt);


// Prints the program usage message.
//void printUsage(int argc, char **argv)
//{
//  cout << "USAGE: " << argv[0]
//    << " [-n <nRobots>]"
//    << " [-f <formationIndex>]"
//    << " [-r <fRadius>]"
//    << " [-h <fHeading>]"
//    << " [-t <dT>]"
//    << endl
//    << "      -n <nRobots>: number of robots"
//    << " [0, MAX_N_ROBOTS] (DEFAULT=19)"        << endl
//    << "      -f <formationIndex>: formation function index"
//    << " [0, 9] (default=0):"                   << endl
//	<< "             0) f(x) = 0;"              << endl
//	<< "             1) f(x) = x;"              << endl
//	<< "             2) f(x) = |x|;"            << endl
//	<< "             3) f(x) = -0.5 x;"         << endl
//	<< "             4) f(x) = -|0.5 x|;"       << endl
//	<< "             5) f(x) = -|x|;"           << endl
//	<< "             6) f(x) = x^2;"            << endl
//	<< "             7) f(x) = x^3;"            << endl
//	<< "             8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0};"
//	<< endl
//	<< "             9) f(x) = 0.05 sin(10 x);" << endl
//	<< "      -r <fRadius>: formation radius"
//	<< " [0.0, 1.0] (default=?.?)"              << endl
//	<< "      -h <fHeading>: formation heading"
//	<< " (in degrees; default=90.0)"            << endl
//	<< "      -t <dt>: update time interval"
//	<< " [1, ??] (in milliseconds; default=50)"
//	<< endl;
//}   // printUsage(int, char **)


// Parses the parameterized program arguments,
// returning true if successful, false otherwise.


// Parameters:
//      argc          in      an argument counter
//      argv          in      initialization arguments
//      nRobots       in/out  the number of robots
//      formationIndex        in/out  the initial function index of the formation
//      fRadius       in/out  the initial radius between robots in the formation
//      fHeading      in/out  the initial heading of robots in the formation
//      dt            in/out  the time interval (in ms) between OpenGL updates
//

//bool parseArguments(int    argc,
//    char   **argv,
//    int   &nRobots,
//    int   &formationIndex,
//    float &fRadius,
//    float &fHeading,
//    int   &dt)
//{
//  int i = 0;
//  while (++i < argc)
//  {
//    if      (!strncmp(argv[i], "--help", 6))
//    {
//      printUsage(argc, argv);
//      exit(0);
//    }
//    else if (!strncmp(argv[i], "-n", 2))
//    {
//      if (++i < argc) nRobots = atoi(argv[i]);
//      else
//      {
//        printUsage(argc, argv);
//        return false;
//      }
//    }
//    else if (!strncmp(argv[i], "-f", 2))
//    {
//      if (++i < argc) formationIndex = atoi(argv[i]);
//      else
//      {
//        printUsage(argc, argv);
//        return false;
//      }
//    }
//    else if (!strncmp(argv[i], "-r", 2))
//    {
//      if (++i < argc) fRadius = atof(argv[i]);
//      else
//      {
//        printUsage(argc, argv);
//        return false;
//      }
//    }
//    else if (!strncmp(argv[i], ".h", 2))
//    {
//      if (++i < argc) fHeading = scaleDegrees(atof(argv[i]));
//      else
//      {
//        printUsage(argc, argv);
//        return false;
//      }
//    }
//    else if (!strncmp(argv[i], "-t", 2))
//    {
//      if (++i < argc) dt = atoi(argv[i]);
//      else
//      {
//        printUsage(argc, argv);
//        return false;
//      }
//    }
//  }
//
//  return true;
//}   // parseArguments(int, char **, int &, int &, float &, float &)


// Tests the validate if the specified parameters,
// returning true if successful, false otherwise.
//
// Parameters:
//      nRobots       in/out  the number of robots
//      formationIndex        in/out  the initial function index of the formation
//      fRadius       in/out  the initial radius between robots in the formation
//      fHeading      in/out  the initial heading of robots in the formation
//      dt            in/out  the time interval (in ms) between OpenGL updates
//
//bool validateParameters(const int   nRobots,
//    const int   formationIndex,
//    const float fRadius,
//    const float fHeading,
//    const int   dt)
//{
//  bool valid = true;
//
//  // validate the number of robots
//  if (nRobots < 0)
//  {
//    cout << "Parameter: "
//      << "'nRobots' must be non-negative." << endl;
//    valid = false;
//  }
//
//  // validate the formation function index
//  if ((formationIndex < 0) || (formationIndex > 9))
//  {
//    cout << "Parameter: "
//      << "'formationIndex' must be in the range [0, 9]." << endl;
//    valid = false;
//  }
//
//  // validate the formation radius
//  if (fRadius < 0.0f)
//  {
//    cout << "Parameter: "
//      << "'fRadius' must be non-negative." << endl;
//    valid = false;
//  }
//
//  // validate the time interval
//  if (dt < 1)
//  {
//    cout << "Parameter: "
//      << "'dt' must be greater than 1." << endl;
//    valid = false;
//  }
//
//  return valid;
//}   // validateParameters(int, int, float, float)
