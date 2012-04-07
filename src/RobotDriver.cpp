//
// Filename:        "RobotDriver.cpp"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This program tests the robot cell simulator.
//

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


#include "Simulator/RobotDriver.h"

#include <Simulator/Environment.h>

using namespace std;


// define SIGPIPE if not defined (compatibility for win32)
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

// simulation environment function prototypes
bool validateParameters(const int   nRobots,
    const int   fIndex,
    const float fRadius,
    const float fHeading,
    const int   dt);
void terminate(int retVal);
void displayMenu();
void keyboardInput();
void clearScreen();
bool initEnv(const int nRobots, const int fIndex);
bool deinitEnv();
bool changeFormation(const int index, const Vector gradient = Vector());
bool changeFormationSim(const int index, const Vector gradient = Vector());


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



// OpenGL global constants
const int INIT_WINDOW_POSITION[2] = {0, 0};      // window offset
const char  CHAR_ESCAPE             = char(27);    // 'ESCAPE' character key

// Menu Global variable
int currentSelection;

// OpenGL global variables
int   g_windowSize[2] = {800, 800};   // window size in pixels
float g_windowWidth   = 2.0f;         // resized window width
float g_windowHeight  = 2.0f;         // resized window height

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




// int main(argc, argv)
// Last modified:   04Sep2006
//
// Uses the OpenGL Utility Toolkit to set the
// window up to display the window and its contents.
//
// Returns:     the result of the OpenGL main loop
// Parameters:
//      argc    in      an argument counter
//      argv    in      initialization arguments
//
int main(int argc, char **argv)
{
  // parse command line arguments
//  if (!parseArguments(argc, argv,
//        g_nRobots, g_formationIndex, g_formationRadius, g_formationHeading, g_dTime))
//  {
//    cerr << ">> ERROR: Unable to parse arguments...\n\n";
//    return 1;
//  }

  // validate parameters
  if (!validateParameters(g_nRobots, g_formationIndex, g_formationRadius, g_formationHeading, g_dTime))
  {
    cerr << ">> ERROR: Unable to validate parameters...\n\n";
    return 1;
  }

  // create handler for interrupts (i.e., ^C)
  if (signal(SIGINT, SIG_IGN) != SIG_IGN) signal(SIGINT, terminate);
  signal(SIGPIPE, SIG_IGN);

  // use the GLUT utility to initialize the window, to handle
  // the input and to interact with the windows system
//  glutInit(&argc, argv);
//  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
//  glutInitWindowSize(g_windowSize[0], g_windowSize[1]);
//  glutInitWindowPosition(INIT_WINDOW_POSITION[0], INIT_WINDOW_POSITION[1]);
//  glutCreateWindow("Simulator");
//
//  // specify the resizing, refreshing, and interactive routines
//  glutDisplayFunc(display);
//  glutIdleFunc(display);
//  glutKeyboardFunc(keyboardPress);
//  glutMouseFunc(mouseClick);
//  glutMotionFunc(mouseDrag );
//  glutReshapeFunc(resizeWindow);
//  glutSpecialFunc(keyboardPressSpecial);
//  glutSpecialUpFunc(keyboardReleaseSpecial);
//  glutTimerFunc(50, timerFunction, 1);

  // initialize and execute the robot cell environment
  if (!initEnv(g_nRobots, g_formationIndex))
  {
    cerr << ">> ERROR: Unable to initialize simulation environment...\n\n";
    return 1;
  }



   // BEGIN DISTROBONET CODE



	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle aNode;
	ros::Publisher chatter_pub = aNode.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	ros::Subscriber subRobot0 = aNode.subscribe("/robot_0/base_pose_ground_truth", 1000, callBackRobot0);

	ros::Publisher pub_cmd_vel1 = aNode.advertise < geometry_msgs::Twist > ("/robot_1/cmd_vel", 1);

	geometry_msgs::Twist commandVelocity;

	displayMenu();


	keyboardInput();

//	while (ros::ok())
//	{
//		cout << "basdgf";
//		std_msgs::String msg;
//		std::stringstream ss;
//		  ss << "hello world " << count;
//		  msg.data = ss.str();
//		  ROS_INFO("%s", msg.data.c_str());
//
//		  chatter_pub.publish(msg);
//
//		 ros::spinOnce();
//		 loop_rate.sleep();
//		 ++count;
//
//	}


















  // END DISTROBONET CODE

  deinitEnv();

  return 0;
}   // main(int, char **)



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

// void displayMenu()
// Last modified: 08Nov2009
//
// Displays the following menu (program description) to the console:
//
// Returns:     <none>
// Parameters:  <none>
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





// bool parseArguments(argc, argv, nRobots, fIndex, fRadius, fHeading, dt)
// Last modified: 08Nov2009
//
// Parses the parameterized program arguments,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      argc          in      an argument counter
//      argv          in      initialization arguments
//      nRobots       in/out  the number of robots
//      fIndex        in/out  the initial function index of the formation
//      fRadius       in/out  the initial radius between robots in the formation
//      fHeading      in/out  the initial heading of robots in the formation
//      dt            in/out  the time interval (in ms) between OpenGL updates
//



//
// bool validateParameters(nRobots, fIndex, fRadius, fHeading, dt)
// Last modified: 08Nov2009
//
// Tests the validate if the specified parameters,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      nRobots       in/out  the number of robots
//      fIndex        in/out  the initial function index of the formation
//      fRadius       in/out  the initial radius between robots in the formation
//      fHeading      in/out  the initial heading of robots in the formation
//      dt            in/out  the time interval (in ms) between OpenGL updates
//
bool validateParameters(const int   nRobots,
    const int   fIndex,
    const float fRadius,
    const float fHeading,
    const int   dt)
{
  bool valid = true;

  // validate the number of robots
  if (nRobots < 0)
  {
    cout << "Parameter: "
      << "'nRobots' must be non-negative." << endl;
    valid = false;
  }

  // validate the formation function index
  if ((fIndex < 0) || (fIndex > 9))
  {
    cout << "Parameter: "
      << "'fIndex' must be in the range [0, 9]." << endl;
    valid = false;
  }

  // validate the formation radius
  if (fRadius < 0.0f)
  {
    cout << "Parameter: "
      << "'fRadius' must be non-negative." << endl;
    valid = false;
  }

  // validate the time interval
  if (dt < 1)
  {
    cout << "Parameter: "
      << "'dt' must be greater than 1." << endl;
    valid = false;
  }

  return valid;
}   // validateParameters(int, int, float, float)




// void terminate(retVal)
// Last modified: 08Nov2009
//
// Terminates the program on interrupt (i.e., ^C).
//
// Returns:     <none>
// Parameters:
//      retVal  in      the exit return code
//
void terminate(int retVal)
{
  signal(SIGINT, SIG_IGN);
  deinitEnv();
  signal(SIGINT, SIG_DFL);
  exit(retVal);
}   // terminate(int)









//
// bool initEnv(const int, const int)
// Last modified: 08Nov2009
//
// Attempts to initialize the environment based on
// the parameterized values, returning true if successful,
// false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      nRobots       in      the number of robots
//      fIndex        in      the index of the initial formation
//
bool initEnv(const int nRobots, const int fIndex)
{
  if (g_env != NULL)
  {
    delete g_env;
    g_env = NULL;
  }

  Formation f(formations[fIndex], g_formationRadius, Vector(),
      g_seedID,            ++g_formationID,     g_formationHeading);
  if ((g_env = new Environment(nRobots, f)) == NULL) return false;
  return true;
}   // initEnv(const int, const int)




// bool deinitEnv()
// Last modified: 08Nov2009
//
// Attempts to deinitialize the environment,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:  <none>

bool deinitEnv()
{
  if (g_env != NULL)
  {
    delete g_env;
    g_env = NULL;
  }
  return g_env == NULL;
}   // deinitEnv()



//
// bool changeFormation(index)
// Last modified: 08Nov2009
//
// Attempts to change the current formation,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
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
}   // changeFormation(const int, const Vector)

// bool changeFormationSim(index)
// last modified: april 18, 2010
//
// called by environment, passes the location of a new calculated cell
// index.
// returns: true if successful, false if not
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
}  //sendNCellRequest()


bool sendFcntrRequest()
{
  PropMsg *fcntr = new PropMsg();
  return g_env->sendMsg(fcntr, g_seedID,ID_OPERATOR, FCNTR_REQUEST);
}  //sendFcntrRequest()

bool sendFRadRequest()
{
  PropMsg *frad = new PropMsg();
  return g_env->sendMsg(frad, g_seedID,ID_OPERATOR, FRAD_REQUEST);
}  //sendFRadRequest()
bool sendFSeedRequest()
{
  PropMsg *fseed = new PropMsg();
  return g_env->sendMsg(fseed, g_seedID,ID_OPERATOR, FSEED_REQUEST);
}// sendFSeedRequest();


//
// void display()
// Last modified:   08Nov2009
//
// Clears the frame buffer and draws the simulated cells within the window.
//
// Returns:     <none>
// Parameters:  <none>
//
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
}   // display()



//
// void keyboardPress(keyPressed, mouseX, mouseY)
// Last modified:   08Nov2009
//
// Handles the keyboard input (ASCII Characters).
//
// Returns:     <none>
// Parameters:
//      keyPressed  in      the ASCII key that was pressed
//      mouseX      in      the x-coordinate of the mouse position
//      mouseY      in      the y-coordinate of the mouse position
//
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
      {
        g_env->showHeading(!g_env->getCell(g_seedID)->showHeading);
      }
      break;
    case 'l': case 'L':
      if (g_env->getNCells() > 0)
      {
        g_env->showLine(!g_env->getCell(g_seedID)->heading.showLine);
      }
      break;
    case 'p': case 'P':
      if (g_env->getNCells() > 0)
      {
        g_env->showPos(!g_env->getCell(g_seedID)->showPos);
      }
      break;
    case 't': case 'T':
      if (g_env->getNCells() > 0)
      {
        g_env->showHead(!g_env->getCell(g_seedID)->heading.showHead);
      }
      break;
    case 'n': case 'N':
      if (g_env->getNCells() > 0)
      {
        //g_prop_toggle = !g_prop_toggle;
        sendNCellRequest();
      }
      break;
    case 'c': case 'C':
      if (g_env->getNCells() > 0)
      {
        sendFcntrRequest();
      }
      break;
    case 'r': case 'R':
      if (g_env->getNCells() > 0)
      {
        sendFRadRequest();
      }
      break;
    case 's': case 'S':
      if (g_env->getNCells()> 0)
      {
        sendFSeedRequest();
      }
      break;
    case 'a': case 'A':
      g_env->addObject(randSign() * frand(),
                       randSign() * frand(),
                       0.0f);
/*
      g_env->addRobot(randSign() * frand(),
                      randSign() * frand(),
                      0.0f,
                      randSign() * frand(0.0f, 180.0f));
*/
      break;
    case 'd': case 'D':
      g_env->removeObject();
/*
      if ((g_selectedIndex >= 0) && (g_selectedIndex < g_env->getNCells()))
      {
        g_env->removeCell(g_env->getCell(g_selectedIndex));
		g_selectedIndex = g_sID;
        //g_env->removeRobot();
      }
*/
      break;
    case CHAR_ESCAPE: deinitEnv(); exit(0);
  }
}   // keyboardPress(unsigned char, int, int)



//
// void keyboardPressSpecial(keyPressed, mouseX, mouseY)
// Last modified:   08Nov2009
//
// Handles the keyboard input (non-ASCII Characters).
//
// Returns:     <none>
// Parameters:
//      keyPressed  in      the non-ASCII key that was pressed
//      mouseX      in      the x-coordinate of the mouse position
//      mouseY      in      the y-coordinate of the mouse position
//
void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY)
{
  switch (keyPressed)
  {
    case GLUT_KEY_LEFT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->rotError =
          -(1.0001f * g_env->getCell(g_seedID)->angThreshold());
        //g_env->getCell(g_sID)->rotateRelative(
        //    min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;
    case GLUT_KEY_UP:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->transError.x =
          1.0001f * g_env->getCell(g_seedID)->threshold();
        //g_env->getCell(g_sID)->translateRelative(
        //    min(0.001f, g_env->getCell(g_sID)->maxSpeed()));
      }
      break;
    case GLUT_KEY_RIGHT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->rotError =
          1.0001f * g_env->getCell(g_seedID)->angThreshold();
        //g_env->getCell(g_sID)->rotateRelative(
        //    -min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;
    case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->transError.x =
          -(1.0001f * g_env->getCell(g_seedID)->threshold());
        //g_env->getCell(g_sID)->translateRelative(
        //    -min(0.001f, g_env->getCell(g_sID)->maxSpeed()));
      }
      break;
    default: break;
  }
}   // keyboardPressSpecial(int, int, int)



//
// void keyboardReleaseSpecial(keyPressed, mouseX, mouseY)
// Last modified:   06Nov2009
//
// Handles the keyboard input (non-ASCII Characters).
//
// Returns:     <none>
// Parameters:
//      keyReleased  in      the non-ASCII key that was released
//      mouseX       in      the x-coordinate of the mouse position
//      mouseY       in      the y-coordinate of the mouse position
//
void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY)
{
  switch (keyReleased)
  {
    case GLUT_KEY_LEFT: case GLUT_KEY_RIGHT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->rotError = 0.0f;
      }
      break;
    case GLUT_KEY_UP: case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_seedID)->transError.x = 0.0f;
      }
      break;
    default: break;
  }
}   // keyboardReleaseSpecial(int, int, int)





// void timerFunction(value)
// Last modified:   08Nov2009
//
// Updates the environment and redraws.
//
// Returns:     <none>
// Parameters:
//      value   in      the value of the timer
//
void timerFunction(int value)
{
  //sendFcntrRequest();
  //sendFRadRequest();
  //if(g_prop_toggle)
    //sendNCellRequest();
  g_env->step();          // update the robot cell environment

  // force a redraw after a number of milliseconds
//  glutPostRedisplay();    // redraw the scene
//  glutTimerFunc(g_dt, timerFunction, 1);
}   // timerFunction(int)



// <test formation functions>

//
// float line(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = 0.
//
// Returns:     f(x) = 0
// Parameters:
//      x       in      used to evaluate the function
//
float line(const float x)
{
  return 0.0f;
}   // line(const float)



//
// float x(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = x.
//
// Returns:     f(x) = x
// Parameters:
//      x       in      used to evaluate the function
//
float x(const float x)
{
  return x;
}   // x(const float)



//
// float absX(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = |x|.
//
// Returns:     f(x) = |x|
// Parameters:
//      x       in      used to evaluate the function
//
float absX(const float x)
{
  return abs(x);
}   // absX(const float)



//
// float negHalfX(x)
// Last modified:   07Jan2007
//
// Returns formation function definition f(x) = -0.5 x.
//
// Returns:     f(x) = -0.5 x
// Parameters:
//      x       in      used to evaluate the function
//
float negHalfX(const float x)
{
  return -0.5f * x;
}   // negHalfX(const float)



//
// float negAbsHalfX(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = -|0.5 x|.
//
// Returns:     f(x) = -|0.5 x|
// Parameters:
//      x       in      used to evaluate the function
//
float negAbsHalfX(const float x)
{
  return -abs(0.5f * x);
}   // negAbsHalfX(const float)



//
// float negAbsX(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = -|x|.
//
// Returns:     f(x) = -|x|
// Parameters:
//      x       in      used to evaluate the function
//
float negAbsX(const float x)
{
  return -abs(x);
}   // negAbsX(const float)



//
// float parabola(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = x^2.
//
// Returns:     f(x) = x^2
// Parameters:
//      x       in      used to evaluate the function
//
float parabola(const float x)
{
  return x * x;
  //return pow(x, 2.0f);
}   // parabola(const float)



//
// float cubic(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = x^3.
//
// Returns:     f(x) = x^3
// Parameters:
//      x       in      used to evaluate the function
//
float cubic(const float x)
{
  return x * x * x;
  //return pow(x, 3.0f);
}   // cubic(const float)



//
// float condSqrt(x)
// Last modified:   04Sep2006
//
// Returns formation function definition
// f(x) = {sqrt(x),  x = 0 | -sqrt|x|, x < 0}.
//
// Returns:     f(x) = {sqrt(x),  x = 0 | -sqrt|x|, x < 0}
// Parameters:
//      x       in      used to evaluate the function
//
float condSqrt(const float x)
{
  return sqrt(abs(0.5f * x)) * ((x >= 0) ? 1.0f : -1.0f);
}   // condSqrt(const float)



//
// float sine(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = 0.05 sin(10 x).
//
// Returns:     f(x) = 0.05 sin(10 x)
// Parameters:
//      x       in      used to evaluate the function
//
float sine(const float x)
{
  return 0.2f * sin(10.0f * x);
}   // sine(const float)



//
// float xRoot3(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = x sqrt(3).
//
// Returns:     f(x) = x sqrt(3)
// Parameters:
//      x       in      used to evaluate the function
//
float xRoot3(const float x)
{
  return x * sqrt(3.0f);
}   // xRoot3(const float)



//
// float negXRoot3(x)
// Last modified:   04Sep2006
//
// Returns formation function definition f(x) = -x sqrt(3).
//
// Returns:     f(x) = -x sqrt(3)
// Parameters:
//      x       in      used to evaluate the function
//
float negXRoot3(const float x)
{
  return -x * sqrt(3.0f);
}   // negXRoot3(const float)
