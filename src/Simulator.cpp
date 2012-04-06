//
// Filename:        "Simulator.cpp"
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
//#include <Simulator/GLIncludes.h>//ADDED BY KEVIN
//
//// Needed by Vector.h
//#include <Simulator/Utils.h>//ADDED BY KEVIN
//
//#include <Simulator/Behavior.h>//ADDED BY KEVIN
//
//// Needed by Environment
//#include <Simulator/Object.h>//ADDED BY KEVIN
//
//// Needed by Cell
//#include <Simulator/Auctioning.h>//ADDED BY KEVIN
//#include <Simulator/Neighborhood.h>//ADDED BY KEVIN
//#include <Simulator/Robot.h>//ADDED BY KEVIN
//
//// Needed by Environment
//#include <Simulator/Cell.h>//ADDED BY KEVIN

#include <Simulator/Environment.h>

//// Needed by Formation.h
//#include <Simulator/Relationship.h>//ADDED BY KEVIN
//
//// Needed by Formation.h
//#include <Simulator/Vector.h>//ADDED BY KEVIN
//
//// Needed by Simulator.cpp
//#include <Simulator/Formation.h>//ADDED BY KEVIN



using namespace std;


// define SIGPIPE if not defined (compatibility for win32)
#ifndef SIGPIPE
#define SIGPIPE 13
#endif


// simulation environment function prototypes
void printUsage(int argc, char **argv);
bool parseArguments(int   argc,
    char  **argv,
    int   &nRobots,
    int   &fIndex,
    float &fRadius,
    float &fHeading,
    int   &dt);
bool validateParameters(const int   nRobots,
    const int   fIndex,
    const float fRadius,
    const float fHeading,
    const int   dt);
void terminate(int retVal);
//void displayMenu();
bool initEnv(const int nRobots, const int fIndex);
bool deinitEnv();
bool changeFormation(const int index, const Vector gradient = Vector());
bool changeFormationSim(const int index, const Vector gradient = Vector());


// OpenGL function prototypes
void initWindow();
void display();
void keyboardPress(unsigned char keyPressed, int mouseX, int mouseY);
void keyboardPressSpecial(int keyPressed, int mouseX, int mouseY);
void keyboardReleaseSpecial(int keyReleased, int mouseX, int mouseY);
void mouseClick(int mouseButton, int mouseState,
    int mouseX,      int mouseY);
void mouseDrag(int mouseX, int mouseY);
void resizeWindow(GLsizei w, GLsizei h);
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



// OpenGL global variables
int   g_windowSize[2] = {800, 800};   // window size in pixels
float g_windowWidth   = 2.0f;         // resized window width
float g_windowHeight  = 2.0f;         // resized window height

//typedef float (*Function)(const float);//ADDED BY KEVIN

// simulation global constants
const float   SELECT_RADIUS     = 1.5f * DEFAULT_ROBOT_RADIUS;
const int     N_CELLS           = 0;
const int     MIDDLE_CELL       = 0;//(N_CELLS - 1) / 2;
// A formation is a vector of Functions, which are functions that take floats and return floats
const Formation DEFAULT_FORMATION = Formation(formations[0], DEFAULT_ROBOT_RADIUS * FACTOR_COLLISION_RADIUS, Vector(), MIDDLE_CELL, 0,  90.0f);


// simulation global variables
Environment *g_env           = NULL;
int        g_nRobots       = 0;
float      g_fRadius       = DEFAULT_FORMATION.getRadius();
int        g_sID           = DEFAULT_FORMATION.getSeedID();
int        g_fID           = DEFAULT_FORMATION.getFormationID();
float      g_fHeading      = DEFAULT_FORMATION.getHeading();
int        g_fIndex        = 0;
int        g_selectedIndex = g_sID;
int        g_dt            = 50;    // time interval (in milliseconds)
bool         g_prop_toggle   = false;




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
//int main(int argc, char **argv)
//{
//  // parse command line arguments
//  if (!parseArguments(argc, argv,
//        g_nRobots, g_fIndex, g_fRadius, g_fHeading, g_dt))
//  {
//    cerr << ">> ERROR: Unable to parse arguments...\n\n";
//    return 1;
//  }
//
//  // validate parameters
//  if (!validateParameters(g_nRobots, g_fIndex, g_fRadius, g_fHeading, g_dt))
//  {
//    cerr << ">> ERROR: Unable to validate parameters...\n\n";
//    return 1;
//  }
//
//  // create handler for interrupts (i.e., ^C)
//  if (signal(SIGINT, SIG_IGN) != SIG_IGN) signal(SIGINT, terminate);
//  signal(SIGPIPE, SIG_IGN);
//
//  // use the GLUT utility to initialize the window, to handle
//  // the input and to interact with the windows system
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
//
//  // initialize and execute the robot cell environment
//  if (!initEnv(g_nRobots, g_fIndex))
//  {
//    cerr << ">> ERROR: Unable to initialize simulation environment...\n\n";
//    return 1;
//  }
//  initWindow();
//  displayMenu();
//  glutMainLoop();
//
//  deinitEnv();
//
//  return 0;
//}   // main(int, char **)




// void printUsage(argc, argv)
// Last modified: 08Nov2009
//
// Prints the program usage message.
//
// Returns:     <none>
// Parameters:
//      argc    in      an argument counter
//      argv    in      initialization arguments
//
void printUsage(int argc, char **argv)
{
  cout << "USAGE: " << argv[0]
    << " [-n <nRobots>]"
    << " [-f <fIndex>]"
    << " [-r <fRadius>]"
    << " [-h <fHeading>]"
    << " [-t <dT>]"
    << endl
    << "      -n <nRobots>: number of robots"
    << " [0, MAX_N_ROBOTS] (DEFAULT=19)"        << endl
    << "      -f <fIndex>: formation function index"
    << " [0, 9] (default=0):"                   << endl
                                                   << "             0) f(x) = 0;"              << endl
                                                     << "             1) f(x) = x;"              << endl
                                                     << "             2) f(x) = |x|;"            << endl
                                                     << "             3) f(x) = -0.5 x;"         << endl
                                                     << "             4) f(x) = -|0.5 x|;"       << endl
                                                     << "             5) f(x) = -|x|;"           << endl
                                                     << "             6) f(x) = x^2;"            << endl
                                                     << "             7) f(x) = x^3;"            << endl
                                                     << "             8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0};"
                                                     << endl
                                                     << "             9) f(x) = 0.05 sin(10 x);" << endl
                                                     << "      -r <fRadius>: formation radius"
                                                     << " [0.0, 1.0] (default=?.?)"              << endl
                                                     << "      -h <fHeading>: formation heading"
                                                     << " (in degrees; default=90.0)"            << endl
                                                     << "      -t <dt>: update time interval"
                                                     << " [1, ??] (in milliseconds; default=50)"
                                                     << endl;
}   // printUsage(int, char **)




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
bool parseArguments(int    argc,
    char   **argv,
    int   &nRobots,
    int   &fIndex,
    float &fRadius,
    float &fHeading,
    int   &dt)
{
  int i = 0;
  while (++i < argc)
  {
    if      (!strncmp(argv[i], "--help", 6))
    {
      printUsage(argc, argv);
      exit(0);
    }
    else if (!strncmp(argv[i], "-n", 2))
    {
      if (++i < argc) nRobots = atoi(argv[i]);
      else
      {
        printUsage(argc, argv);
        return false;
      }
    }
    else if (!strncmp(argv[i], "-f", 2))
    {
      if (++i < argc) fIndex = atoi(argv[i]);
      else
      {
        printUsage(argc, argv);
        return false;
      }
    }
    else if (!strncmp(argv[i], "-r", 2))
    {
      if (++i < argc) fRadius = atof(argv[i]);
      else
      {
        printUsage(argc, argv);
        return false;
      }
    }
    else if (!strncmp(argv[i], ".h", 2))
    {
      if (++i < argc) fHeading = scaleDegrees(atof(argv[i]));
      else
      {
        printUsage(argc, argv);
        return false;
      }
    }
    else if (!strncmp(argv[i], "-t", 2))
    {
      if (++i < argc) dt = atoi(argv[i]);
      else
      {
        printUsage(argc, argv);
        return false;
      }
    }
  }

  return true;
}   // parseArguments(int, char **, int &, int &, float &, float &)



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



//// void displayMenu()
//// Last modified: 08Nov2009
////
//// Displays the following menu (program description) to the console:
////
//// ******************************************
//// *                                        *
//// * ALGORITHMS FOR CONTROL AND INTERACTION *
//// *     OF LARGE FORMATIONS OF ROBOTS      *
//// *                                        *
//// *   Ross Mead & Dr. Jerry B. Weinberg    *
//// *  Southern Illinois Univ. Edwardsville  *
//// *                                        *
//// ******************************************
////
//// Use the 'ARROW' keys to translate and rotate the seed robot.
////
//// Use the '+/-' keys to increase/decrease the separation between robots.
////
//// Use the '</>' keys to rotate the robot headings relative to the formation.
////
//// Use the '0-9' keys to change to a formation seeded at the selected robot.
////
//// PRESET FORMATIONS
//// -----------------
//// 0) f(x) = 0
//// 1) f(x) = x
//// 2) f(x) = |x|
//// 3) f(x) = -0.5 x
//// 4) f(x) = -|0.5 x|
//// 5) f(x) = -|x|
//// 6) f(x) = x^2
//// 7) f(x) = x^3
//// 8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0}
//// 9) f(x) = 0.05 sin(10 x)
////
//// Use the mouse to select a robot.
////
//// Use 'h|l|p|t' to toggle robot display settings.
////
//// Returns:     <none>
//// Parameters:  <none>
////
////void displayMenu()
////{
////  cout << "******************************************"      << endl
////    << "*                                        *"      << endl
////    << "* ALGORITHMS FOR CONTROL AND INTERACTION *"      << endl
////    << "*     OF LARGE FORMATIONS OF ROBOTS      *"      << endl
////    << "*                                        *"      << endl
////    << "*   Ross Mead & Dr. Jerry B. Weinberg    *"      << endl
////    << "*  Southern Illinois Univ. Edwardsville  *"      << endl
////    << "*                                        *"      << endl
////    << "******************************************"      << endl << endl
////    << "Use the 'ARROW' keys to "
////    << "translate and rotate the seed robot."
////    << endl << endl
////    << "Use the '+/-' keys to "
////    << "increase/decrease the separation between robots."
////    << endl << endl
////    << "Use the '</>' keys to "
////    << "rotate the robot headings relative to the formation."
////    << endl << endl
////    << "Use the '0-9' keys to "
////    << "change to a formation seeded at the selected robot."
////    << endl << endl
////    << "PRESET FORMATIONS\n-----------------"            << endl
////    << "0) f(x) = 0"                                     << endl
////    << "1) f(x) = x"                                     << endl
////    << "2) f(x) = |x|"                                   << endl
////    << "3) f(x) = -0.5 x"                                << endl
////    << "4) f(x) = -|0.5 x|"                              << endl
////    << "5) f(x) = -|x|"                                  << endl
////    << "6) f(x) = x^2"                                   << endl
////    << "7) f(x) = x^3"                                   << endl
////    << "8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0}"  << endl
////    << "9) f(x) = 0.05 sin(10 x)"                        << endl << endl
////    << "Use the mouse to select a robot."                << endl << endl
////    << "Use 'h|l|p|t' to toggle robot display settings." << endl << endl
////    << "Use ESC to exit."                                << endl << endl;
////}   // displayMenu()



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

  Formation f(formations[fIndex], g_fRadius, Vector(),
      g_sID,            ++g_fID,     g_fHeading);
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
  g_fIndex = index;
  if (!g_env->startFormation)
  {
    g_env->startFormation = true;
  }
  // determine if a new seed has been selected
  if (g_selectedIndex != -1)
  {
    g_env->getCell(g_sID)->setColor(DEFAULT_CELL_COLOR);
    g_sID = g_selectedIndex;
  }
  Formation f(formations[index], g_fRadius,     gradient,
      g_sID,           ++g_env->formationID, g_fHeading);

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
    g_env->getCell(g_sID)->setColor(DEFAULT_CELL_COLOR);
    g_selectedIndex = index;
    return changeFormation(g_fIndex,gradient);
  }
  else return false;
}

bool sendNCellRequest()
{
  PropMsg *ncell = new PropMsg();
  return g_env->sendMsg(ncell, g_sID,ID_OPERATOR, NCELL_REQUEST);
}  //sendNCellRequest()


bool sendFcntrRequest()
{
  PropMsg *fcntr = new PropMsg();
  return g_env->sendMsg(fcntr, g_sID,ID_OPERATOR, FCNTR_REQUEST);
}  //sendFcntrRequest()

bool sendFRadRequest()
{
  PropMsg *frad = new PropMsg();
  return g_env->sendMsg(frad, g_sID,ID_OPERATOR, FRAD_REQUEST);
}  //sendFRadRequest()
bool sendFSeedRequest()
{
  PropMsg *fseed = new PropMsg();
  return g_env->sendMsg(fseed, g_sID,ID_OPERATOR, FSEED_REQUEST);
}// sendFSeedRequest();

//
// void initWindow()
// Last modified:   08Nov2009
//
// Initializes the simulator window.
//
// Returns:     <none>
// Parameters:  <none>
//
void initWindow()
{

  // clear background color
  glClearColor(g_env->color[0], g_env->color[1], g_env->color[2], 0.0f);

  // viewport transformation
  glViewport(0, 0, g_windowSize[0], g_windowSize[1]);

  glMatrixMode(GL_PROJECTION);    // projection transformation
  glLoadIdentity();               // initialize projection identity matrix
}   // initWindow()



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
  glClear(GL_COLOR_BUFFER_BIT);   // clear background color
  glMatrixMode(GL_MODELVIEW);     // modeling transformation

  // draws environment robot cells
  if (g_env->getCells().size() > 0)
  {
    g_env->getCell(g_sID)->setColor(GREEN);
    for(int i = 0; i < g_env->getNCells(); ++i)
    {
      if(g_env->getCell(i) != g_env->getCell(g_sID))
      {
        if(g_env->getCell(i) == g_env->getCell(g_selectedIndex))
        {
          g_env->getCell(i)->setColor(RED);
        }
        else
        {
          g_env->getCell(i)->setColor(DEFAULT_CELL_COLOR);
        }
      }
    }
  }
  g_env->draw();

  glFlush();                      // force the execution of OpenGL commands
  glutSwapBuffers();              // swap visible buffer and writing buffer
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
        g_fHeading += g_env->getCell(g_sID)->maxAngSpeed();
        changeFormation(g_fIndex);
        g_env->getCell(g_sID)->rotateRelative(
            g_env->getCell(g_sID)->maxAngSpeed());
        //min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;
    case '>': case '.':
      if (g_env->getNCells() > 0)
      {
        g_fHeading -= g_env->getCell(g_sID)->maxAngSpeed();
        changeFormation(g_fIndex);
        g_env->getCell(g_sID)->rotateRelative(
            -g_env->getCell(g_sID)->maxAngSpeed());
        //-min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;

      // change formation scale
    case '+': case '=':
      if (g_env->getNCells() > 0)
      {
        g_fRadius += 0.01f;
        changeFormation(g_fIndex);
      }
      break;
    case '-': case '_':
      if (g_env->getNCells() > 0)
      {
        g_fRadius -= 0.01f;
        g_fRadius  = max(g_fRadius,
            g_env->getCell(g_sID)->collisionRadius());
        changeFormation(g_fIndex);
      }
      break;

    case 'h': case 'H':
      if (g_env->getNCells() > 0)
      {
        g_env->showHeading(!g_env->getCell(g_sID)->showHeading);
      }
      break;
    case 'l': case 'L':
      if (g_env->getNCells() > 0)
      {
        g_env->showLine(!g_env->getCell(g_sID)->heading.showLine);
      }
      break;
    case 'p': case 'P':
      if (g_env->getNCells() > 0)
      {
        g_env->showPos(!g_env->getCell(g_sID)->showPos);
      }
      break;
    case 't': case 'T':
      if (g_env->getNCells() > 0)
      {
        g_env->showHead(!g_env->getCell(g_sID)->heading.showHead);
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
        g_env->getCell(g_sID)->rotError =
          -(1.0001f * g_env->getCell(g_sID)->angThreshold());
        //g_env->getCell(g_sID)->rotateRelative(
        //    min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;
    case GLUT_KEY_UP:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_sID)->transError.x =
          1.0001f * g_env->getCell(g_sID)->threshold();
        //g_env->getCell(g_sID)->translateRelative(
        //    min(0.001f, g_env->getCell(g_sID)->maxSpeed()));
      }
      break;
    case GLUT_KEY_RIGHT:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_sID)->rotError =
          1.0001f * g_env->getCell(g_sID)->angThreshold();
        //g_env->getCell(g_sID)->rotateRelative(
        //    -min(1.0f, g_env->getCell(g_sID)->maxAngSpeed()));
      }
      break;
    case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_sID)->transError.x =
          -(1.0001f * g_env->getCell(g_sID)->threshold());
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
        g_env->getCell(g_sID)->rotError = 0.0f;
      }
      break;
    case GLUT_KEY_UP: case GLUT_KEY_DOWN:
      if (g_env->getNCells() > 0)
      {
        g_env->getCell(g_sID)->transError.x = 0.0f;
      }
      break;
    default: break;
  }
}   // keyboardReleaseSpecial(int, int, int)



//
// void mouseClick(mouseButton, mouseState, mouseX, mouseY)
// Last modified:   08Nov2009
//
// Reacts to mouse clicks.
//
// Returns:     <none>
// Parameters:
//      mouseButton     in      the mouse button that was pressed
//      mouseState      in      the state of the mouse
//      mouseX          in      the x-coordinate of the mouse position
//      mouseY          in      the y-coordinate of the mouse position
//
void mouseClick(int mouseButton,    int mouseState,
    int mouseX, int mouseY)
{
  int mod = glutGetModifiers();
  if (mouseState == GLUT_DOWN)
  {
    if(mod == GLUT_ACTIVE_CTRL)
    {
      if(g_env->getCells().size() > 0)
      {
        for (int i = 0; i < g_env->getNCells(); ++i)
        {
          float x     = g_windowWidth * mouseX / g_windowSize[0] -
            0.5 * g_windowWidth;
          float y     = 0.5 * g_windowHeight -
            (g_windowHeight * mouseY / g_windowSize[1]);
          g_selectedIndex = g_sID;
          float dx = g_env->getCell(i)->x - x,
                  dy = g_env->getCell(i)->y - y;
          if ((g_selectedIndex == g_sID) &&
              (sqrt(dx * dx  + dy * dy) < SELECT_RADIUS))
          {
            g_env->removeCell(g_env->getCell(g_selectedIndex = i));

          }else if (i != g_sID){
            g_env->getCell(i)->setColor(DEFAULT_CELL_COLOR);
          }
        }
      }
      if(g_env->getRobots().size()>0)
      {
        int ii=-1;
        for (int i = 0; i < g_env->getNFreeRobots(); ++i)
        {
          float x     = g_windowWidth * mouseX / g_windowSize[0] -
            0.5 * g_windowWidth;
          float y     = 0.5 * g_windowHeight -
            (g_windowHeight * mouseY / g_windowSize[1]);
          g_selectedIndex = -1;
          float dx = g_env->getRobot(ii)->x - x,
                  dy = g_env->getRobot(ii)->y - y;
          float distance = sqrt(dx * dx  + dy * dy);
          cout << "Distance between click and robotID " << ii << " is " << distance << endl;
          if (distance < SELECT_RADIUS)
          {
            string printf("IS WITHIN RADIUS");
            cout << "Remove robotID = " << ii << endl;
            g_env->removeRobot(g_env->getRobot(g_selectedIndex = ii));
            //g_env->getCell(i)->setColor(DEFAULT_CELL_COLOR);
            //cout << "Remove robotID = " << ii << endl;
            //g_env->removeRobot(g_env->getRobot(g_selectedIndex = ii));
          }
          ii--;
        }
      }
    }
    if(g_env->getCells().size()>0)
    {
      float x     = g_windowWidth * mouseX / g_windowSize[0] -
        0.5 * g_windowWidth;
      float y     = 0.5 * g_windowHeight -
        (g_windowHeight * mouseY / g_windowSize[1]);
      g_selectedIndex = g_sID;
      for (int i = 0; i < g_env->getNCells(); ++i)
      {
        float dx = g_env->getCell(i)->x - x,
                dy = g_env->getCell(i)->y - y;
        if ((g_selectedIndex == g_sID) &&
            (sqrt(dx * dx  + dy * dy) < SELECT_RADIUS))
        {
         g_env->getCell(g_selectedIndex = i)->setColor(RED);
        }
        else if (i != g_sID)
        {
          g_env->getCell(i)->setColor(RED);
        }
      }
    }else{
      float x     = g_windowWidth * mouseX / g_windowSize[0] -
        0.5 * g_windowWidth;
      float y     = 0.5 * g_windowHeight -
        (g_windowHeight * mouseY / g_windowSize[1]);
      g_env->formFromClick(x,y);
    }
  }
  glutPostRedisplay();            // redraw the scene
}   // mouseClick(int, int, int, int)



//
// void mouseDrag(mouseX, mouseY)
// Last modified:   08Nov2009
//
// Moves the position of a nearby cell to the
// current mouse pointer position if the mouse
// button is pressed and the pointer is in motion.
//
// Returns:     <none>
// Parameters:
//      mouseX  in      the x-coordinate of the mouse position
//      mouseY  in      the y-coordinate of the mouse position
//
void mouseDrag(int mouseX, int mouseY)
{
  if (g_env->getNCells() > 0)
  {
    if (g_selectedIndex != ID_NO_NBR)
    {
      g_env->getCell(g_selectedIndex)->
        set(g_windowWidth * mouseX / g_windowSize[0] - 0.5 *
            g_windowWidth, 0.5 * g_windowHeight - (g_windowHeight *
              mouseY / g_windowSize[1]));
    }
  }
  glutPostRedisplay();    // redraw the scene
}   // mouseDrag(int, int)




// void resizeWindow(w, h)
// Last modified:   08Nov2009
//
// Scales the rendered scene according to the window dimensions,
// setting the global variables so the mouse operations will
// correspond to mouse pointer positions.
//
// Returns:     <none>
// Parameters:
//      w       in      the new screen width
//      h       in      the new screen height

void resizeWindow(GLsizei w, GLsizei h)
{
  g_windowSize[0] = w;            // obtain new screen width
  g_windowSize[1] = h;            // obtain new screen height

  glClear(GL_COLOR_BUFFER_BIT);   // clear color buffer to draw next frame
  glViewport(0, 0, w, h);         // viewport transformation

  glMatrixMode(GL_PROJECTION);    // projection transformation
  glLoadIdentity();               // initialize projection identity matrix

  if (w <= h)
  {
    g_windowWidth  = 2.0f;
    g_windowHeight = 2.0f * (float)h / (float)w;
    glOrtho(-1.0f, 1.0f, -1.0f * (float)h / (float)w,
        (float)h / (float)w, -10.0f, 10.0f);
  }
  else
  {
    g_windowWidth  = 2.0f * (float)w / (float)h;
    g_windowHeight = 2.0f;
    glOrtho(-1.0f * (float)w / (float)h,
        (float)w / (float)h,
        -1.0f, 1.0f, -10.0f, 10.0f);
  }
  glutPostRedisplay();            // redraw the scene
}   // resizeWindow(GLsizei, GLsizei)



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
  glutPostRedisplay();    // redraw the scene
  glutTimerFunc(g_dt, timerFunction, 1);
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
