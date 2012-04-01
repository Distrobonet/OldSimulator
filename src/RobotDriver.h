#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// Used for non-blocking user input
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Global variable
int currentSelection;


#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

using namespace std;

double velocityX0, velocityY0, theta0;
double velocityX1, velocityY1, theta1;
double velocityX2, velocityY2, theta2;

static void callBackRobot0(const nav_msgs::Odometry::ConstPtr& odom)
{
	velocityY0 = odom-> pose.pose.position.x;
	velocityX0 = -odom-> pose.pose.position.y;

	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);

	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	theta0 = angles::normalize_angle(yaw + M_PI / 2.0l);
	ros::spinOnce();
}

static void callBackRobot1(const nav_msgs::Odometry::ConstPtr& odom)
{
	velocityY1 = odom-> pose.pose.position.x;
	velocityX1 = -odom-> pose.pose.position.y;

	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);

	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	theta1 = angles::normalize_angle(yaw + M_PI / 2.0l);
	ros::spinOnce();
}

static void callBackRobot2(const nav_msgs::Odometry::ConstPtr& odom)
{
	velocityY2 = odom-> pose.pose.position.x;
	velocityX2 = -odom-> pose.pose.position.y;

	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);
	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	theta2 = angles::normalize_angle(yaw + M_PI / 2.0l);
	ros::spinOnce();
}

// Returns the distance between two points
double getDistance(double target_x, double target_y, double origin_x, double origin_y)
{
  return sqrt(pow(target_x - origin_x, 2) + pow(target_y - origin_y, 2));
}

// Returns the angle between two points
double getAngle(double target_x, double target_y, double origin_x, double origin_y, double origin_th)
{
  return angles::normalize_angle(atan2(target_y - origin_y, target_x - origin_x) - origin_th);
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
		<< "Use ESC to exit."                                << endl << endl
		<< "Please enter your selection: ";

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
// TODO: This should probably clear the terminal window each time the user changes the selection
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



#endif /* ROBOTDRIVER_H */
