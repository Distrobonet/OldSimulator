#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// Used for non-blocking user input
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>




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
//double getYValue(double xValue)
//{
//	double yValue = -9999.0l;
//	switch(currentSelection)
//	{
//		case '0':
//			yValue = 0.0l;
//		break;
//		case '1':
//			yValue = xValue;
//		break;
//		case '2':
//			yValue = abs(xValue);
//		break;
//		case '3':
//			yValue = -0.5f * xValue;
//		break;
//		case '4':
//			yValue = -abs(0.5f * xValue);
//		break;
//		case '5':
//			yValue = -abs(xValue);
//		break;
//		case '6':
//			yValue = xValue*xValue;
//		break;
//		case '7':
//			yValue = xValue*xValue*xValue;
//		break;
//		case '8':
//			yValue = sqrt(abs(0.5f * xValue)) * ((xValue >= 0) ? 1.0f : -1.0f);
//		break;
//		case '9':
//			yValue = 0.05f * sin(10.0f * xValue);;
//		break;
//	}
//	return yValue;
//}








#endif /* ROBOTDRIVER_H */
