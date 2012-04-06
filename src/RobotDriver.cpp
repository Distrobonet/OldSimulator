#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

//#include <boost/thread/thread.hpp>

#include <Simulator/RobotDriver.h>
//#include <Simulator/Simulator.h>

using namespace std;

#include "Simulator/Formation.h"
Function aFunction;
Formation asdf = Formation(aFunction, 1,Vector(),1,1,1);



// Global variables
double distanceToTarget = 0.0l;
double angleChange = 0.0l;

double distanceToTarget1 = 0.0l;
double angleChange1 = 0.0l;

double xValue = 0.0l;
double yValue = 0.0l;


int main(int argc, char** argv)
{

	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle aNode;
	ros::Rate loop_rate(10);


	ros::Subscriber subRobot0 = aNode.subscribe("/robot_0/base_pose_ground_truth", 1000, callBackRobot0);
	ros::Subscriber subRobot1 = aNode.subscribe("/robot_1/base_pose_ground_truth", 1000, callBackRobot1);
	ros::Subscriber subRobot2 = aNode.subscribe("/robot_2/base_pose_ground_truth", 1000, callBackRobot2);

	ros::Publisher pub_cmd_vel1 = aNode.advertise < geometry_msgs::Twist > ("/robot_1/cmd_vel", 1);
	ros::Publisher pub_cmd_vel2 = aNode.advertise < geometry_msgs::Twist > ("/robot_2/cmd_vel", 1);
	geometry_msgs::Twist commandVelocity;
	geometry_msgs::Twist commandVelocity1;


	// Menu
	displayMenu();

	// Primary ROS loop
	while(ros::ok())
	{
		keyboardInput();

		// Robot 1
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
			commandVelocity.linear.x = 0;
			commandVelocity.linear.y = 0;
			commandVelocity.angular.z = angleChange;
			pub_cmd_vel1.publish(commandVelocity);

			if((angleChange < 0.1 && angleChange > 0) || (angleChange > -0.1 && angleChange < 0))
			{
				commandVelocity.linear.x = distanceToTarget;
				commandVelocity.linear.y = distanceToTarget;
				commandVelocity.angular.z = 0;
				pub_cmd_vel1.publish(commandVelocity);
			}
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Robot 2
		xValue = velocityX0 - 1;
		yValue = getYValue(velocityY0 - 1);
		if(yValue != -9999)
			{
			distanceToTarget1 = getDistance(xValue, yValue, velocityX2, velocityY2);
			angleChange1 = getAngle(xValue, yValue, velocityX2, velocityY2, theta2);

			if (distanceToTarget1 < 0.00001){
				distanceToTarget1 = 0;
				angleChange1 = getAngle(xValue, velocityY0 + 2, velocityX2, velocityY2, theta2);
			}
			commandVelocity1.linear.x = 0;
			commandVelocity1.linear.y = 0;
			commandVelocity1.angular.z = angleChange1;
			pub_cmd_vel2.publish(commandVelocity1);

			if((angleChange1 < 0.1 && angleChange1 > 0) || (angleChange1 > -0.1 && angleChange1 < 0)){
				commandVelocity1.linear.x = distanceToTarget1;
				commandVelocity1.linear.y = distanceToTarget1;
				commandVelocity1.angular.z = 0;
				pub_cmd_vel2.publish(commandVelocity1);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return(0);
}


