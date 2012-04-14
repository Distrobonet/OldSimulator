/*
 * OverLord.h
 *
 *  Created on: Apr 13, 2012
 *      Author: glenn
 */

#ifndef OVERLORD_H_
#define OVERLORD_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>
#define SUBSCRIBER 0
#define ROBOT_LABEL 1

namespace std {

class OverLord {
public:
	OverLord();
	virtual ~OverLord();

    double robotX;
    double robotY;
    double robotTheta;
    nav_msgs::Odometry odomMsg;
    ros::Subscriber subRobot;

    string generateSubMessage(bool msgType);
    void updatePosition(int robotID);
    void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom);

    float getDistanceTo(const int fromID, const int toID) const;
    float getAngleTo(const int fromID, const int toID) const;


protected:
	Environment  *env;    // the environment of the robot
	static int  numOfRobots;    // number of total robots
};

} /* namespace std */
#endif /* OVERLORD_H_ */
