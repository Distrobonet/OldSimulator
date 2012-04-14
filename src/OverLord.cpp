/*
 * OverLord.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: glenn
 */

#include "OverLord.h"

namespace std {

OverLord::OverLord() {
	Robot *tempBotPtr = this;
	ros::NodeHandle aNode;
	vector<subRobot> subRobots;

	for(int i = 0; i < numOfRobots; i++){
		subRobots.push_back() = aNode.subscribe(generateSubMessage(SUBSCRIBER), 1000, &Robot::callBackRobot, tempBotPtr);
		odomMsg.header.frame_id = generateSubMessage(ROBOT_LABEL);
	}

}

OverLord::~OverLord() {
	// TODO Auto-generated destructor stub
}

string OverLord::generateSubMessage(bool msgType)
{
	stringstream ss;//create a stringstream
	ss << (numOfRobots);//add number to the stream
	string numRobots = ss.str();

	// Subscriber
	if(msgType == SUBSCRIBER)
	{
		string subString = "/robot_/odom";

		subString.insert(7, numRobots);
		return subString;

	}

	//else Robot label
	string subString = "";
	subString.insert(0, numRobots);
	return subString;
}

void OverLord::updatePosition(int robot)
{
	Robot *r = env->getRobot(robot);
	r->robotX = velocityX;
	r->robotY = velocityY;
	r->robotTheta = velocityTheta;

	//	cout<<"id "<<r->ID<<endl;
	//	cout<<"pub "<<r->pub_cmd_vel.getTopic()<<endl;
}

void OverLord::callBackRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	int robotID = atoi(odom->header.frame_id.substr(7,1).c_str());

	velocityY = odom-> pose.pose.position.x;
	velocityX = -odom-> pose.pose.position.y;


	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);

	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	velocityTheta = angles::normalize_angle(yaw + M_PI / 2.0l);
	updatePosition(robotID);
	ros::spinOnce();
}

float OverLord::getDistanceTo(const int fromID, const int toID) const
{
    return ((env == NULL) ? Vector() : env->getRelationship(toID, fromID)).magnitude();
}

float OverLord::getAngleTo(const int fromID, const int toID) const
{
    return ((env == NULL) ? Vector() : env->getRelationship(toID, fromID)).angle();
}


} /* namespace std */
