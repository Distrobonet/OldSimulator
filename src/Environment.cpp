//
// Description:     This class implements a robot cell environment.
//

// preprocessor directives
#include <stdio.h>
#include <Simulator/Environment.h>
#include <time.h>
#include <ctime>
#include <stdlib.h>
#include <angles/angles.h>

#define AUCTION_STEP_COUNT (3)

Environment::Environment()
{}

// Default constructor that initializes
// this environment to the parameterized values.
Environment::Environment(int numRobots)
{
	int argc = 0;
	char **argv = 0;
	ros::init(argc, argv, "relationship_server");

	numOfRobots = numRobots;
	initOverloardSubscribers(this);
}

void Environment::initOverloardSubscribers(Environment *e)
{
	ros::NodeHandle overLord;

	vector<double> subRobot0Vel;
	subRobot0Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot0Vel.push_back(0);
	subRobot0Vel.push_back(0);
	subRobotVels.push_back(subRobot0Vel);

	vector<double> subRobot1Vel;
	subRobot1Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot1Vel.push_back(0);
	subRobot1Vel.push_back(0);
	subRobotVels.push_back(subRobot1Vel);

	vector<double> subRobot2Vel;
	subRobot2Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot2Vel.push_back(0);
	subRobot2Vel.push_back(0);
	subRobotVels.push_back(subRobot2Vel);

	vector<double> subRobot3Vel;
	subRobot3Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot3Vel.push_back(0);
	subRobot3Vel.push_back(0);
	subRobotVels.push_back(subRobot3Vel);

	vector<double> subRobot4Vel;
	subRobot4Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot4Vel.push_back(0);
	subRobot4Vel.push_back(0);
	subRobotVels.push_back(subRobot4Vel);

	vector<double> subRobot5Vel;
	subRobot5Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot5Vel.push_back(0);
	subRobot5Vel.push_back(0);
	subRobotVels.push_back(subRobot5Vel);

	vector<double> subRobot6Vel;
	subRobot6Vel.push_back(0); //Sets up 3 spots for the x,y,theta
	subRobot6Vel.push_back(0);
	subRobot6Vel.push_back(0);
	subRobotVels.push_back(subRobot6Vel);

	ros::Subscriber subRobot0 = overLord.subscribe("/robot_0/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot0);
	ros::Subscriber subRobot1 = overLord.subscribe("/robot_1/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot1);
	ros::Subscriber subRobot2 = overLord.subscribe("/robot_2/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot2);
	ros::Subscriber subRobot3 = overLord.subscribe("/robot_3/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot3);
	ros::Subscriber subRobot4 = overLord.subscribe("/robot_4/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot4);
	ros::Subscriber subRobot5 = overLord.subscribe("/robot_5/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot5);
	ros::Subscriber subRobot6 = overLord.subscribe("/robot_6/base_pose_ground_truth", 1000, &Environment::callBackRobot, e);
	subRobots.push_back(subRobot6);
}

void Environment::callBackRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	btScalar yaw = 0.0l;

	double currentY = odom-> pose.pose.position.x;
	double currentX = -odom-> pose.pose.position.y;
	double currentTheta = angles::normalize_angle(yaw + M_PI / 2.0l);
	string ID = odom->header.frame_id.substr(7,1);
	int IDNumber = atoi(ID.c_str());

	subRobotVels.at(IDNumber).at(0) = currentX;
	subRobotVels.at(IDNumber).at(1) = currentY;
	subRobotVels.at(IDNumber).at(2) = currentTheta;

	ros::spinOnce();
}

// Copy constructor that copies the contents of
// the parameterized environment into this environment.
  Environment::Environment(const Environment &env)
{}


// Destructor
Environment::~Environment()
{}


void Environment::update(bool doSpin)
{
	Cell *temp = NULL;
//	ros::init(argc, argv, "message_listener");
	ros::NodeHandle rosNode;
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// TODO update base_pose_gnd_truth
		//ROS handles this for us with the call back function

		ros::spinOnce();
	}
}


string Environment::generateSubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/state";

	subString.insert(7, nbrID);
	return subString;
}


// sets the response(relationship vector) based on the requests(IDs).  This is a callback.
bool Environment::setRelationshipMessage(Simulator::Relationship::Request  &req, Simulator::Relationship::Response &res )
{
//	Vector temp   = *toCell - *fromCell;
//	temp.rotateRelative(-fromCell->getHeading());

	//target - original
	Vector tempVector;
	tempVector.x = subRobotVels[req.OriginID][0] - subRobotVels[req.TargetID][0];
	tempVector.y = subRobotVels[req.OriginID][1] - subRobotVels[req.TargetID][1];

	//tempVector.rotateRelative(-subRobotVels[req.OriginID][0].getHeading());

	float theta = -subRobotVels[req.OriginID][3];
	theta = degreesToRadians(theta);
	//theta =
	//set(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta), z);
	tempVector.x = tempVector.x * cos(theta) - tempVector.y * sin(theta);
	tempVector.y = tempVector.x * sin(theta) + tempVector.y * cos(theta);

	res.theRelationship.actual.x = tempVector.x;
	res.theRelationship.actual.y = tempVector.y;
	return true;
}


// Starts the environment's relationship service server
void Environment::startRelationshipServiceServer()
{
	ros::NodeHandle RelationshipServerNode;

	relationshipService = RelationshipServerNode.advertiseService("relationship", &Environment::setRelationshipMessage, this);
	//cout << "Now serving the " << stateService.getService() << " service!\n";

	ros::spinOnce();

	RelationshipServerNode.shutdown();
}

