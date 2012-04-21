#include<Simulator/Environment.h>

int main(int argc, char **argv)
{
	// TODO: this needs to be fixed
	// initialize ROS stuff
	ros::init(argc, argv, "environment");
	ros::NodeHandle base_pose;

	//TODO: create base_poseCallback
	//ros::Subscriber sub = base_pose.subscribe("chatter", 1000, base_poseCallback);

	int numOfRobots = atoi(argv[1]);
	Environment env(numOfRobots);
	env.update(argv[2]);
	return 0;
}
