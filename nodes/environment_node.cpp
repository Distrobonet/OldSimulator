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


void envCallBack(const nav_msgs::OdometryConstPtr &odom)
{
  g_by = odom->pose.pose.position.x;
  g_bx = -odom->pose.pose.position.y;

  double roll = 0.0l;
  double pitch = 0.0l;
  double yaw = 0.0l;
  btQuaternion q(odom->pose.pose.orientation.x,
                 odom->pose.pose.orientation.y,
                 odom->pose.pose.orientation.z,
                 odom->pose.pose.orientation.w);
  btMatrix3x3(q).getRPY(roll, pitch, yaw);
  g_bth = angles::normalize_angle(yaw + M_PI / 2.0l);
}
