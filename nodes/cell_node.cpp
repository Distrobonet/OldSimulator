#include<Simulator/Environment.h>

int main(int argc, char **argv)
{
    // Service stuff
	string ros_name;
	ros_name = "formation_client";
	ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

    ros_name = "state_client_";
    ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);



	//TODO: this needs to be finished
	//ros node handle and init
	Cell thisCell;
	thisCell.index = atoi(argv[1]);

	// TODO: initialize x & y from the "base_pose_ground_truth"?
	thisCell.x = 0.0f;
	thisCell.y = 0.0f;
	thisCell.setHeading(90.0f);	// default heading

	thisCell.update(argv[2]);


	// testing formation client for cell
	cout << thisCell.formation.formationID;
	thisCell.setFormationFromService();
	cout << thisCell.formation.formationID;


	return 0;
}
