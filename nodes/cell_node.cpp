#include<Simulator/Cell.h>

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

    // sets dx, dy, dz = 0.0f, theta (heading = 90.0f[default], cellID
    cout << "Robot ID (cell_node): " << atoi(argv[1]) << endl;
    Cell *thisCell = new Cell(atoi(argv[2]), atoi(argv[3]), 0.0f, 90.0f, atoi(argv[1]));
	thisCell->index = atoi(argv[1]);

//	// TODO: initialize x & y from the "base_pose_ground_truth"?
//	thisCell->x = atoi(argv[2]);
//	thisCell->y = atoi(argv[3]);
//	thisCell->setHeading(90.0f);	// default heading
	thisCell->initNbrs(atoi(argv[1]));

	thisCell->update(argv[4]);


	return 0;
}
