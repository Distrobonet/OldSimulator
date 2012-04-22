#include<Simulator/Cell.h>

int main(int argc, char **argv)
{
	string ros_name;

	// Formation Service
	ros_name = "formation_client_";
	ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

    // State Service
    ros_name = "state_client_";
    ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

    // State publisher
    ros::init(argc, argv, "state");
    ros::NodeHandle state_pub;


    // Initilize cell
    ros::Publisher chatter_pub = state_pub.advertise<std_msgs::String>("state", 1000);
    Cell thisCell = Cell(atoi(argv[1]));
	thisCell.setID(atoi(argv[1]));
	thisCell.initNbrs(&thisCell, atoi(argv[1]));

	// Start the state service server for this cell
	ros_name = "state_server_";
	ros_name += thisCell.getID();
	ros::init(argc, argv, ros_name);
	thisCell.startStateServiceServer();

	// Update this cell
	thisCell.update(argv[4]);


	return 0;
}
