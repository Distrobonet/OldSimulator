#include<Simulator/Environment.h>

int main(bool doSpin)
{
	//TODO: this needs to be finished
	//ros node handle and init
	Cell thisCell;

	// TODO: initialize x & y from the "base_pose_ground_truth"?
	thisCell.x = 0.0f;
	thisCell.y = 0.0f;
	thisCell.setHeading(90.0f);	// default heading

	thisCell.update(doSpin);
	return 0;
}
