#include<Simulator/Environment.h>

int main(bool doSpin)
{
	//TODO: this needs to be finished
	//ros node handle and init
	Cell *c = new Cell;

	// TODO: initialize x & y from the "base_pose_ground_truth"?
	c->x = 0.0f;
	c->y = 0.0f;
	c->setHeading(90.0f);	// default heading

	c->update(doSpin);
	return 0;
}
