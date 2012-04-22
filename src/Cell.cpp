//
// Filename:        "Cell.cpp"
//
// Description:     This class implements a robot cell.
//

#define VERBOSE            (0)
#define AUTONOMOUS_INIT    (1)
#define ALLOW_CELL_BIDS    (0)
#define CELL_INFO_VIEW     (0)
#define AUCTION_STEP_COUNT (3)

// created for future
#define RADIUS			   (1.0)
#define Z_VALUE 		   (0)

#include<iostream>
// preprocessor directives
#include <stdlib.h>
#include <stdio.h>
#include <Simulator/Cell.h>
#include <Simulator/Environment.h>

using namespace std;

// <protected static data members>
int Cell::nCells = 0;

// Default constructor that initializes
// this cell to the parameterized values.
Cell::Cell(const int cellID) :
		State(), Neighborhood()
{
	cout << "Robot ID (Cell()): " << cellID << endl;
	init(cellID);
}

// Copy constructor that copies the contents of
// the parameterized cell into this cell.
Cell::Cell(const Cell &c) :	State(c), Neighborhood(c)
{
	leftNbr = c.leftNbr;
	rightNbr = c.rightNbr;
}

// Destructor that clears this cell.
Cell::~Cell() {}

// Updates the cell
void Cell::update(bool doSpin)
{
	while(ros::ok())
	{
		// Update the formation according to this cell
		//ROS_INFO("Trying to access the formation");



		//TODO: update the cell
		//cell.spin();
		Simulator::StateMessage state;
	  	state.formation.radius = formation.radius;
	  	state.formation.heading = formation.heading;
	  	state.formation.seed_frp.x = formation.seedFrp.x;
	  	state.formation.seed_frp.y = formation.seedFrp.y;
	  	state.formation.seed_id = formation.seedID;
	  	state.formation.formation_id = formation.formationID;
		state.frp.x = frp.x;
	 	state.frp.y = frp.y;

	 	for(uint i = 0; i < rels.size(); i++)
	 	{
	 		state.relationships[i].id = rels[i].ID;
	 		state.relationships[i].actual.x = rels[i].relActual.x;
	 		state.relationships[i].actual.x = rels[i].relActual.x;
	 		state.relationships[i].desired.x = rels[i].relDesired.x;
	 		state.relationships[i].desired.x = rels[i].relDesired.x;
	 	}

		state.linear_error.x = transError.x;
		state.linear_error.y = transError.y;
		state.angular_error = rotError;
		state.timestep = tStep;
		state.reference_id = refID;
		state.temperature = temperature;
		state.heat = heat;
		state_pub.publish(state);

		commandVelocity.linear.x = behavior.getTransVel();
		commandVelocity.angular.z = behavior.getRotVel();
		cmd_velPub.publish(commandVelocity);



		if(doSpin)
			ros::spinOnce();


		setFormationFromService();
		//cout << "Formation ID according to cell " << index << ": " << formation.formationID << endl;

	}
}


// Initializes the cell to the parameterized values,
// returning true if successful, false otherwise.
bool Cell::init(const int cellID)
{
	leftNbr = rightNbr = NULL;
	ID  			   = cellID;
	behavior           = DEFAULT_ROBOT_BEHAVIOR;
	return true;
}

// Initializes the neighborhood of each cell,
// returning true if successful, false otherwise.
bool Cell::initNbrs(Cell *cell, int currentRobotsID)
{
	string cellSubName;
	std::stringstream converter;

		int leftNbrID, rightNbrID;

		switch (currentRobotsID) {
			case 0:
				leftNbrID = currentRobotsID + 1;
				rightNbrID = currentRobotsID + 2;
				break;
			case 1:
				leftNbrID = currentRobotsID + 2;
				rightNbrID = currentRobotsID - 1;
				break;
			case 2:
				leftNbrID = currentRobotsID - 1;
				rightNbrID = currentRobotsID + 2;
				break;
			default:
				leftNbrID = currentRobotsID + 2;
				rightNbrID = currentRobotsID - 2;
				break;
		}

		if ((currentRobotsID >= 0) && (cell->addNbr(leftNbrID)))
		{
			cell->leftNbr  = cell->nbrWithID(leftNbrID);

			cellSubName = "robot_";
			converter << leftNbrID;
			cellSubName.append(converter.str());
			cellSubName.append("/state");
			cell->leftNeighborState = stateNode.subscribe(cellSubName, 1000, &Cell::stateCallback, cell);

		}

		if ((currentRobotsID < nCells) && (cell->addNbr(rightNbrID)))
		{
			cell->rightNbr = cell->nbrWithID(currentRobotsID + rightNbrID);
			cellSubName = "robot_";
			converter << leftNbrID;
			cellSubName.append(converter.str());
			cellSubName.append("/state");
			cell->rightNeighborState = stateNode.subscribe(cellSubName, 1000, &Cell::stateCallback, cell);

		}

	if(VERBOSE)
		printf("finished initNbrs()\n");
	return true;
}


// create correct string for state subscriber
string Cell::generateSubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/state";

	subString.insert(7, nbrID);
	return subString;
}

string Cell::generatePubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/cmd_vel";

	subString.insert(7, nbrID);
	return subString;
}


// Attempts to set the state to the parameterized state,
// returning true if successful, false otherwise.
bool Cell::setState(const State &s)
{
	*this = s;
	return true;
}

// Attempts to set the neighborhood to the parameterized neighborhood,
// returning true if successful, false otherwise.
bool Cell::setNbrs(Neighborhood &nh)
{
	*this = nh;
	return true;
}

// Returns the state of this cell.
State Cell::getState() const
{
	return (State) *this;
}

// Returns the neighborhood of this cell.
Neighborhood Cell::getNbrs() const
{
	return (Neighborhood) *this;
}


// Returns the ID of this robot.
int Cell::getID() const
{
    return ID;
}


// Sets the ID of this robot.
void Cell::setID(int cellID)
{
    ID = cellID;
}


// Updates the state of the cell based upon the
// current states of the neighbors of the cell.
void Cell::updateState()
{
	if ((getNNbrs() == 0) || (nbrWithMinStep()->tStep < tStep)
			|| ((formation.getSeedID() != ID)
					&& (nbrWithMaxStep()->tStep == tStep)))
		return;

	// update actual relationships to neighbors
	Neighbor *currNbr = NULL;
	for (unsigned int i = 0; i < size(); i++) {
		currNbr = getNbr(i);
		if (currNbr == NULL)
			break;

		// change formation if a neighbor has changed formation
		if (currNbr->formation.getFormationID() > formation.getFormationID())
			changeFormation(currNbr->formation, *currNbr);

		// TODO: Service call for reationship
		//currNbr->relActual = getRelationship(ID);
	}
	rels = getRelationships();

	// update temperature
	float m = 1.0f; // mass
	//float C    = 2.11f;  // specific heat
	//float K    = 2.0f;  // thermal conductivity (of material)
	//float A    = PI * getRadius();  // area of contact
	float C = 1.0f; // specific heat
	float K = 1.0f; // thermal conductivity (of material)
	float A = 1.0f; // area of contact
	float dT = 0.0f; // difference in temperature
	float d = 0.0f; // distance
	float q = 0.0f; // heat
	float qSum = 0.0f; // accumulated heat
	for (unsigned int i = 0; i < size(); i++) {
		currNbr = getNbr(i);
		if (currNbr == NULL)
			break;

		dT = currNbr->temperature - temperature;
		d = currNbr->relActual.magnitude();
		q = K * A * dT / d;
		qSum += q;
	}
	// FACTOR IN ROOM TEMPERATURE!!!
	//temperature = ((qSum - heat) + m * C * temperature) / (m * C);
	temperature = (qSum + m * C * temperature) / (m * C);
	heat = qSum;
	//printf("T[%d] = %.4f\n", ID, temperature);

	// reference the neighbor with the minimum gradient
	// to establish the correct position in formation
	if (getNNbrs() > 0) {
		Neighbor *refNbr = nbrWithMinFrp(formation.getSeedFrp());
		Relationship *nbrRelToMe = relWithID(refNbr->rels, ID);
		if ((formation.getSeedID() != ID) && (refNbr != NULL)
				&& (nbrRelToMe != NULL)) {

			// error (state) is based upon the
			// accumulated error in the formation
			Vector nbrRelToMeDesired = nbrRelToMe->relDesired;
			nbrRelToMeDesired.rotateRelative(-refNbr->rotError);
			float theta = scaleDegrees(
					nbrRelToMe->relActual.angle()
							- (-refNbr->relActual).angle());
			rotError = scaleDegrees(theta + refNbr->rotError);
			transError = nbrRelToMeDesired - nbrRelToMe->relActual
					+ refNbr->transError;
			transError.rotateRelative(-theta);
			//set the state variable of refID  = ID of the reference nbr.
			refID = refNbr->ID;
		}
	}

	tStep = max(tStep + 1, nbrWithMaxStep()->tStep);
}


void Cell::stateCallback(const Simulator::StateMessage &state)
{
	// TODO: to be fleshed out when after a publisher is defined
}

// Attempts to change the formation of the cell,
// returning true if successful, false otherwise.
bool Cell::changeFormation(const Formation &f, Neighbor n)
{
	formation = f;

	if (formation.getSeedID() == ID) {
		frp = formation.getSeedFrp();
		transError = Vector();
		rotError = 0.0f;
	}

	else {
		Relationship *nbrRelToMe = relWithID(n.rels, ID);
		if (nbrRelToMe == NULL)
			return false;

		nbrRelToMe->relDesired.rotateRelative(n.formation.getHeading());
		frp = n.frp + nbrRelToMe->relDesired;
		transError = Vector();
		rotError = 0.0f;
	}

	vector<Vector> r = formation.getRelationships(frp);


	if (leftNbr != NULL)
		leftNbr->relDesired = r[LEFT_NBR_INDEX];

	if (rightNbr != NULL)
		rightNbr->relDesired = r[RIGHT_NBR_INDEX];

	return true;
}

// Moves the robot cell using the current translational and
// rotational errors, activating and returning the appropriate
// robot behavior.
Behavior Cell::moveError()
{
	return moveErrorBehavior(transError, rotError);
}

// Moves the robot cell using the parameterized translational and
// rotational errors, activating and returning the appropriate
// robot behavior.
Behavior Cell::moveError(const Vector tError, const float rError)
{
	return moveErrorBehavior(tError, rError);
}

// Moves the robot using the parameterized translational and
// rotational errors, returning the appropriate robot behavior.
Behavior Cell::moveErrorBehavior(const Vector tError, const float rError)
{
	if (transError.magnitude() > threshold())
		return move(transError);

	else if (abs(rotError) > angThreshold())
		return move(0.0, degreesToRadians(-rotError));

	return moveStop();
}

// Moves the robot using the parameterized movement vector,
// returning the appropriate robot behavior.
Behavior Cell::move(const Vector &target)
{
	// TODO: Need call to cmd_vel subscriber
	float theta    		= target.angle();
	float phi     		= this->heading.angle();
	float delta   		= degreesToRadians(theta);
	float cosDelta 		= cos(delta);
	float sinDelta 		= sin(delta);
	float t       		= cosDelta * cosDelta * sign(cosDelta);
	float r       		= sinDelta * sinDelta * sign(sinDelta);
	Behavior behavior	= Behavior(t, r, maxSpeed());

	if (abs(theta) < 90.0f)
	      behavior.setDiffVel(maxSpeed() * (t + r), maxSpeed() * (t - r));
    else
        behavior.setDiffVel(maxSpeed() * (t - r), maxSpeed() * (t + r));

	return behavior;
/*
    float r     = target.magnitude();
    if (r <= threshold()) return moveStop();
    float theta = degreesToRadians(target.angle());
    if (theta == 0.0f)    return moveForwardBehavior(r);
    else return moveArcBehavior((abs(theta) >
                                degreesToRadians(angThreshold())) ?
                                0.0f :
                                r * velocityTheta / sin(theta), getDiameter() * theta);
*/
}


// Returns the minimum angular movement threshold of this robot.
float Cell::angThreshold() const
{
    return 0.5f * FACTOR_THRESHOLD * maxAngSpeed();
}


// Returns the minimum movement threshold of this robot.
float Cell::threshold() const
{
    return FACTOR_THRESHOLD * maxSpeed();
}


// Returns the max angular speed of this robot.
float Cell::maxAngSpeed() const
{
    return radiansToDegrees(maxSpeed() / RADIUS);
}


// Stops the robot from moving,
// returning the appropriate robot behavior.
Behavior Cell::moveStop()
{
    return move(0.0f, 0.0f);
}


// Moves the robot using the parameterized translational
// and rotational velocities, returning the appropriate robot behavior.
Behavior Cell::move(const float t, const float r)
{
    return Behavior(t, r, maxSpeed());
}


// Returns the max speed of this robot.
float Cell::maxSpeed() const
{
    return FACTOR_MAX_SPEED * RADIUS;
}

// Attempts to set the heading to the parameterized heading,
// returning true if successful, false otherwise.
bool Cell::setHeading(const float theta)
{
    return heading.setPolar(RADIUS + VECTOR_HEAD_HEIGHT, theta, Z_VALUE);
}


// Copies the contents of the parameterized state into this cell.
Cell& Cell::operator =(const State &s) {
	return *this = s;
}

// Copies the contents of the parameterized neighborhood into this cell.
Cell& Cell::operator =(const Neighborhood &nh) {
	return *this = nh;
}


// Sets this cell's formation object from the response from the Formation service
bool Cell::setFormationFromService()
{
	//ROS_INFO("Trying to access the formation message");
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	ros::NodeHandle clientNode;
	formationClient = clientNode.serviceClient<Simulator::CurrentFormation>("formation");

	if (formationClient.call(formationSrv))
	{
		formation.formationID = formationSrv.response.formation.formation_id;
		formation.heading = formationSrv.response.formation.heading;
		formation.radius = formationSrv.response.formation.radius;
		formation.seedFrp.x = formationSrv.response.formation.seed_frp.x;
		formation.seedFrp.y = formationSrv.response.formation.seed_frp.y;
		formation.seedID = formationSrv.response.formation.seed_id;
		clientNode.shutdown();
		spinner.stop();
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service formation");
		clientNode.shutdown();
		spinner.stop();
		return false;
	}
}

// Get a neighbor's state from the State service//TODO: do this function
bool Cell::getNeighborState(bool leftOrRight)
{
	//TODO: do this function
	return false;
}

// Starts the cell's state service server
void Cell::startStateServiceServer()
{
	ros::NodeHandle StateServerNode;
	string name = "cell_state_";
	name = name + boost::lexical_cast<std::string>(ID);	// add the index to the name string

	stateService = StateServerNode.advertiseService(name, &Cell::setStateMessage, this);
	//cout << "Now serving the " << stateService.getService() << " service!\n";

	ros::spinOnce();

	StateServerNode.shutdown();
}

// Sets the state message to this state's info.  This is the callback for the state service.
bool Cell::setStateMessage(Simulator::State::Request  &req, Simulator::State::Response &res )
{
  	res.state.formation.radius = formation.radius;
  	res.state.formation.heading = formation.heading;
  	res.state.formation.seed_frp.x = formation.seedFrp.x;
  	res.state.formation.seed_frp.y = formation.seedFrp.y;
  	res.state.formation.seed_id = formation.seedID;
  	res.state.formation.formation_id = formation.formationID;
	res.state.frp.x = frp.x;
 	res.state.frp.y = frp.y;

 	for(uint i = 0; i < rels.size(); i++)
 	{
 		res.state.relationships[i].id = rels[i].ID;
 		res.state.relationships[i].actual.x = rels[i].relActual.x;
 		res.state.relationships[i].actual.y = rels[i].relActual.x;
 		res.state.relationships[i].desired.x = rels[i].relDesired.x;
 		res.state.relationships[i].desired.y = rels[i].relDesired.x;
 	}

	res.state.linear_error.x = transError.x;
	res.state.linear_error.y = transError.y;
	res.state.angular_error = rotError;
	res.state.timestep = tStep;
	res.state.reference_id = refID;
	res.state.temperature = temperature;
	res.state.heat = heat;

	ROS_INFO("sending back response with state info");
	return true;
}

// Relationship client, sends its ID and a target ID as requests and gets a relationship vector response
bool Cell::getRelationship(int targetID)
{
	ROS_INFO("Trying to access the Relationship message");
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	ros::NodeHandle clientNode;
	relationshipClient = clientNode.serviceClient<Simulator::Relationship>("relationship");

	if (relationshipClient.call(relationshipSrv))
	{
		// TODO: relationship stuff in cell is done here

//		formation.formationID = formationSrv.response.formation.formation_id;

		clientNode.shutdown();
		spinner.stop();
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call relationship service");
		clientNode.shutdown();
		spinner.stop();
		return false;
	}
}
