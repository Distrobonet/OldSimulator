//
// Filename:        "Cell.cpp"
//
// Programmer:      Ross Mead
// Last modified:   13Apr2011
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

//Cell::Cell(const char ID) : State(), Neighborhood()
//{
//	const float dx = 0.0f, dy = 0.0f, dz = 0.0f, theta = 0.0f;
//	init(dx, dy, dz, theta, ID);
//
//}

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
	ID = cellID;
	return true;
}

// Initializes the neighborhood of each cell,
// returning true if successful, false otherwise.
bool Cell::initNbrs(int currentRobotsID)
{
	Cell *c = this;
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

		if ((currentRobotsID >= 0) && (c->addNbr(leftNbrID)))
		{
			c->leftNbr  = c->nbrWithID(leftNbrID);
			c->leftNeighborState = stateNode.subscribe(generateSubMessage(leftNbrID), 1000, &Cell::stateCallback, &*c);
		}

		if ((currentRobotsID < nCells) && (c->addNbr(rightNbrID)))
		{
			c->rightNbr = c->nbrWithID(rightNbrID);
			c->rightNeighborState = stateNode.subscribe(generateSubMessage(rightNbrID), 1000, &Cell::stateCallback, &*c);
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

//// Attempts to set the robot to the parameterized robot,
//// returning true if successful, false otherwise.
//bool Cell::setRobot(const Robot &r)
//{
//	//if(VERBOSE) printf("in setRobot() ============\n");
//
//	//changed to  cast *this as a Robot variable
//	(Robot) *this = r;
//	//if(VERBOSE) printf("robot set============\n");
//	return true;
//} // setRobot(const Robot &)

//bool Cell::setRobotP(Robot *r)
//{
//	//(Robot *)this = r;
//	return true;
//}

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

//// Returns the robot of this cell.
//Robot Cell::getRobot() const
//{
//	return (Robot) *this;
//}


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


/*void Cell::updateState()
 {
 Neighbor currNbr;
 for (int i = 0; i < getNNbrs(); i++)
 {
 if (!getHead(currNbr)) break;

 // change formation if a neighbor has changed formation
 if (getNbr(0)->formation.getFormationID() > formation.getFormationID())
 changeFormation(getNbr(0)->formation, *getNbr(0));
 getNbr(0)->relActual = getRelationship(currNbr.ID);
 ++(*this);
 }
 rels = getRelationships();

 // reference the neighbor with the smallest gradient
 // to establish correct position in formation
 Neighbor     *refNbr = nbrWithMinGradient();
 Relationship *nbrRel = relWithID(refNbr->rels, ID);
 if ((formation.getSeedID() != ID) && (refNbr != NULL) && (nbrRel != NULL))
 {

 // error (state) is based upon the accumulated error in the formation
 nbrRel->relDesired.rotateRelative(-refNbr->rotError);
 float theta = scaleDegrees(nbrRel->relActual.angle() -
 (-refNbr->relActual).angle());
 rotError      = scaleDegrees(theta + refNbr->rotError);
 transError    = nbrRel->relDesired - nbrRel->relActual +
 refNbr->transError;
 transError.rotateRelative(-theta);
 if (transError.norm() > threshold()) moveArc(transError);
 else
 if (abs(rotError) > angThreshold())
 moveArc(0.0, degreesToRadians(-rotError));
 //if (abs(scaleDegrees(refNbr->relActual.angle() -
 //                    refNbr->relDesired.angle())) > angThreshold())
 //orientTo(refNbr->relActual, refNbr->relDesired.angle());
 else moveStop();
 }
 else moveStop();
 }   // updateState()*/

/*void Cell::updateState()
 {
 if(VERBOSE) printf("entering updateState()\n");
 Neighbor currNbr;
 for (int i = 0; i < getNNbrs(); i++)
 {
 if (!getHead(currNbr)) break;

 // change formation if a neighbor has changed formation
 if (getNbr(0)->formation.getFormationID() > formation.getFormationID())
 changeFormation(getNbr(0)->formation, *getNbr(0));
 getNbr(0)->relActual = getRelationship(currNbr.ID);
 ++(*this);
 }
 rels = getRelationships();

 // reference the neighbor with the smallest gradient
 // to establish correct position in formation
 if(VERBOSE) printf("updateState() -- BEFORE nbrWithMinGradient()\n");
 Neighbor     *refNbr = nbrWithMinGradient(formation.getSeedGradient());
 if(VERBOSE) printf("updateState() -- AFTER nbrWithMinGradient()\n");
 Relationship *nbrRel;
 if(refNbr == NULL){
 //printf("refNbr == NULL\n");
 }else{
 nbrRel = relWithID(refNbr->rels, ID);


 if(VERBOSE) printf("updateState() -- AFTER relWithID()\n");
 }
 if ((formation.getSeedID() != ID) && (refNbr != NULL) && (nbrRel != NULL))
 {

 // error (state) is based upon the accumulated error in the formation
 nbrRel->relDesired.rotateRelative(-refNbr->rotError);
 float theta = scaleDegrees(nbrRel->relActual.angle() -
 (-refNbr->relActual).angle());
 rotError      = scaleDegrees(theta + refNbr->rotError);
 transError    = nbrRel->relDesired - nbrRel->relActual +
 refNbr->transError;
 transError.rotateRelative(-theta);
 if (transError.norm() > threshold()) moveArc(transError);
 else
 if (abs(rotError) > angThreshold())
 moveArc(0.0, degreesToRadians(-rotError));
 //if (abs(scaleDegrees(refNbr->relActual.angle() -
 //                   refNbr->relDesired.angle())) > angThreshold())
 //orientTo(refNbr->relActual, refNbr->relDesired.angle());
 else moveStop();
 }
 else moveStop();
 if(VERBOSE) printf("leaving updateState()\n");
 }   // updateState()*/

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

	/*--ROSS--
	 float currDist        = 0.0f, closestDist = float(RAND_MAX);
	 int   closestNbrIndex = -1;
	 //int   closestRelIndex = -1;
	 cout << "myID = " << ID << endl;
	 cout << "+ nRels = " << r.getSize() << endl;
	 cout << "+ nNbrs = " << getNNbrs()  << endl;
	 vector<int> assignedIDs;
	 for (int i = 0; i < r.getSize(); i++)
	 {
	 closestDist     = float(RAND_MAX);
	 closestNbrIndex = -1;
	 Vector currRel;
	 if (r.getHead(currRel))
	 {
	 for (int j = 0; j < getNNbrs(); j++)
	 {
	 Neighbor currNbr;
	 if (getHead(currNbr))
	 {
	 currDist = (getNbr(0)->relDesired - r[0]).magnitude();
	 if (currDist < closestDist)
	 {
	 bool assignedID = false;
	 for (int k = 0; k < assignedIDs.getSize(); k++)
	 {
	 int currID = -1;
	 if ((assignedIDs.getHead(currID)) &&
	 (currID == getNbr(0)->ID))
	 {
	 assignedID = true;
	 break;
	 }
	 ++assignedIDs;
	 }
	 if (!assignedID)
	 {
	 closestDist     = currDist;
	 closestNbrIndex = j;
	 }
	 }
	 }
	 ++(*this);
	 }
	 if ((closestNbrIndex >= 0) && (closestNbrIndex < getNNbrs()))
	 {
	 assignedIDs.insertTail(getNbr(closestNbrIndex)->ID);
	 cout << "  -nbrID[" << i << "] = "
	 << getNbr(closestNbrIndex)->ID;
	 cout << " | rel = " << r[0] << endl;
	 getNbr(closestNbrIndex)->relDesired = r[0];
	 ++r;
	 }
	 else
	 {
	 cout << "   -nbrID[" << i << "] = " << ID_NO_NBR << endl;
	 r.removeHead();
	 }
	 }
	 else ++r;
	 }
	 --ROSS--*/
	/*--ROSS--
	 for (int i = 0; i < getNNbrs(); i++)
	 {
	 closestDist     = float(RAND_MAX);
	 closestRelIndex = -1;
	 if (getHead(currNbr))
	 {
	 for (int j = 0; j < r.getSize(); j++)
	 {
	 Vector currRel;
	 if (r.getHead(currRel))
	 {
	 currDist = (getNbr(0)->relDesired - currRel).magnitude();
	 if (currDist < closestDist)
	 {
	 closestDist     = currDist;
	 closestRelIndex = j;
	 }
	 }
	 ++r;
	 }
	 cout << "  -nbrID[" << i << "] = " << getNbr(0)->ID;
	 if ((closestRelIndex >= 0) && (closestRelIndex < r.getSize()))
	 {
	 cout << " | rel = " << r[closestRelIndex] << endl;
	 getNbr(0)->relDesired = r[closestRelIndex];
	 r.remove(closestRelIndex);
	 ++(*this);
	 }
	 else
	 {
	 cout << " | [DELETING]" << endl;
	 removeHead();
	 }
	 }
	 else ++(*this);
	 }
	 //setNbrs(nh);
	 --ROSS--*/

	if (leftNbr != NULL)
		leftNbr->relDesired = r[LEFT_NBR_INDEX];

	if (rightNbr != NULL)
		rightNbr->relDesired = r[RIGHT_NBR_INDEX];

	return true;
}

//// Attempts to broadcast the state of the cell
//// to the neighborhood of the cell, returning
//// true if successful, false otherwise.
//bool Cell::sendStateToNbrs() {
//	Neighbor curr;
//	if (VERBOSE)
//		cout << "cellID=%d\n", getID();
//
//	for (int i = 0; i < getNNbrs(); i++) {
//		if (VERBOSE)
//			cout << "sending state to id= %d\n", getNbr(i)->ID;
//
//		//if((getNbr(i)->ID)==NULL
//		//if(VERBOSE)printf("
//
//		if (!sendState(getNbr(i)->ID)) {
//			//printf("sendState returned false\n");
//			return false;
//		}
//	}
//	//printf("leaving sendStateToNbrs()\n");
//	return true;
//}


//// Attempts to send the state of the cell
//// to the neighbor with the parameterized ID,
//// returning true if successful, false otherwise.
//bool Cell::sendState(const int toID) {
//	//TODO: Publish State Here
//	//printf("in sendState()\n");
//	State *s = new State(*this);
//	//printf("number of relations of %d is %d \n",toID,s->rels.getSize());
//	//printf("calling cell = %d\n", this->ID);
//	//printf("number of relations of %d is %d \n",this->ID,this->rels.getSize());
//	bool answer = sendMsg(s, toID, STATE);
//	//printf("leaving sendState()\n");
//	return answer;
//}

// Attempts to process all packets received by the cell,
// returning true if successful, false otherwise.
//bool Cell::processPackets() {
//	bool success = true;
//	Packet p;
//	while (!msgQueue.empty()) {
//		p = msgQueue.front();
//		if (!processPacket(p))
//			success = false;
//		msgQueue.pop();
//	}
//	msgQueue.pop();
//	return success;
//}

// Attempts to process the parameterized packet,
// returning true if successful, false otherwise.
//bool Cell::processPacket(Packet &packet) {
//	bool success = false;
//	if ((packet.fromOperator()) && (packet.type == CHANGE_FORMATION)) {
//		success = changeFormation(*((Formation *) packet.msg));
//	}
////	else if (packet.type == AUCTION_ANNOUNCEMENT) {
////		if (ALLOW_CELL_BIDS) {
////
////		}
////		success = true;
////	} else if (packet.type == BID) {
////		if (packet.msg != NULL) {
////			bids.push_back((Bid*) packet.msg);
////			success = true;
////			numBids++;
////			//cout << "bid received, total = " << numBids << endl;
////		}
//		//PROP_MESSAGE may need to be some kind of prop super-enum that covers the base for all enum_props
//		//
////}
//	 else if (((isNbr(packet.fromID)) || (packet.fromBroadcast()))
//			&& !(packet.type == NCELL_REQUEST || packet.type == NCELL_RESPONSE
//					|| (packet.type == FCNTR_RESPONSE || packet.type == FCNTR_REQUEST)
//					|| (packet.type == FRAD_RESPONSE || packet.type == FRAD_REQUEST)
//					|| (packet.type == FSEED_RESPONSE || packet.type == FSEED_REQUEST))) {
//		switch (packet.type) {
//		case STATE:
//			success =
//					(packet.msg == NULL) ?
//							false : updateNbr(packet.fromID, *((State *) packet.msg));
//			delete (State *) packet.msg;
//			packet.msg = NULL;
//			break;
//		default:
//			break;
//		}
//	}
//	//receive a request, send out a request to nbrs that ref you
//	else if (packet.type == NCELL_REQUEST || packet.type == NCELL_RESPONSE) {
//		success = processNCell(packet);
//	}
//	//receiving a response back from nbrs that ref you
//	else if (packet.type == FCNTR_REQUEST || packet.type == FCNTR_RESPONSE) {
//		success = processFcntr(packet);
//	} else if (packet.type == FRAD_REQUEST || packet.type == FRAD_RESPONSE) {
//		success = processFRad(packet);
//	} else if (packet.type == FSEED_REQUEST || packet.type == FSEED_RESPONSE) {
//		success = processFSeed(packet);
//	}
//
//	return success;
//}

//bool Cell::processNCell(Packet &p) {
//	bool success = false;
//	if (p.type == NCELL_REQUEST) {
//		//If size is != 0, clear out the entire list and re-calculate.
//		//wonder if the messages are still floating around
//		//if information has been calculated previously, we need to erase it,
//		//and recalculate it
//		//this doesnt work for radius since the radius is computed only if NCELLS
//		//and FCNTR have been calculated
//		//
//		if (props.size() != 0) {
//			props.erase(props.begin(), props.end());
//			success = true;
//		}
//
//		bool is_ref_nbr = false;
//		Neighbor nbr;
//		//loop through nbrs
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//
//			if (nbr.refID == ID) //this nbr references you!
//					{
//				//send msg on to this nbr.
//				//make copy of p.msg, push it onto props, fwd on the message to this nbr
//
//				PropMsg *propm = new PropMsg();
//				propm->toID = nbr.ID;
//				propm->count = 0;
//				propm->response = false;
//
//				props.push_back(*propm); //here and not elsewhere
//				env->sendMsg(p.msg, nbr.ID, ID, NCELL_REQUEST);
//				is_ref_nbr = true;
//				success = true;
//			}
//			if (i == size() - 1 && is_ref_nbr == false) //no one has ref'd you.
//					{
//				//if this nbr doesnt ref you, and none have, you're at edge,
//				//send response msg with count +1;
//				// cout << "at and edge with ID: "
//				// << ID << " and my gradient is: " << gradient << endl;
//				PropMsg *prop = new PropMsg();
//				(*(PropMsg *) prop).count = 1;
//				env->sendMsg(prop, refID, ID, NCELL_RESPONSE);
//				success = true;
//			}
//		}
//
//	} else if (p.type == NCELL_RESPONSE) {
//
//		Neighbor nbr;
//		bool all_response = false;
//		// cycle through nbrs in your props list?
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//			//check if msg came from a nbr who references you
//			if (p.fromID == nbr.ID && nbr.refID == ID) {
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).toID == p.fromID) //msg came from a cell we sent to
//							{
//						props.at(j).response = true;
//						//following statement grabs the most recent "count" that's out there,
//						props.at(j).count = (*((PropMsg *) p.msg)).count; //FCntrMsg.gradient
//					}
//				}
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).response == false) {
//						all_response = false;
//						break;
//					}
//					all_response = true; //if you get here, you didnt break, all must be true
//				}
//
//				if (all_response) {
//					int sum = 1;
//					//cout << "gradient of " << ID << " is: " << gradient << endl;
//					for (unsigned int k = 0; k < props.size(); k++) {
//						sum += props.at(k).count;
//					}
//					if (refID == -1) //seed
//							{
//						cout << endl << endl;
//						cout << endl << endl;
//						cout << "Count of the robots is => " << sum << endl;
//						success = true;
//					} else {
//						PropMsg *prop = new PropMsg();
//						(*(PropMsg *) prop).count = sum; //FCntrMsg.gradient
//						env->sendMsg(prop, refID, ID, NCELL_RESPONSE);
//						success = true;
//					}
//				}
//			}
//		}
//	}
//	return success;
//}

//bool Cell::processFcntr(Packet &p) {
//	//If size is != 0, clear out the entire list and re-calculate.
//	bool success = false;
//	if (p.type == FCNTR_REQUEST) {
//		if (props.size() != 0) {
//			props.erase(props.begin(), props.end());
//			success = true;
//
//		}
//		bool is_ref_nbr = false;
//		Neighbor nbr;
//		//loop through nbrs
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//
//			if (nbr.refID == ID) //this nbr references you!
//					{
//				//send msg on to this nbr.
//				//make copy of p.msg, push it onto props, fwd on the message to this nbr
//				PropMsg *propm = new PropMsg();
//				propm->toID = nbr.ID;
//				propm->count = 0;
//				propm->response = false;
//
//				props.push_back(*propm); // a pointer? shouldnt it be a copy?
//				env->sendMsg(p.msg, nbr.ID, ID, FCNTR_REQUEST);
//				is_ref_nbr = true;
//				success = true;
//			}
//			if (i == size() - 1 && is_ref_nbr == false) //no one has ref'd you.
//					{
//				//if this nbr doesnt ref you, and none have, you're at edge,
//				//send response msg with count +1;
//				cout << "at and edge with ID: " << ID << " and my gradient is: "
//						<< gradient << endl;
//				PropMsg *prop = new PropMsg();
//				(*(PropMsg *) prop).count = 1;
//				Vector transErrorCopy = transError;
//				transErrorCopy.rotateRelative(
//						rotError + formation.getHeading());
//				(*(PropMsg *) prop).gradient = gradient - transErrorCopy;
//				env->sendMsg(prop, refID, ID, FCNTR_RESPONSE);
//				success = true;
//			}
//		}
//	} else if (p.type == FCNTR_RESPONSE) {
//
//		Neighbor nbr;
//		bool all_response = false;
//		//maybe cycle through nbrs in your props list?
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//			//if msg came from a nbr who references you
//			if (p.fromID == nbr.ID && nbr.refID == ID) {
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).toID == p.fromID) //msg came from a cell we sent to
//							{
//						props.at(j).response = true;
//						//grab p.msg's count, set it to that vector position's count
//						//for later summing up, only need to do this if props.at(j).count < 1.
//						//if(props.at(j).count <= 1)
//						//{
//						/*cout << "current p.msg count: " << (*((PropMsg *)p.msg)).count
//						 << " and position " << (*((PropMsg *)p.msg)).gradient
//						 << " for cell: " << ID << endl;*/
//						props.at(j).count = (*((PropMsg *) p.msg)).count;
//						props.at(j).gradient = (*((PropMsg *) p.msg)).gradient;
//						// }
//					}
//				}
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).response == false) {
//						all_response = false;
//						break;
//					}
//					all_response = true; //if you get here, you didnt break, all must be true
//				}
//
//				if (all_response) {
//					int sum = 1;
//					Vector transErrorCopy = transError;
//					transErrorCopy.rotateRelative(
//							rotError + formation.getHeading());
//					Vector gradSum = gradient - transErrorCopy;
//					cout << "gradient of " << ID << " is: " << gradient << endl;
//					for (unsigned int k = 0; k < props.size(); k++) {
//						sum += props.at(k).count;
//						gradSum += props.at(k).gradient;
//					}
//					if (refID == -1) //seed
//							{
//						cout << endl << endl;
//						cout << endl << endl;
//						cout << "sum of gradients is => " << gradSum << endl;
//						cout << "Count of the robots is => " << sum << endl;
//						//gradSum / sum for the center of mass? needs overloaded in vector
//						Vector mid = gradSum * (1.0f / (float) sum);
//						cout << "Middle of automaton is => " << mid << endl;
//						//cout <<"Magnitude is: " << mid.magnitude() << endl;
//						//draw x at location
//						env->getCentroid(mid);
//						success = true;
//					} else {
//						PropMsg *prop = new PropMsg();
//						(*(PropMsg *) prop).count = sum; //FCntrMsg.gradient
//						(*(PropMsg *) prop).gradient = gradSum; //FCntrMsg.gradient
//						env->sendMsg(prop, refID, ID, FCNTR_RESPONSE);
//						success = true;
//					}
//				}
//			}
//		}
//	}
//	return success;
//}
//
//bool Cell::processFRad(Packet &p) {
//	//If size is != 0, clear out the entire list and re-calculate.
//	bool success = false;
//	if (p.type == FRAD_REQUEST) {
//		bool is_ref_nbr = false;
//		Neighbor nbr;
//		//loop through nbrs
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//
//			if (nbr.refID == ID) //this nbr references you!
//					{
//				//send msg on to this nbr.
//				//make copy of p.msg, push it onto props, fwd on the message to this nbr
//
//				PropMsg *propm = new PropMsg();
//				propm->toID = nbr.ID;
//				propm->count = 0;
//				propm->response = false;
//
//				props.push_back(*propm); // a pointer? shouldnt it be a copy?
//				env->sendMsg(p.msg, nbr.ID, ID, FRAD_REQUEST);
//				is_ref_nbr = true;
//				success = true;
//			}
//			if (i == size() - 1 && is_ref_nbr == false) //no one has ref'd you.
//					{
//				//if this nbr doesnt ref you, and none have, you're at edge,
//				//send response msg with count +1;
//				PropMsg *prop = new PropMsg();
//				Vector transErrorCopy = transError;
//				transErrorCopy.rotateRelative(
//						rotError + formation.getHeading());
//				(*(PropMsg *) prop).radius = ((gradient - transErrorCopy)
//						- env->centroid).magnitude(); //calculation;
//				env->sendMsg(prop, refID, ID, FRAD_RESPONSE);
//				success = true;
//			}
//		}
//	} else if (p.type == FRAD_RESPONSE) {
//
//		Neighbor nbr;
//		bool all_response = false;
//		// cycle through nbrs in your props list?
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//			//if msg came from a nbr who references you
//			if (p.fromID == nbr.ID && nbr.refID == ID) {
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).toID == p.fromID) //msg came from a cell we sent to
//							{
//						props.at(j).response = true;
//						props.at(j).radius = (*((PropMsg *) p.msg)).radius;
//					}
//				}
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).response == false) {
//						all_response = false;
//						break;
//					}
//					all_response = true; //if you get here, you didnt break, all must be true
//				}
//
//				if (all_response) {
//					//cout << "gradient of " << ID << " is: " << gradient << endl;
//					Vector transErrorCopy = transError;
//					transErrorCopy.rotateRelative(
//							rotError + formation.getHeading());
//					float maxRadius = ((gradient - transErrorCopy)
//							- env->centroid).magnitude();
//					for (unsigned int k = 0; k < props.size(); k++) {
//						//compute max radius between mine and the best in my list
//						if (maxRadius < props.at(k).radius) {
//							maxRadius = props.at(k).radius;
//						}
//					}
//					if (refID == -1) //seed
//							{
//						cout << "Radius = " << maxRadius << endl;
//						env->getRadius(maxRadius);
//						success = true;
//					} else {
//						PropMsg *prop = new PropMsg();
//						(*(PropMsg *) prop).radius = maxRadius; //FCntrMsg.gradient
//						env->sendMsg(prop, refID, ID, FRAD_RESPONSE);
//						success = true;
//					}
//				}
//			}
//		}
//	}
//	return success;
//}
//
//bool Cell::processFSeed(Packet &p) {
//	//If size is != 0, clear out the entire list and re-calculate.
//	bool success = false;
//	if (p.type == FSEED_REQUEST) {
//		bool is_ref_nbr = false;
//		Neighbor nbr;
//		//loop through nbrs
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//
//			if (nbr.refID == ID) //this nbr references you!
//					{
//				//send msg on to this nbr.
//				//make copy of p.msg, push it onto props, fwd on the message to this nbr
//
//				PropMsg *propm = new PropMsg();
//				propm->toID = nbr.ID;
//				propm->count = 0;
//				propm->response = false;
//
//				props.push_back(*propm); // a pointer? shouldnt it be a copy?
//				env->sendMsg(p.msg, nbr.ID, ID, FSEED_REQUEST);
//				is_ref_nbr = true;
//				success = true;
//			}
//			if (i == size() - 1 && is_ref_nbr == false) //no one has ref'd you.
//					{
//				//if this nbr doesnt ref you, and none have, you're at edge,
//				//send response msg with count +1;
//				PropMsg *prop = new PropMsg();
//				//may hve to do the same trick with error as i did with radius
//				//Vector transErrorCopy = transError;
//				//transErrorCopy.rotateRelative(rotError + formation.getHeading());
//				(*(PropMsg *) prop).distance = (gradient - env->centroid);
//				env->sendMsg(prop, refID, ID, FSEED_RESPONSE);
//				success = true;
//			}
//		}
//	} else if (p.type == FSEED_RESPONSE) {
//
//		Neighbor nbr;
//		bool all_response = false;
//		//maybe cycle through nbrs in your props list?
//		for (int i = 0; i < getNNbrs(); i++) {
//			nbr = at(i);
//			//if msg came from a nbr who references you
//			if (p.fromID == nbr.ID && nbr.refID == ID) {
//				for (unsigned int j = 0; j < props.size(); j++) {
//					if (props.at(j).toID == p.fromID) //msg came from a cell we sent to
//							{
//						props.at(j).response = true;
//						props.at(j).distance = (*((PropMsg *) p.msg)).distance;
//					}
//				}
//				for (unsigned int j = 0; j < props.size(); j++)
//				{
//					if (props.at(j).response == false)
//					{
//						all_response = false;
//						break;
//					}
//					all_response = true; //if you get here, you didnt break, all must be true
//				}
//
//				if (all_response) {
//					//cout << "gradient of " << ID << " is: " << gradient << endl;
//					//Vector transErrorCopy = transError;
//					//transErrorCopy.rotateRelative(rotError + formation.getHeading());
//					// float maxRadius =  ((gradient-transErrorCopy)-env->centroid).magnitude();
//					float minDist = (gradient - env->centroid).magnitude();
//					Vector minVector = (gradient - env->centroid);
//					for (unsigned int k = 0; k < props.size(); k++)
//					{
//						//compute max radius between mine and the best in my list
//						if (minDist > (props.at(k).distance).magnitude())
//						{
//							minDist = (props.at(k).distance).magnitude();
//							minVector = props.at(k).distance;
//						}
//					}
//					if (refID == -1) //seed
//							{
//						cout << "Distance = " << minDist << endl;
//						cout << "Cell at " << (env->centroid) + minVector
//								<< " should be seed." << endl;
//						env->getDistance(env->centroid + minVector);
//						success = true;
//					}
//
//					else
//					{
//						PropMsg *prop = new PropMsg();
//						(*(PropMsg *) prop).distance = minVector; //FCntrMsg.gradient
//						env->sendMsg(prop, refID, ID, FSEED_RESPONSE);
//						success = true;
//					}
//				}
//			}
//		}
//	}
//	return success;
//}

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
 		res.state.relationships[i].actual.x = rels[i].relActual.x;
 		res.state.relationships[i].desired.x = rels[i].relDesired.x;
 		res.state.relationships[i].desired.x = rels[i].relDesired.x;
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

//void Cell::settleAuction() {
//	//cout << "Cell::settleAuction() entered\n"<< endl;
//	auctionStepCount = 0;
//	if (bids.size() <= 0) {
//		return;
//	}
//	Bid* winningBid;
//	winningBid = bids[0];
//	for (int i = 0; i < bids.size(); i++) {
//		if (bids[i]->b_i < winningBid->b_i) {
//			winningBid = bids[i];
//		}
//	}
//	//cout <<"Robot # "<<winningBid->rID<<" won the auction" << endl;
//	env->settleAuction(this, winningBid->rID);
//	bids.clear();
//}

//int Cell::getNBids() const {
//	return bids.size();
//}
//
//int Cell::getAuctionStepCount() const {
//	return auctionStepCount;
//}
//
//bool Cell::setAuctionStepCount(const int& asc) {
//	auctionStepCount = asc;
//	return true;
//}
