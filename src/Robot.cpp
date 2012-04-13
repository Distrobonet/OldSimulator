//
// This class implements a 2-dimensional robot.
//

// preprocessor directives
#include <Simulator/Environment.h>
#include <Simulator/Robot.h>
#define SUBSCRIBER 0
#define PUBLISHER 1
#define ROBOT_LABEL -1

// <protected static data members>
int Robot::numOfRobots = ID_ROBOT;   // initializes the number of robots to 0
double velocityX, velocityY, velocityTheta;



static void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	int robotID = atoi(odom->header.frame_id.c_str());

	velocityY = odom-> pose.pose.position.x;
	velocityX = -odom-> pose.pose.position.y;

	//TODO: can't seem to resolve this
	//updatePosition(robotID);

	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);

	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	velocityTheta = angles::normalize_angle(yaw + M_PI / 2.0l);
	ros::spinOnce();
}

void Robot::updatePosition(int robot)
{
	Robot *r = env->getRobot(robot);

	r->robotX = velocityX;
	r->robotY = velocityY;
	r->robotTheta = velocityTheta;
}

// Default constructor that initializes
// this robot to the parameterized values.
Robot::Robot(const float dx,    const float dy, const float dz,
             const float theta)
{
    init(dx, dy, dz, theta);
    ID = numOfRobots++;

}


// Initializes the robot to the parameterized values,
// returning true if successful, false otherwise.
bool Robot::init(const float dx, const float dy, const float dz, const float theta)
{
    Circle::init(dx, dy, dz, DEFAULT_ROBOT_RADIUS);
    setHeading(theta);
    behavior         = DEFAULT_ROBOT_BEHAVIOR;
    behavior.setMaxSpeed(maxSpeed());
    showHeading      = DEFAULT_ROBOT_SHOW_HEADING;
    heading.showLine = DEFAULT_ROBOT_SHOW_LINE;
    heading.showHead = DEFAULT_ROBOT_SHOW_HEAD;
    showFilled       = DEFAULT_ROBOT_SHOW_FILLED;

	ros::NodeHandle aNode;
	subRobot = aNode.subscribe(generateSubPubMessage(SUBSCRIBER), 1000, callBackRobot);
	robotX = velocityX;
	robotY = velocityY;
	robotTheta = velocityTheta;
	pub_cmd_vel = aNode.advertise < geometry_msgs::Twist > (generateSubPubMessage(PUBLISHER), 1);
	geometry_msgs::Twist commandVelocity;

	// odom message
	odomMsg.header.stamp = current_time = ros::Time::now();
	odomMsg.header.frame_id = generateSubPubMessage(ROBOT_LABEL);

    setEnvironment(NULL);
    return true;
}


// This method will generate the appropriate Subscriber/Publisher message for a new robot
// using the current number of robots + 1
string Robot::generateSubPubMessage(bool subOrPub)
{
	stringstream ss;//create a stringstream
	ss << (numOfRobots + 1);//add number to the stream
	string numRobots = ss.str();


	// Subscriber
	if(subOrPub == SUBSCRIBER)
	{
		string subString = "/robot_/base_pose_ground_truth";

		subString.insert(7, numRobots);
		return subString;

	}

	// Robot label
	else if(subOrPub == ROBOT_LABEL)
	{
		string subString = "";
		subString.insert(0, numRobots);
		return subString;
	}

	// Publisher
	else
	{
		string pubString = "/robot_/cmd_vel";
		pubString.insert(7, numRobots);
		return pubString;
	}
}


// Copy constructor that copies the contents of
// the parameterized robot into this robot.
Robot::Robot(const Robot &robot)
{
    init(robot.x, robot.y, robot.z, robot.getHeading());
    for (int i = 0; i < 3; ++i)
    {
        translate[i] = robot.translate[i];
        rotate[i]    = robot.rotate[i];
        scale[i]     = robot.scale[i];
    }
    showLine    = robot.showLine;
    showHead    = robot.showHead;
    showPos     = robot.showPos;
    showHeading = robot.showHeading;
    ID          = robot.ID;
    env         = robot.env;
    msgQueue    = robot.msgQueue;
}


// Destructor that clears this robot.
Robot::~Robot()
{
}


// Attempts to set the radius to the parameterized radius,
// returning true if successful, false otherwise.
bool Robot::setRadius(const float radis)
{
    if (!Circle::setRadius(radis)) return false;
    return behavior.setMaxSpeed(maxSpeed());
}


// Attempts to set the heading to the parameterized heading,
// returning true if successful, false otherwise.
bool Robot::setHeading(const float theta)
{
    return heading.setPolar(radius + VECTOR_HEAD_HEIGHT, theta);
}


// Attempts to set the environment to the parameterized environment,
// returning true if successful, false otherwise.
bool Robot::setEnvironment(Environment *enviroment)
{
    env = enviroment;
    return true;
}


// Translates the robot relative to itself based
// on the parameterized translation vector.
void Robot::translateRelative(Vector v)
{
    v.rotateRelative(getHeading());
    x += v.x;
    y += v.y;
}


// Translates the robot relative to itself based
// on the parameterized x-/y-coordinate translations.
void Robot::translateRelative(const float dx, const float dy)
{
    translateRelative(Vector(dx, dy));
}


// Rotates the robot about itself (in 2-dimensions)
// based on the parameterized rotation angle.
void Robot::rotateRelative(float theta)
{
    heading.rotateRelative(theta);
}


// Returns the environment of this robot.
Environment* Robot::getEnvironment() const
{
    return env;
}


// Returns the ID of this robot.
int Robot::getID() const
{
    return ID;
}


// Returns the heading of this robot.
float Robot::getHeading() const
{
    return heading.angle();
}


// Returns the translational velocity of this robot.
float Robot::getTransVel() const
{
    return behavior.getTransVel();
}


// Returns the rotational velocity of this robot.
float Robot::getRotVel() const
{
    return behavior.getRotVel();
}


// Returns the angular velocity of this robot.
float Robot::getAngVel() const
{
    return radiansToDegrees(getRotVel() / radius);
}


// Returns the velocity of this robot.
float Robot::getVelocity() const
{
    return behavior.getVelocity();
}


// Returns the arc radius based on the translational
// and rotational velocities this robot.
float Robot::getArcRadius() const
{
    return (behavior.getRotVel() == 0.0f) ? 0.0f :
            radius * behavior.getTransVel() / behavior.getRotVel();
}


// Renders the robot as a circle with a vector heading.
void Robot::draw()
{
    if ((color[GLUT_RED]   == COLOR[INVISIBLE][GLUT_RED])   &&
        (color[GLUT_GREEN] == COLOR[INVISIBLE][GLUT_GREEN]) &&
        (color[GLUT_BLUE]  == COLOR[INVISIBLE][GLUT_BLUE])) return;

    vector<Vector> relationships = getObjectRelationships(2.0f * collisionRadius());
    for (int i = 0, n = relationships.size(); i < n; ++i)
    {
        relationships[i].rotated(rotate[2]);
        relationships[i].translated(x, y, z);
        relationships[i].scaled(scale[0]);
        relationships[i].draw();
    }

    // draw a circle representing the robot
    Circle::draw();

    // draw a vector representing the robot heading
    if (showHeading)
    {
//        glPushMatrix();
//            glRotated(rotate[0], 0, 0, 1);
//            glRotated(rotate[1], 0, 0, 1);
//            glRotated(rotate[2], 0, 0, 1);
//            heading.translated(x + translate[0],
//                               y + translate[1],
//                               z + translate[2]);
//            heading.scaled(radius / DEFAULT_ROBOT_RADIUS);
//            heading.draw();
//        glPopMatrix();
    }
}


// Executes the appropriate active behavior.
void Robot::step()
{
    float collisionDistance = collisionRadius();
    vector<Vector> relationships = getObjectRelationships(collisionDistance);
    if (relationships.size() > 0)
    {
		float minDist  = HUGE;
		int   minIndex = 0;
		for (int i = 0, n = relationships.size(); i < n; ++i)
		{
			float currentDist = relationships[i].magnitude();
			if (currentDist < minDist)
			{
				minDist  = currentDist;
				minIndex = i;
			}
		}
		//avoid(rels[minIndex], collDist);
	}
	
    if (behavior.isActive())
    {
        translateRelative(getTransVel());
        rotateRelative(getAngVel());
    }
}


// Returns the relationship from this robot
// to the parameterized target vector.
Vector Robot::getRelationship(Vector &target) const
{
    Vector temp = target - *this;
    temp.rotateRelative(-getHeading());
    return temp;
}


// Returns the distance from this robot
// to the parameterized target vector.
float Robot::getDistanceTo(Vector &target) const
{
    return getRelationship(target).magnitude();
}


// Returns the angle from this robot
// to the parameterized target vector.
float Robot::getAngleTo(Vector &target) const
{
    return getRelationship(target).angle();
}


// Returns the max speed of this robot.
float Robot::maxSpeed() const
{
    return FACTOR_MAX_SPEED * radius;
}


// Returns the max angular speed of this robot.
float Robot::maxAngSpeed() const
{
    return radiansToDegrees(maxSpeed() / radius);
}


// Returns the minimum movement threshold of this robot.
float Robot::threshold() const
{
    return FACTOR_THRESHOLD * maxSpeed();
}


// Returns the minimum angular movement threshold of this robot.
float Robot::angThreshold() const
{
    return 0.5f * FACTOR_THRESHOLD * maxAngSpeed();
}


// Returns the minimum collision radius of this robot.
float Robot::collisionRadius() const
{
    return FACTOR_COLLISION_RADIUS * radius;
}


// Returns the relationships from this robot
// to all objects in the environment.
vector<Vector> Robot::getObjectRelationships(const float maxDist) const
{
    return (env == NULL) ? vector<Vector>()
                         : env->getObjectRelationships(ID, maxDist);
}


// Returns the relationship from this robot
// to the robot with the parameterized ID.
Vector Robot::getRelationship(const int toID) const
{
    return (env == NULL) ? Vector() : env->getRelationship(toID, ID);
}


// Returns the distance from this robot
// to the robot with the parameterized ID.
float Robot::getDistanceTo(const int toID) const
{
    return getRelationship(toID).magnitude();
}


// Returns the angle from this robot
// to the robot with the parameterized ID.
float Robot::getAngleTo(const int toID) const
{
    return getRelationship(toID).angle();
}


// Attempts to send a packet to its destination
// based upon the given parameters, returning
// true if successful, false otherwise.
bool Robot::sendMsg(const Message &msg, const int toID, const int type)
{
    if (sendPacket(Packet(msg, toID, ID, type))) return true;
    //delete msg;    // TO-DO: call destructor of appropriate type... ?
    return false;
}


// Attempts to send a packet to its destination,
// returning true if successful, false otherwise.
bool Robot::sendPacket(const Packet &packet)
{
    if ((env != NULL) && (env->sendPacket(packet))) return true;
    //delete p.msg;    // TO-DO: call destructor of appropriate type... ?
    return false;
}


// Moves the robot using the parameterized movement vector,
// activating and returning the appropriate robot behavior.
Behavior Robot::moveArc(const Vector &target)
{
    return behavior = moveArcBehavior(target);
}


// Moves the robot using the parameterized movement vector,
// returning the appropriate robot behavior.
Behavior Robot::moveArcBehavior(const Vector &target)
{
    float theta    = target.angle();
	float phi      = this->heading.angle();
	float delta    = degreesToRadians(theta);
    float cosDelta = cos(delta);
    float sinDelta = sin(delta);
    float t        = cosDelta * cosDelta * sign(cosDelta);
	float r        = sinDelta * sinDelta * sign(sinDelta);
	behavior         = Behavior(ACTIVE, t, r, maxSpeed());

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


// Moves the robot using the parameterized translational
// and rotational velocities, activating and returning
// the appropriate robot behavior.
Behavior Robot::moveArc(const float t, const float r, const Status s)
{
    return behavior = moveArcBehavior(t, r, s);
}


// Moves the robot using the parameterized translational
// and rotational velocities, returning the appropriate robot behavior.
Behavior Robot::moveArcBehavior(const float t,
                                const float r,
                                const Status  s)
{
    return Behavior(s, t, r, maxSpeed());
}


// Moves the robot forward using the parameterized robot velocity,
// activating and returning the appropriate robot behavior.
Behavior Robot::moveForward(const float speed)
{
    return behavior = moveForwardBehavior(speed);
}


// Moves the robot forward using the parameterized robot velocity,
// returning the appropriate robot behavior.
Behavior Robot::moveForwardBehavior(const float vel)
{
    return moveArcBehavior(vel, 0.0f);
}


// Moves the robot backward using the parameterized robot velocity,
// activating and returning the appropriate robot behavior.
Behavior Robot::moveBackward(const float vel)
{
    return behavior = moveForwardBehavior(vel);
}


// Moves the robot backward using the parameterized robot velocity,
// returning the appropriate robot behavior.
Behavior Robot::moveBackwardBehavior(const float vel)
{
    return moveArcBehavior(-vel, 0.0f);
}


// Stops the robot from moving, activating and
// returning the appropriate robot behavior.
Behavior Robot::moveStop()
{
    return behavior = moveStopBehavior();
}


// Stops the robot from moving,
// returning the appropriate robot behavior.
Behavior Robot::moveStopBehavior()
{
    return moveArcBehavior(0.0f, 0.0f, DONE);
}


// Rotates the robot to the parameterized heading
// relative to the paratermized target, activating and
// returning the appropriate robot behavior.
Behavior Robot::orientTo(const Vector &target, const float theta)
{
    return behavior = orientToBehavior(target, theta);
}


// Rotates the robot to the parameterized heading
// relative to the paratermized target,
// returning the appropriate robot behavior.
Behavior Robot::orientToBehavior(const Vector &target, const float theta)
{
    float delta   = scaleDegrees(target.angle() - theta);
    if (abs(delta) <= angThreshold()) return moveStopBehavior();
    return moveArcBehavior(0.0f, degreesToRadians(delta));
}


// Directs the robot to follow the parameterized target
// maintaining the parameterized distance, activating and
// returning the appropriate robot behavior.
Behavior Robot::follow(const Vector &target, const float dist)
{
    return behavior = followBehavior(target, dist);
}


// Directs the robot to follow the parameterized target
// maintaining the parameterized distance,
// returning the appropriate robot behavior.
Behavior Robot::followBehavior(const Vector &target, const float dist)
{
    float  r   = target.magnitude();
    if (r <= threshold()) return moveStopBehavior();
    Behavior beh = orientToBehavior(target, 0.0f);
    if ((beh.isDone()) && (r > dist)) return moveForwardBehavior(r - dist);
    return beh;
}


// Directs the robot to avoid the parameterized target,
// maintaining the parameterized distance, activating and
// returning the appropriate robot behavior.
Behavior Robot::avoid(const Vector &target, const float dist)
{
    return behavior = avoidBehavior(target, dist);
}


// Directs the robot to avoid the parameterized target,
// maintaining the parameterized distance,
// returning the appropriate robot behavior.
Behavior Robot::avoidBehavior(const Vector &target, const float dist)
{
    float r                     = target.magnitude();
    if (r < dist) return sumBehaviors(orientToBehavior(target, 180.0f),
                                      moveForwardBehavior(dist - r));
    return moveStopBehavior();
}


// Orients the robot with respect to the parameterized target
// such that if the robot were moving forward, it would move
// in a circular path maintaining the parameterized distance
// around the target, activating and returning the appropriate
// robot behavior.
Behavior Robot::orientForOrbit(const Vector &target, const float dist)
{
    return behavior = orientForOrbitBehavior(target, dist);
}


// Orients the robot with respect to the parameterized target
// such that if the robot were moving forward, it would move
// in a circular path maintaining the parameterized distance
// around the target, returning the appropriate robot behavior.
Behavior Robot::orientForOrbitBehavior(const Vector &target,
                                       const float dist)
{
    float  r = target.magnitude(),       dir = 0.0f;
    if      (r > dist + collisionRadius()) dir = 0.0f;
    else if (r < dist - collisionRadius()) dir = 180.0f;
    else dir = 180.0f - r * 90.0f / collisionRadius();
    return orientToBehavior(target, dir);
}


// Guides the robot around the parameterized target
// in a circular path maintaining the parameterized distance,
// activating and returning the appropriate robot behavior.
Behavior Robot::orbit(const Vector &target, const float dist)
{
    return behavior = orbitBehavior(target, dist);
}


// Guides the robot around the parameterized target
// in a circular path maintaining the parameterized distance,
// returning the appropriate robot behavior.
Behavior Robot::orbitBehavior(const Vector &target, const float dist)
{
    return sumBehaviors(orientForOrbitBehavior(target, dist),
                        moveForwardBehavior(maxSpeed()));
}


// Handles incoming packets, returning true
// if successful, false otherwise.
bool Robot::processPackets()
{
    bool   success = true;
    Packet packet;
    while (!msgQueue.empty())
    {
        packet = msgQueue.front();
        if (!processPacket(packet)) success = false;
        msgQueue.pop();
		}
    return success;
}


// Responds appropriately to auction announcements.
bool Robot::processPacket(Packet &packet)
{
    bool success = false;
    switch (packet.type)
    {
        case AUCTION_ANNOUNCEMENT:
        {
           float range = rangeSensor(packet);
           if (range > 0.0f)
           {
               float b_j = E * range;
               Bid    *bid   = new Bid(b_j, getID());
               success     = env->sendMsg(bid, packet.fromID, (-1 * (ID * 10)), BID);
           }
           else success    = true;
        }
        break;
        default: break;
    }
    return success;
}


// Returns with the distance from this robot
// to the position being auctioned.
float Robot::rangeSensor(Packet &packet)
{

    // unpack the auction from the packet
    Auction_Announcement *auctionAnnouncement = (Auction_Announcement *)packet.msg;

    // unpack the formation definition from the state within the auction
    Formation formation = auctionAnnouncement->s_i.formation;

    // get the range from the auctioneer to this robot
    float range = env->distanceToRobot(env->getCell(packet.fromID), this);

    // if the auctioneer is beyond sensor range, do not bid
    if (range > SENSOR_RANGE) return -1.0f;

    // get the relative bearing from the auctioneer to this robot
    float bearing = bearingSensor(packet.fromID);

    // get the angle between the auctioneer and the position being auctioned
    float ang = atan2(formation.getFunction()(formation.getRadius()), formation.getRadius()) -
                  bearing;

    // get the vector between the auctioneer and this robot
    Vector d = (*(Vector *)this) - (*(Vector *)env->getCell(packet.fromID));

    // get the vector between the auctioneer and the position being auctioned
    Vector e;
    e.y = formation.getFunction()(formation.getRadius());
    e.x = sqrt((formation.getRadius() * formation.getRadius()) - (e.y * e.y));

    // get the vector between the position being auctioned and this robot
    Vector a;
    if (auctionAnnouncement->right)    // if the position is to the right of the auctioneer
        a = d - e;
    else
        a = d + e;

    // range is the magnitude of the vector between
    // the position being auctioned and this robot
    range = a.magnitude();

    //cout << "robot " << getID() << " bidding " << range << endl;

    return range;
}


// Returns the relative bearing (in degrees) from the robot
// with the parameterized ID and this robot.
float Robot::bearingSensor(int &cellID)
{

    // get the vector between the auctioneer and this robot
    Vector e = (*(Vector *)this) - (*(Vector *)env->getCell(cellID));

    // get the distance between the auctioneer and this robot
    float dist = e.magnitude();

    // get the bearing between the auctioneer and this robot
    float bearing = radiansToDegrees(atan2(e.y, e.x));

    // subtract the bearing between the auction and this robot
    // from the auctioneer's heading
    bearing = env->getCell(cellID)->getHeading() - bearing;

    return bearing;
}
