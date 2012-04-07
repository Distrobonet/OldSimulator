//
// Filename:        "Robot.cpp"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class implements a 2-dimensional robot.
//



// preprocessor directives
#include <Simulator/Environment.h>
#include <Simulator/Robot.h>
#define SUBSCRIBER 0
#define PUBLISHER 1

// <protected static data members>
int Robot::nRobots = ID_ROBOT;   // initializes the number of robots to 0




// <constructors>

//
// Robot(dx, dy, dz, theta, colorIndex)
// Last modified: 04Sep2006
//
// Default constructor that initializes
// this robot to the parameterized values.
//
// Returns:     <none>
// Parameters:
//      dx          in      the initial x-coordinate of the robot (default 0)
//      dy          in      the initial y-coordinate of the robot (default 0)
//      dz          in      the initial z-coordinate of the robot (default 0)
//      theta       in      the initial heading of the robot (default 0)
//      colorIndex  in      the initial array index of the color of the robot
//

   double velocityX, velocityY, theta;

static void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	velocityY = odom-> pose.pose.position.x;
	velocityX = -odom-> pose.pose.position.y;

	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;
	btScalar yaw = 0.0l;
	btQuaternion q(odom-> pose.pose.orientation.x,
	odom-> pose.pose.orientation.y,
	odom-> pose.pose.orientation.z,
	odom-> pose.pose.orientation.w);

	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	theta = angles::normalize_angle(yaw + M_PI / 2.0l);
	ros::spinOnce();
}





Robot::Robot(const float dx,    const float dy, const float dz,
             const float theta)
{
	subRobot = aNode.subscribe(generateSubPubMessage(SUBSCRIBER), 1000, callBackRobot);

	pub_cmd_vel = aNode.advertise < geometry_msgs::Twist > (generateSubPubMessage(PUBLISHER), 1);

    init(dx, dy, dz, theta);
    ID = --nRobots;

}   // Robot(const float..<4>, const Color)


// This method will generate the appropriate Subscriber/Publisher message for a new robot
// using the current number of robots + 1
string Robot::generateSubPubMessage(bool subOrPub)
{
	stringstream ss;//create a stringstream
	ss << (nRobots + 1);//add number to the stream
	string numRobots = ss.str();


	// Subscriber
	if(subOrPub == SUBSCRIBER)
	{
		string subString = "/robot_/base_pose_ground_truth";

		subString.insert(7, numRobots);
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


//
// Robot(r)
// Last modified: 03Sep2006
//
// Copy constructor that copies the contents of
// the parameterized robot into this robot.
//
// Returns:     <none>
// Parameters:
//      r       in/out      the robot being copied
//
Robot::Robot(const Robot &r)
{
    init(r.x, r.y, r.z, r.getHeading());
    for (int i = 0; i < 3; ++i)
    {
        translate[i] = r.translate[i];
        rotate[i]    = r.rotate[i];
        scale[i]     = r.scale[i];
    }
    showLine    = r.showLine;
    showHead    = r.showHead;
    showPos     = r.showPos;
    showHeading = r.showHeading;
    ID          = r.ID;
    env         = r.env;
    msgQueue    = r.msgQueue;
}   // Robot(const Robot &)



// <destructors>

//
// ~Robot()
// Last modified: 03Sep2006
//
// Destructor that clears this robot.
//
// Returns:     <none>
// Parameters:  <none>
//
Robot::~Robot()
{
}   // ~Robot()



// <virtual public mutator functions>

//
// bool setRadius(s)
// Last modified: 03Sep2006
//
// Attempts to set the radius to the parameterized radius,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      r       in      the radius to be set to
//
bool Robot::setRadius(const float r)
{
    if (!Circle::setRadius(r)) return false;
    return behavior.setMaxSpeed(maxSpeed());
}   // setRadius(const float)



//
// bool setHeading(theta)
// Last modified: 03Sep2006
//
// Attempts to set the heading to the parameterized heading,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      theta   in      the heading to be set to
//
bool Robot::setHeading(const float theta)
{
    return heading.setPolar(radius + VECTOR_HEAD_HEIGHT, theta);
}   // setHeading(const float)



//
// bool setEnvironment(e)
// Last modified: 03Sep2006
//
// Attempts to set the environment to the parameterized environment,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      e       in      the envirnment to be set to
//
bool Robot::setEnvironment(Environment *e)
{
    env = e;
    return true;
}   // setEnvironment(Environment *)



//
// void translateRelative(v)
// Last modified: 03Sep2006
//
// Translates the robot relative to itself based
// on the parameterized translation vector.
//
// Returns:     <none>
// Parameters:
//      v       in      the translation vector
//
void Robot::translateRelative(Vector v)
{
    v.rotateRelative(getHeading());
    x += v.x;
    y += v.y;
}   // rotateRelative(const Vector)



//
// void translateRelative(dx, dy)
// Last modified: 03Sep2006
//
// Translates the robot relative to itself based
// on the parameterized x-/y-coordinate translations.
//
// Returns:     <none>
// Parameters:
//      dx      in      the x-coordinate translation
//      dy      in      the y-coordinate translation
//
void Robot::translateRelative(const float dx, const float dy)
{
    translateRelative(Vector(dx, dy));
}   // rotateRelative(const float, const float)



//
// void rotateRelative(theta)
// Last modified: 03Sep2006
//
// Rotates the robot about itself (in 2-dimensions)
// based on the parameterized rotation angle.
//
// Returns:     <none>
// Parameters:
//      theta   in      the rotation angle
//
void Robot::rotateRelative(float theta)
{
    heading.rotateRelative(theta);
}   // rotateRelative(float)



// <virtual public accessor functions>

//
// Environment* getEnvironment() const
// Last modified: 03Sep2006
//
// Returns the environment of this robot.
//
// Returns:     the environment of this robot
// Parameters:  <none>
//
Environment* Robot::getEnvironment() const
{
    return env;
}   // getEnvironment() const



// <public accessor functions>

//
// int getID() const
// Last modified: 03Sep2006
//
// Returns the ID of this robot.
//
// Returns:     the ID of this robot
// Parameters:  <none>
//
int Robot::getID() const
{
    return ID;
}   // getID()



//
// float getHeading() const
// Last modified: 03Sep2006
//
// Returns the heading of this robot.
//
// Returns:     the heading of this robot
// Parameters:  <none>
//
float Robot::getHeading() const
{
    return heading.angle();
}   // getHeading() const



//
// float getTransVel() const
// Last modified: 03Sep2006
//
// Returns the translational velocity of this robot.
//
// Returns:     the translational velocity of this robot
// Parameters:  <none>
//
float Robot::getTransVel() const
{
    return behavior.getTransVel();
}   // getTransVel() const



//
// float getRotVel() const
// Last modified: 03Sep2006
//
// Returns the rotational velocity of this robot.
//
// Returns:     the rotational velocity of this robot
// Parameters:  <none>
//
float Robot::getRotVel() const
{
    return behavior.getRotVel();
}   // getRotVel() const



//
// float getAngVel() const
// Last modified: 03Sep2006
//
// Returns the angular velocity of this robot.
//
// Returns:     the angular velocity of this robot
// Parameters:  <none>
//
float Robot::getAngVel() const
{
    return radiansToDegrees(getRotVel() / radius);
}   // getAngVel() const



//
// float getVelocity() const
// Last modified: 03Sep2006
//
// Returns the velocity of this robot.
//
// Returns:     the velocity of this robot
// Parameters:  <none>
//
float Robot::getVelocity() const
{
    return behavior.getVelocity();
}   // getVelocity() const



//
// float getArcRadius() const
// Last modified: 03Sep2006
//
// Returns the arc radius based on the translational
// and rotational velocities this robot.
//
// Returns:     the arc radius of this robot
// Parameters:  <none>
//
float Robot::getArcRadius() const
{
    return (behavior.getRotVel() == 0.0f) ? 0.0f :
            radius * behavior.getTransVel() / behavior.getRotVel();
}   // getArcRadius() const



// <virtual public utility functions>

//
// void draw()
// Last modified: 27Aug2006
//
// Renders the robot as a circle with a vector heading.
//
// Returns:     <none>
// Parameters:  <none>
//
void Robot::draw()
{
    if ((color[GLUT_RED]   == COLOR[INVISIBLE][GLUT_RED])   &&
        (color[GLUT_GREEN] == COLOR[INVISIBLE][GLUT_GREEN]) &&
        (color[GLUT_BLUE]  == COLOR[INVISIBLE][GLUT_BLUE])) return;

    vector<Vector> rels = getObjectRelationships(2.0f * collisionRadius());
    for (int i = 0, n = rels.size(); i < n; ++i)
    {
        rels[i].rotated(rotate[2]);
        rels[i].translated(x, y, z);
        rels[i].scaled(scale[0]);
        rels[i].draw();
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
}   // draw()



//
// void step()
// Last modified: 27Dec2010
//
// Executes the appropriate active behavior.
//
// Returns:     <none>
// Parameters:  <none>
//
void Robot::step()
{
    float collDist = collisionRadius();
    vector<Vector> rels = getObjectRelationships(collDist);
    if (rels.size() > 0)
    {
		float minDist  = HUGE;
		int   minIndex = 0;
		for (int i = 0, n = rels.size(); i < n; ++i)
		{
			float currDist = rels[i].magnitude();
			if (currDist < minDist)
			{
				minDist  = currDist;
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
}   // step()



// <public utility functions>

//
// Vector getRelationship(target)
// Last modified: 03Sep2006
//
// Returns the relationship from this robot
// to the parameterized target vector.
//
// Returns:     the relationship from this robot to the target vector
// Parameters:
//      target  in/out  the target vector being related to
//
Vector Robot::getRelationship(Vector &target) const
{
    Vector temp = target - *this;
    temp.rotateRelative(-getHeading());
    return temp;
}   // getRelationship(Vector &) const



//
// float getDistanceTo(target)
// Last modified: 07Nov2009
//
// Returns the distance from this robot
// to the parameterized target vector.
//
// Returns:     the distance from this robot to the target vector
// Parameters:
//      target  in/out  the target vector being related to
//
float Robot::getDistanceTo(Vector &target) const
{
    return getRelationship(target).magnitude();
}   // getDistanceTo(Vector &) const



//
// float getAngleTo(target)
// Last modified: 06Mar2007
//
// Returns the angle from this robot
// to the parameterized target vector.
//
// Returns:     the angle from this robot to the target vector
// Parameters:
//      target  in/out  the target vector being related to
//
float Robot::getAngleTo(Vector &target) const
{
    return getRelationship(target).angle();
}   // getAngleTo(Vector &) const



//
// float maxSpeed() const
// Last modified: 03Sep2006
//
// Returns the max speed of this robot.
//
// Returns:     the max speed of this robot
// Parameters:  <none>
//
float Robot::maxSpeed() const
{
    return FACTOR_MAX_SPEED * radius;
}   // maxSpeed() const



//
// float maxAngSpeed() const
// Last modified: 03Sep2006
//
// Returns the max angular speed of this robot.
//
// Returns:     the max angular speed of this robot
// Parameters:  <none>
//
float Robot::maxAngSpeed() const
{
    return radiansToDegrees(maxSpeed() / radius);
}   // maxAngSpeed() const



//
// float threshold() const
// Last modified: 03Sep2006
//
// Returns the minimum movement threshold of this robot.
//
// Returns:     the minimum movement threshold of this robot
// Parameters:  <none>
//
float Robot::threshold() const
{
    return FACTOR_THRESHOLD * maxSpeed();
}   // threshold() const



//
// float angThreshold() const
// Last modified: 07Nov2009
//
// Returns the minimum angular movement threshold of this robot.
//
// Returns:     the minimum angular movement threshold of this robot
// Parameters:  <none>
//
float Robot::angThreshold() const
{
    return 0.5f * FACTOR_THRESHOLD * maxAngSpeed();
}   // angThreshold() const



//
// float collisionRadius() const
// Last modified: 15Nov2006
//
// Returns the minimum collision radius of this robot.
//
// Returns:     the minimum collision radius of this robot
// Parameters:  <none>
//
float Robot::collisionRadius() const
{
    return FACTOR_COLLISION_RADIUS * radius;
}   // collisionRadius() const




// <virtual public sensor functions>

//
// vector<Vector> getObjectRelationships(maxDist)
// Last modified: 26Dec2010
//
// Returns the relationships from this robot
// to all objects in the environment.
//
// Returns:     the relationships from this robot to objects
// Parameters:
//      maxDist in      the maximum distance to be considered
//
vector<Vector> Robot::getObjectRelationships(const float maxDist) const
{
    return (env == NULL) ? vector<Vector>()
                         : env->getObjectRelationships(ID, maxDist);
}   // getObjectRelationships(const float) const



// <virtual public environment functions>

//
// Vector getRelationship(toID)
// Last modified: 03Sep2006
//
// Returns the relationship from this robot
// to the robot with the parameterized ID.
//
// Returns:     the relationship from this robot to another robot
// Parameters:
//      toID    in      the ID of the robot being related to
//
Vector Robot::getRelationship(const int toID) const
{
    return (env == NULL) ? Vector() : env->getRelationship(toID, ID);
}   // getRelationship(const int) const



//
// float getDistance(toID)
// Last modified: 07Nov2009
//
// Returns the distance from this robot
// to the robot with the parameterized ID.
//
// Returns:     the distance from this robot to another robot
// Parameters:
//      toID    in      the ID of the robot being related to
//
float Robot::getDistanceTo(const int toID) const
{
    return getRelationship(toID).magnitude();
}   // getDistanceTo(const int) const



//
// float getAngle(toID)
// Last modified: 03Sep2006
//
// Returns the angle from this robot
// to the robot with the parameterized ID.
//
// Returns:     the angle from this robot to another robot
// Parameters:
//      toID    in      the ID of the robot being related to
//
float Robot::getAngleTo(const int toID) const
{
    return getRelationship(toID).angle();
}   // getAngleTo(const int) const



// <virtual public environment functions>

//
// bool sendMsg(msg, toID, type)
// Last modified: 08Nov2009
//
// Attempts to send a packet to its destination
// based upon the given parameters, returning
// true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      msg     in/out  the message being sent
//      toID    in      the ID of the cell receiving the packet
//      type    in      the type of message being sent
//
bool Robot::sendMsg(const Message &msg, const int toID, const int type)
{
    if (sendPacket(Packet(msg, toID, ID, type))) return true;
    //delete msg;    // TO-DO: call destructor of appropriate type... ?
    return false;
}   // sendMsg(const Message &, const int, const int)



//
// bool sendPacket(p)
// Last modified: 08Nov2009
//
// Attempts to send a packet to its destination,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      p       in/out  the packet being sent
//
bool Robot::sendPacket(const Packet &p)
{
    if ((env != NULL) && (env->sendPacket(p))) return true;
    //delete p.msg;    // TO-DO: call destructor of appropriate type... ?
    return false;
}   // sendPacket(const Packet &)



// <public primitive behaviors>

//
// Behavior moveArc(target)
// Last modified: 03Sep2006
//
// Moves the robot using the parameterized movement vector,
// activating and returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target move of the behavior
//
Behavior Robot::moveArc(const Vector &target)
{
    return behavior = moveArcBehavior(target);
}   // moveArc(const Vector &)



//
// Behavior moveArcBehavior(target)
// Last modified: 07Nov2009
//
// Moves the robot using the parameterized movement vector,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target move of the behavior
//
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
                                r * theta / sin(theta), getDiameter() * theta);
*/
}   // moveArcBehavior(const Vector &)



//
// Behavior moveArc(t, r, s)
// Last modified: 03Sep2006
//
// Moves the robot using the parameterized translational
// and rotational velocities, activating and returning
// the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      t       in      the translational velocity of the behavior
//      r       in      the rotational velocity of the behavior
//      s       in      the status of the behavior (default ACTIVE)
//
Behavior Robot::moveArc(const float t, const float r, const Status s)
{
    return behavior = moveArcBehavior(t, r, s);
}   // moveArc(const float, const float, const Status)



//
// Behavior moveArcBehavior(t, r, s)
// Last modified: 03Sep2006
//
// Moves the robot using the parameterized translational
// and rotational velocities, returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      t       in      the translational velocity of the behavior
//      r       in      the rotational velocity of the behavior
//      s       in      the status of the behavior (default ACTIVE)
//
Behavior Robot::moveArcBehavior(const float t,
                                const float r,
                                const Status  s)
{
    return Behavior(s, t, r, maxSpeed());
}   // moveArcBehavior(const float, const float, const Status)



//
// Behavior moveForward(vel)
// Last modified: 03Sep2006
//
// Moves the robot forward using the parameterized robot velocity,
// activating and returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      vel     in      the velocity of the behavior
//
Behavior Robot::moveForward(const float speed)
{
    return behavior = moveForwardBehavior(speed);
}   // moveForward(const float)



//
// Behavior moveForwardBehavior(vel)
// Last modified: 03Sep2006
//
// Moves the robot forward using the parameterized robot velocity,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      vel     in      the velocity of the behavior
//
Behavior Robot::moveForwardBehavior(const float vel)
{
    return moveArcBehavior(vel, 0.0f);
}   // moveForwardBehavior(const float)



//
// Behavior moveBackward(vel)
// Last modified: 03Sep2006
//
// Moves the robot backward using the parameterized robot velocity,
// activating and returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      vel     in      the velocity of the behavior
//
Behavior Robot::moveBackward(const float vel)
{
    return behavior = moveForwardBehavior(vel);
}   // moveForward(const float)



//
// Behavior moveBackwardBehavior(vel)
// Last modified: 03Sep2006
//
// Moves the robot backward using the parameterized robot velocity,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      vel     in      the velocity of the behavior
//
Behavior Robot::moveBackwardBehavior(const float vel)
{
    return moveArcBehavior(-vel, 0.0f);
}   // moveForwardBehavior(const float)



//
// Behavior moveStop()
// Last modified: 03Sep2006
//
// Stops the robot from moving, activating and
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:  <none>
//
Behavior Robot::moveStop()
{
    return behavior = moveStopBehavior();
}   // moveStop()



//
// Behavior moveStopBehavior()
// Last modified: 03Sep2006
//
// Stops the robot from moving,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:  <none>
//
Behavior Robot::moveStopBehavior()
{
    return moveArcBehavior(0.0f, 0.0f, DONE);
}   // moveStopBehavior()



// <public pair behaviors>

//
// Behavior orientTo(target, theta)
// Last modified: 03Sep2006
//
// Rotates the robot to the parameterized heading
// relative to the paratermized target, activating and
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orient to
//      theta   in      the heading to maintain to the target
//
Behavior Robot::orientTo(const Vector &target, const float theta)
{
    return behavior = orientToBehavior(target, theta);
}   // orientTo(const Vector &, const float)



//
// Behavior orientToBehavior(target, theta)
// Last modified: 03Sep2006
//
// Rotates the robot to the parameterized heading
// relative to the paratermized target,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orient to
//      theta   in      the heading to maintain to the target
//
Behavior Robot::orientToBehavior(const Vector &target, const float theta)
{
    float delta   = scaleDegrees(target.angle() - theta);
    if (abs(delta) <= angThreshold()) return moveStopBehavior();
    return moveArcBehavior(0.0f, degreesToRadians(delta));
}   // orientToBehavior(const Vector &, const float)



//
// Behavior follow(target, dist)
// Last modified: 03Sep2006
//
// Directs the robot to follow the parameterized target
// maintaining the parameterized distance, activating and
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to follow
//      dist    in      the distance to maintain from the target
//
Behavior Robot::follow(const Vector &target, const float dist)
{
    return behavior = followBehavior(target, dist);
}   // follow(const Vector &, const float)



//
// Behavior followBehavior(target, dist)
// Last modified: 07Nov2009
//
// Directs the robot to follow the parameterized target
// maintaining the parameterized distance,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to follow
//      dist    in      the distance to maintain from the target
//
Behavior Robot::followBehavior(const Vector &target, const float dist)
{
    float  r   = target.magnitude();
    if (r <= threshold()) return moveStopBehavior();
    Behavior beh = orientToBehavior(target, 0.0f);
    if ((beh.isDone()) && (r > dist)) return moveForwardBehavior(r - dist);
    return beh;
}   // followBehavior(const Vector &, const float)



//
// Behavior avoid(target, dist)
// Last modified: 03Sep2006
//
// Directs the robot to avoid the parameterized target,
// maintaining the parameterized distance, activating and
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to avoid
//      dist    in      the distance to maintain from the target
//
Behavior Robot::avoid(const Vector &target, const float dist)
{
    return behavior = avoidBehavior(target, dist);
}   // avoid(const Vector &, const float)



//
// Behavior avoidBehavior(target, dist)
// Last modified: 07Nov2009
//
// Directs the robot to avoid the parameterized target,
// maintaining the parameterized distance,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to avoid
//      dist    in      the distance to maintain from the target
//
Behavior Robot::avoidBehavior(const Vector &target, const float dist)
{
    float r                     = target.magnitude();
    if (r < dist) return sumBehaviors(orientToBehavior(target, 180.0f),
                                      moveForwardBehavior(dist - r));
    return moveStopBehavior();
}   // avoidBehavior(const Vector &, const float)



//
// Behavior orientForOrbit(target, dist)
// Last modified: 03Sep2006
//
// Orients the robot with respect to the parameterized target
// such that if the robot were moving forward, it would move
// in a circular path maintaining the parameterized distance
// around the target, activating and returning the appropriate
// robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orient to for orbit
//      dist    in      the distance to maintain from the target
//
Behavior Robot::orientForOrbit(const Vector &target, const float dist)
{
    return behavior = orientForOrbitBehavior(target, dist);
}   // orientForOrbit(const Vector &, const float)



//
// Behavior orientForOrbitBehavior(target, dist)
// Last modified: 07Nov2009
//
// Orients the robot with respect to the parameterized target
// such that if the robot were moving forward, it would move
// in a circular path maintaining the parameterized distance
// around the target, returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orient to for orbit
//      dist    in      the distance to maintain from the target
//
Behavior Robot::orientForOrbitBehavior(const Vector &target,
                                       const float dist)
{
    float  r = target.magnitude(),       dir = 0.0f;
    if      (r > dist + collisionRadius()) dir = 0.0f;
    else if (r < dist - collisionRadius()) dir = 180.0f;
    else dir = 180.0f - r * 90.0f / collisionRadius();
    return orientToBehavior(target, dir);
}   // orientForOrbitBehavior(const Vector &, const float)



//
// Behavior orbit(target, dist)
// Last modified: 03Sep2006
//
// Guides the robot around the parameterized target
// in a circular path maintaining the parameterized distance,
// activating and returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orbit
//      dist    in      the distance to maintain from the target
//
Behavior Robot::orbit(const Vector &target, const float dist)
{
    return behavior = orbitBehavior(target, dist);
}   // orbit(const Vector &, const float)



//
// Behavior orbitBehavior(target, dist)
// Last modified: 03Sep2006
//
// Guides the robot around the parameterized target
// in a circular path maintaining the parameterized distance,
// returning the appropriate robot behavior.
//
// Returns:     the appropriate robot behavior
// Parameters:
//      target  in/out  the target to orbit
//      dist    in      the distance to maintain from the target
//
Behavior Robot::orbitBehavior(const Vector &target, const float dist)
{
    return sumBehaviors(orientForOrbitBehavior(target, dist),
                        moveForwardBehavior(maxSpeed()));
}   // orbit(const Vector &, const float)



// <virtual protected utility functions>

//
// bool init(dx, dy, dz, theta)
// Last modified: 06Nov2009
//
// Initializes the robot to the parameterized values,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      dx          in      the initial x-coordinate of the robot (default 0)
//      dy          in      the initial y-coordinate of the robot (default 0)
//      dz          in      the initial z-coordinate of the robot (default 0)
//      theta       in      the initial heading of the robot (default 0)
//      colorIndex  in      the initial array index of the color of the robot
//
bool Robot::init(const float dx,    const float dy, const float dz, const float theta)
{
    Circle::init(dx, dy, dz, DEFAULT_ROBOT_RADIUS);
    setHeading(theta);
    behavior         = DEFAULT_ROBOT_BEHAVIOR;
    behavior.setMaxSpeed(maxSpeed());
    showHeading      = DEFAULT_ROBOT_SHOW_HEADING;
    heading.showLine = DEFAULT_ROBOT_SHOW_LINE;
    heading.showHead = DEFAULT_ROBOT_SHOW_HEAD;
    showFilled       = DEFAULT_ROBOT_SHOW_FILLED;
    setEnvironment(NULL);
    return true;
}   // init(const float..<4>)



//
// bool processPackets()
// Last modified: 29Nov2009
//
// Handles incoming packets, returning true
// if successful, false otherwise.
//
// Returns:    true if successful, false otherwise
// Parameters: <none>
bool Robot::processPackets()
{
    bool   success = true;
    Packet p;
    while (!msgQueue.empty())
    {
        p = msgQueue.front();
        if (!processPacket(p)) success = false;
        msgQueue.pop();
		}
    return success;
}   // processPackets()



//
// bool processPacket(p)
// Last modified: 08Nov2009
//
// Responds appropriately to auction announcements.
//
// Returns:    true if successful, false otherwise
// Parameters:
//      p      in/out  the packet to be processed
//
bool Robot::processPacket(Packet &p)
{
    bool success = false;
    switch (p.type)
    {
        case AUCTION_ANNOUNCEMENT:
        {
           float range = rangeSensor(p);
           if (range > 0.0f)
           {
               float b_j = E * range;
               Bid    *b   = new Bid(b_j, getID());
               success     = env->sendMsg(b, p.fromID, (-1 * (ID * 10)), BID);
           }
           else success    = true;
        }
        break;
        default: break;
    }
    return success;
}   // processPacket(Packet &)



//
// float rangeSensor(p)
// Last modified: 08Nov2009
//
// Returns with the distance from this robot
// to the position being auctioned.
//
// Returns:    the distance from this robot to the position being auctioned
// Parameters:
//      p      in/out   the packet (auction) to be processed
float Robot::rangeSensor(Packet &p)
{

    // unpack the auction from the packet
    Auction_Announcement *aa = (Auction_Announcement *)p.msg;

    // unpack the formation definition from the state within the auction
    Formation f = aa->s_i.formation;

    // get the range from the auctioneer to this robot
    float range = env->distanceToRobot(env->getCell(p.fromID), this);

    // if the auctioneer is beyond sensor range, do not bid
    if (range > SENSOR_RANGE) return -1.0f;

    // get the relative bearing from the auctioneer to this robot
    float bearing = bearingSensor(p.fromID);

    // get the angle between the auctioneer and the position being auctioned
    float ang = atan2(f.getFunction()(f.getRadius()), f.getRadius()) -
                  bearing;

    // get the vector between the auctioneer and this robot
    Vector d = (*(Vector *)this) - (*(Vector *)env->getCell(p.fromID));

    // get the vector between the auctioneer and the position being auctioned
    Vector e;
    e.y = f.getFunction()(f.getRadius());
    e.x = sqrt((f.getRadius() * f.getRadius()) - (e.y * e.y));

    // get the vector between the position being auctioned and this robot
    Vector a;
    if (aa->right)    // if the position is to the right of the auctioneer
        a = d - e;
    else
        a = d + e;

    // range is the magnitude of the vector between
    // the position being auctioned and this robot
    range = a.magnitude();

    //cout << "robot " << getID() << " bidding " << range << endl;

    return range;
}   // rangeSensor(Packet &)



//
// float bearingSensor(cellID)
// Last modified: 08Nov2009
//
// Returns the relative bearing (in degrees) from the robot
// with the parameterized ID and this robot.
//
// Returns:    the relative bearing from robot[ID] and this robot
// Parameters:
//      cellID       in/out  the ID of the robot
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
}   // bearingSensor(int &)

