//
// Filename:        "Robot.h"
//
// Programmer:      Ross Mead
// Last modified:   22Dec2010
//
// Description:     This class describes a 2-dimensional robot.
//

// preprocessor directives
#ifndef ROBOT_H
#define ROBOT_H

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>


#include <queue>
#include <Simulator/Behavior.h>
#include <Simulator/Circle.h>
#include <Simulator/Packet.h>

using namespace std;


#define SENSOR_RANGE (2.0f)

// global constants
static const float    DEFAULT_ROBOT_RADIUS       = 0.03f;
static const Behavior DEFAULT_ROBOT_BEHAVIOR     = Behavior();
static const bool     DEFAULT_ROBOT_SHOW_HEADING = true;
static const bool     DEFAULT_ROBOT_SHOW_LINE    = false;
static const bool     DEFAULT_ROBOT_SHOW_HEAD    = true;
static const bool     DEFAULT_ROBOT_SHOW_FILLED  = false;
static const float    FACTOR_MAX_SPEED           = 0.3f;
static const float    FACTOR_THRESHOLD           = 1.0f;
static const float    FACTOR_COLLISION_RADIUS    = 3.0f;



// forward declaration of a robot cell environment
class Environment;



// describes a 2-dimensional robot
class Robot: public Circle
{

    public:

        // <public data members>
        Vector   heading;          // 3D vector heading of robot
        Behavior behavior;         // behavior of robot
        bool     showHeading;      // shows the vector heading of the robot
        queue<Packet> msgQueue;    // message packet queue for communication

//        ros::NodeHandle aNode;
//        ros::Subscriber subRobot;
//        ros::Publisher  pub_cmd_vel, chatter_pub;


        string generateSubPubMessage(bool subOrPub);

        // <constructors>
        Robot(const float dx         = 0.0f,
              const float dy         = 0.0f,
              const float dz         = 0.0f,
              const float theta      = 0.0f);
        Robot(const Robot &r);

        // <destructors>
        virtual ~Robot();

        // <virtual public mutator functions>
        virtual bool setRadius(const float r = DEFAULT_ROBOT_RADIUS);
        virtual bool setHeading(const float theta);
        virtual bool setEnvironment(Environment *e);
        virtual void translateRelative(Vector v);
        virtual void translateRelative(const float dx = 0.0f,
                                       const float dy = 0.0f);
        virtual void rotateRelative(float theta);

        // <virtual public accessor functions>
        virtual Environment* getEnvironment() const;

        // <public accessor functions>
        int   getID()        const;
        float getHeading()   const;
        float getTransVel()  const;
        float getRotVel()    const;
        float getAngVel()    const;
        float getVelocity()  const;
        float getArcRadius() const;

        // <virtual public utility functions>
        virtual void draw();
        virtual void step();

        // <public utility functions>
        Vector getRelationship(Vector &target) const;
        float  getDistanceTo(Vector &target)   const;
        float  getAngleTo(Vector &target)      const;
        float  maxSpeed()                      const;
        float  maxAngSpeed()                   const;
        float  threshold()                     const;
        float  angThreshold()                  const;
        float  collisionRadius()               const;



        // <public sensor functions>
		vector<Vector> getObjectRelationships(
		   float maxDist = SENSOR_RANGE) const;

        // <public environment functions>
        Vector  getRelationship(const int toID) const;
        float   getDistanceTo(const int toID)   const;
        float   getAngleTo(const int toID)      const;

        // <virtual public environment functions>
        virtual bool sendMsg(const Message &msg  = NULL,
                             const int    toID = ID_BROADCAST,
                             const int    type = 0);
        virtual bool sendPacket(const Packet &p = Packet());

        // <public primitive behaviors>
        Behavior moveArc(const Vector &target);
        Behavior moveArcBehavior(const Vector &target);
        Behavior moveArc(const float t = 0.0f,
                         const float r = 0.0f,
                         const Status  s = ACTIVE);
        Behavior moveArcBehavior(const float t = 0.0f,
                                 const float r = 0.0f,
                                 const Status  s = ACTIVE);
        Behavior moveForward(const float speed);
        Behavior moveForwardBehavior(const float speed);
        Behavior moveBackward(const float speed);
        Behavior moveBackwardBehavior(const float speed);
        Behavior moveStop();
        Behavior moveStopBehavior();

        // <public pair behaviors>
        Behavior orientTo(const Vector &target, const float theta = 0.0f);
        Behavior orientToBehavior(const Vector &target,
                                  const float theta = 0.0f);
        Behavior follow(const Vector &target, const float dist);
        Behavior followBehavior(const Vector &target, const float dist);
        Behavior avoid(const Vector &target, const float dist);
        Behavior avoidBehavior(const Vector &target, const float dist);
        Behavior orientForOrbit(const Vector &target, const float dist);
        Behavior orientForOrbitBehavior(const Vector &target,
                                        const float dist);
        Behavior orbit(const Vector &target, const float dist);
        Behavior orbitBehavior(const Vector &target, const float dist);
        bool     processPacket(Packet &p);
        bool     processPackets();

    protected:

        // <protected data members>
        int          ID;     // identification number of robot
        Environment  *env;    // the environment of the robot

        // <protected static data members>
        static int  nRobots;    // number of total robots

        // <protected utility functions>
        float rangeSensor(Packet &p);
        float bearingSensor(int &cellID);

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f,
                          const float theta      = 0.0f);
};  // Robot

#endif
