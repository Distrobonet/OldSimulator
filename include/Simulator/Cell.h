//
// Filename:        "Cell.h"
//
// Description:     This class describes a robot cell.
//

// preprocessor directives
#ifndef CELL_H
#define CELL_H

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <vector>
#include <Simulator/Auctioning.h>
#include <Simulator/Neighborhood.h>
#include <Simulator/Behavior.h>
#include "std_msgs/String.h"

#include <ros/ros.h>	// Used for service to get Formation

// Formation service
#include "../msg_gen/cpp/include/Simulator/FormationMessage.h"
#include "../srv_gen/cpp/include/Simulator/CurrentFormation.h"

// State service
#include "../msg_gen/cpp/include/Simulator/StateMessage.h"
#include "../srv_gen/cpp/include/Simulator/State.h"

// Relationship service
#include "../msg_gen/cpp/include/Simulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/Simulator/Relationship.h"

using namespace std;


// message type index values
enum MessageType
{
    HEARTBEAT = 0,
    STATE,
    CHANGE_FORMATION,
    AUCTION_ANNOUNCEMENT,
    BID,
    NCELL_REQUEST,
    NCELL_RESPONSE,
    FCNTR_REQUEST,
    FCNTR_RESPONSE,
    FRAD_REQUEST,
    FRAD_RESPONSE,
    FCSEL_REQUEST,
    FCSEL_RESPONSE,
    FSEED_REQUEST,
    FSEED_RESPONSE
};  // MessageType



// global constants
static const bool   DEFAULT_CELL_SHOW_FILLED = true;
static const int    LEFT_NBR_INDEX           = 0;
static const int    RIGHT_NBR_INDEX          = 1;
static const int    NEIGHBORHOOD_SIZE        = 2;
static const float  MAX_TRANSLATIONAL_ERROR  = 0.02f;
static const float  FACTOR_MAX_SPEED         = 0.3f;
static const float  FACTOR_THRESHOLD         = 1.0f;
static const Behavior DEFAULT_ROBOT_BEHAVIOR     = Behavior();



// describes a robot cell
class Cell: public State, public Neighborhood
{
    friend class Environment;

    public:

        // <constructors>
        Cell(const int cellID);
        Cell(const Cell &r);

        // <destructors>
        virtual ~Cell();

        void update(bool doSpin);
        bool initNbrs(Cell *cell, int currentRobotsID);

        ros::NodeHandle stateNode;
        ros::Publisher state_pub;
		geometry_msgs::Twist commandVelocity;
		ros::Publisher cmd_velPub;
        ros::Subscriber leftNeighborState;
        ros::Subscriber rightNeighborState;


        // <public mutator functions>
        bool setState(const State &s);
        bool setNbrs(Neighborhood &nh);
        bool setAuctionStepCount(const int& asc);

        // <public accessor functions>
        State        getState() const;
        Neighborhood getNbrs()  const;
        int          getNBids() const;
        int          getAuctionStepCount() const;
        int 		 getID() const;
        void 		 setID(int cellID);



        // <virtual public utility functions>
        virtual void updateState();

        //TODO: fix this input data type
        void stateCallback(const Simulator::StateMessage &state);
        string generateSubMessage(int cellID);
        string generatePubMessage(int cellID);

        // <virtual public neighborhood functions>
        virtual bool changeFormation(const Formation &f,
                                     Neighbor         n = Neighbor());
//        virtual bool sendStateToNbrs();
//        virtual bool sendState(const int);

        // <public primitive behaviors>
        Behavior moveError();
        Behavior moveError(const Vector tError, const float rError);
        Behavior moveErrorBehavior(const Vector tError, const float rError);

        Behavior move(const Vector &target);
        Behavior move(const float t, const float r);
        Behavior moveStop();

        float maxSpeed() const;
        float threshold() const;
        float angThreshold() const;
        bool setHeading(const float theta);
        float maxAngSpeed() const;

        // <virtual overloaded operators>
        virtual Cell& operator =(const State &s);
        virtual Cell& operator =(const Neighborhood &nh);
//        virtual Cell& operator =(const Robot &r);

        // Service stuff
        // Formation service client
        bool setFormationFromService();
		ros::ServiceClient formationClient;
		Simulator::CurrentFormation formationSrv;

		// State service client
		bool getNeighborState(bool leftOrRight);
		ros::ServiceClient stateClient;
		Simulator::CurrentFormation stateSrv;

		// State service server
		ros::ServiceServer stateService;
		bool setStateMessage(Simulator::State::Request  &req, Simulator::State::Response &res);
		void startStateServiceServer();

		// Relationship service client
		bool getRelationship(int targetID);
		ros::ServiceClient relationshipClient;
		Simulator::CurrentFormation relationshipSrv;


    protected:

        // <protected data members>
        Formation currentFormation;
        Neighbor    *leftNbr, *rightNbr;
        Vector		heading;
        Behavior    behavior;         // behavior of robot
        float		cellX;
        float		cellTheta;
        int			ID;

        // <protected static data members>
        static int nCells;

        // <protected utility functions>
        void settleAuction();

        // <virtual protected utility functions>
        virtual bool init(const int ID = 0);
};  // Cell

#endif
