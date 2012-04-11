//
// Filename:        "Cell.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
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
#include <Simulator/Robot.h>
using namespace std;



//




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
static const Color   DEFAULT_CELL_COLOR       = WHITE;
static const bool    DEFAULT_CELL_SHOW_FILLED = true;
static const int   LEFT_NBR_INDEX           = 0;
static const int   RIGHT_NBR_INDEX          = 1;
static const int   NEIGHBORHOOD_SIZE        = 2;
static const float MAX_TRANSLATIONAL_ERROR  = 0.02f;



// describes a robot cell
class Cell: public State, public Neighborhood, public Robot
{
    friend class Environment;

    public:

        // <constructors>
        Cell(const float dx         = 0.0f,
             const float dy         = 0.0f,
             const float dz         = 0.0f,
             const float theta      = 0.0f);
        Cell(const Cell &r);

        // <destructors>
        virtual ~Cell();

        // <public mutator functions>
        bool setState(const State &s);
        bool setNbrs(Neighborhood &nh);
        bool setRobot(const Robot &r);
        bool setRobotP(Robot *r);
        bool setAuctionStepCount(const int& asc);

        // <public accessor functions>
        State        getState() const;
        Neighborhood getNbrs()  const;
        Robot        getRobot() const;
        int          getNBids() const;
        int          getAuctionStepCount() const;



        // <virtual public utility functions>
        virtual void draw();
        Cell* cStep();
        //virtual  step();
        virtual void updateState();

        // <virtual public neighborhood functions>
        virtual bool changeFormation(const Formation &f,
                                     Neighbor         n = Neighbor());
        virtual bool sendStateToNbrs();
        virtual bool sendState(const int);
        virtual bool processPackets();
        virtual bool processPacket(Packet &p);
        virtual bool processNCell(Packet &p);
        virtual bool processFcntr(Packet &p);
        virtual bool processFRad(Packet &p);
        virtual bool processFSeed(Packet &p);

        // <public primitive behaviors>
        Behavior moveError();
        Behavior moveError(const Vector tError, const float rError);
        Behavior moveErrorBehavior(const Vector tError, const float rError);

        // <virtual overloaded operators>
        virtual Cell& operator =(const State &s);
        virtual Cell& operator =(const Neighborhood &nh);
        virtual Cell& operator =(const Robot &r);

    protected:

        // <protected data members>
        vector<Bid *> bids;
        Neighbor     *leftNbr, *rightNbr;
        int         index;
        int         numBids;
        int           auctionStepCount;

        // <protected static data members>
        static int nCells;

        // <protected utility functions>
        void settleAuction();

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f,
                          const float theta      = 0.0f);
};  // Cell

#endif