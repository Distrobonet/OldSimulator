//
// Filename:        "Environment.h"
//
// Description:     This class describes a robot cell environment.
//

// preprocessor directives
#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#define SUBSCRIBER 0
#define ROBOT_LABEL 1

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <queue>
#include <vector>
#include <Simulator/Cell.h>
#include <Simulator/Object.h>

using namespace std;

#define VERBOSE (0)

#include "../msg_gen/cpp/include/Simulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/Simulator/Relationship.h"



// describes a robot cell environment
class Environment
{
    public:

        // <public data members>
        bool    startFormation;
        int     formationID;
        Vector  centroid;
        float   radius;
        Vector  distance;

        double robotX;
		double robotY;
		double robotTheta;
		nav_msgs::Odometry odomMsg;
		vector<ros::Subscriber> subRobots;
		vector< vector<double> > subRobotVels;

        // <constructors>
		Environment();
        Environment(int numRobots);
        Environment(const Environment &e);

        // <destructors>
        virtual ~Environment();

        void initOverloardSubscribers(Environment *e);
        void update(bool doSpin);

        // <virtual public mutator functions>
//        virtual bool setColor(const float r,
//                              const float g,
//                              const float b);
//        virtual bool setColor(const float clr[3]);
//        virtual bool setColor(const Color colorIndex = DEFAULT_VECTOR_COLOR);

        // <public mutator functions>
        bool addCell(Cell *c = NULL);
        bool removeCell();
        bool removeCell(Cell *c);
        bool addRobot(float x = 0.0f, float y = 0.0f, float z = 0.0f, float theta = 0.0f);
		bool addObject(float dx = 0.0f, float dy = 0.0f, float dz = 0.0f);
		bool addObject(Object *o = NULL);
		bool removeObject();
        bool removeObject(Object *o);

        // <public accessor functions>
        Cell*               getCell(int pos) const;
		Object*             getObject(int index) const;
        vector<Cell *>      getCells();    // BAD!!!
		vector<Object *>    getObjects();  // BAD!!!
        int                 getNumberOfCells() const;
        int                 getNumberOfFreeRobots() const;
		int                 getNObjects() const;


        // <virtual public utility functions>
//        virtual bool step();
        virtual void clear();

        //functions for the subscribers for all robots
        string generateSubMessage(bool msgType);
		void updatePosition(int robotID);
		void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom);

		float getDistanceTo(const int fromID, const int toID) const;
		float getAngleTo(const int fromID, const int toID) const;

        // <public utility functions>
        Vector  getRelationship(const int toID, const int fromID);
        float   getDistanceTo(const int id)   const;
        float   getAngleTo(const int id)      const;
        //brntbeer added for Prop_ops
        void    getCentroid( Vector v);
        void    getRadius(float rad);
        void    getDistance(Vector dist);
		//removed const from parameters
        bool    sendMsg(const Message &msg = NULL,
                        const int    toID   = ID_BROADCAST,
                        const int    fromID = ID_OPERATOR,
                        const int    type   = HEARTBEAT);

		//removed const
        bool    sendPacket(const Packet &p = Packet());
        vector<Cell *>  getCellVector();

        // <public sensor functions>

		//removed const
        bool    forwardPacket(const Packet &p);
        bool    forwardPackets();

        // <public utility cell functions>
        bool    showLine(const bool show);
        bool    showHead(const bool show);
        bool    showPos(const bool show);
        bool    showHeading(const bool show);

        // <public utility auctioning functions>
        //bool    auctionPosition(Cell* a);
        void    formUp();
        void    formFromClick(float x,float y);
        bool    changeFormation(Formation &f);
        bool    changeFormationSeed(Formation &f, int id);
        void    settleAuction(Cell* c,int rID);

        //brntbeer added for prop_ops
        friend bool changeFormationSim(const int index, const Vector frp);

        // Relationship service server
		ros::ServiceServer relationshipService;
		bool setRelationshipMessage(Simulator::Relationship::Request  &req, Simulator::Relationship::Response &res );
        void startRelationshipServiceServer();

    protected:

        // <protected data members>
        vector<Cell *>     cells;
		vector<Object *>   objects;
        queue<Packet>      msgQueue;
        Cell              *newestCell;
        Formation          formation;
        int                numOfRobots;

        // <virtual protected utility functions>
        virtual bool init();
        virtual bool initCells(const Formation f = Formation());
        virtual bool initNbrs(const int nNbrs = 0);
//		virtual bool initRobots();
};  // Environment

#endif
