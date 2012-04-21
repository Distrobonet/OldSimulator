//
// Filename:        "State.h"
//
// Programmer:      Ross Mead
// Last modified:   13Apr2011
//
// Description:     This structure defines a robot cell state.
//

// preprocessor directives
#ifndef STATE_H
#define STATE_H
#include <Simulator/Formation.h>
#include <Simulator/Vector.h>
#include <ros/serialization.h>
#include <../msg_gen/cpp/include/Simulator/StateMessage.h>

using namespace std;



// defines a robot cell state
struct State
{

    // <data members>
    Formation            formation;     // the current formation
    Vector               gradient;      // the formation gradient
    vector<Relationship> rels;          // the formation relationships
    Vector               transError;    // the summed translational error
    float                rotError;      // the summed rotational error
    int                  tStep;         // the time step in the formation
    int                  refID;         // the ID of the reference nbr
    float                temperature;   // the current temperature
    float                heat;          // the current heat
    State 				 getStateMsgAsObject(Simulator::StateMessage stateMsg);

    // Default constructor that initializes
    // this state to the parameterized values.
    State(const Formation            f      = Formation(),
          const Vector               grad   = Vector(),
          const vector<Relationship> r      = vector<Relationship>(),
          const Vector               tError = Vector(),
          const float                rError = 0.0f,
          const int                  ts     = 0,
          const int                  rID    = -1,
          const float                temp   = 0.0f,
          const float                h      = 0.0f)
          : formation(f),       gradient(grad),   rels(r),
            transError(tError), rotError(rError), tStep(ts), refID(rID),
            temperature(temp),  heat(h)
    {
    }   // State(const..{Formation, Vector, LL<Relationship>, Vector, int,int})



    State getStateAsMsg(Simulator::StateMessage stateMsg)
    {
    	//Cant have *
//    	State *stateObject = new State();
    	//Those 4 cannot not be set using equals
//    	stateObject.formation = stateMsg.formation;
//    	stateObject.gradient = stateMsg.frp;
//    	stateObject.rels = stateMsg.relationships;
//    	stateObject.transError = stateMsg.linear_error;
    	//These can
//    	stateObject->rotError = stateMsg.angular_error;
//    	stateObject->tStep = stateMsg.timestep;
//    	stateObject->refID = stateMsg.reference_id;
//    	stateObject->temperature = stateMsg.temperature;
//    	stateObject->heat = stateMsg.heat;
//    	return stateObject;
    }

};






#endif
