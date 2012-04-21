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
//#include <ros/message_traits.h>

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
    static const string __s_getMD5Sum() { return "4a842b65f413084dc2b10fb484ea7f17"; }
    static const string __getMD5Sum() { return "4a842b65f413084dc2b10fb484ea7f17"; }
    static const string __s_getDataType() { return "Simulator/State"; }
    static const string __getDataType() { return "Simulator/State"; }
    static const string __s_getMessageDefinition() { return "A state messsge"; }

//    vector<PropMsg>      props;         // maintaining responses of the
                                        // from the request msgs sent


    // <constructors>


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



    // TODO: this needs to be fixed
//    StateMsg getStateAsMsg()
//    {
//    	StateMsg s;
//    	s.formation = formation;
//    	s.frp = frp;
//    	s.relationship = relationships;
//    	...
//    	return s;
//    }

};






#endif
