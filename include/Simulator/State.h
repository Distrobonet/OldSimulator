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
#include <Simulator/PropMsg.h>
#include <vector>
#include <Simulator/Formation.h>
#include <Simulator/Vector.h>
using namespace std;



// defines a robot cell state
struct State
{

    // <data members>
    Formation            formation;     // the current formation
    Vector               gradient;      // the formation gradient
    vector<Relationship> rels;          // the formation relationships
    Vector               transError;    // the summed translational error
    float              rotError;      // the summed rotational error
    int                tStep;         // the time step in the formation
    int                refID;         // the ID of the reference nbr
    float              temperature;   // the current temperature
    float              heat;          // the current heat
    vector<PropMsg>     props;         // maintaining responses of the 
                                        // from the request msgs sent


    // <constructors>

    //
    // State(f, grad, r, e, d, s)
    // Last modified: 22Feb2010
    //
    // Default constructor that initializes
    // this state to the parameterized values.
    //
    // Returns:     <none>
    // Parameters:
    //      f       in      the default formation
    //      grad    in      the default gradient
    //      r       in      the default list of relationships
    //      tError  in      the default translational error
    //      rError  in      the default rotational error
    //      ts      in      the default time step
    //
    State(const Formation            f      = Formation(),
          const Vector               grad   = Vector(),
          const vector<Relationship> r      = vector<Relationship>(),
          const Vector               tError = Vector(),
          const float              rError = 0.0f,
          const int                ts     = 0,
          const int                rID    = -1,
          const float              temp   = 0.0f,
          const float              h      = 0.0f,
          const vector<PropMsg>     msgs   = vector<PropMsg>())
          : formation(f),       gradient(grad),   rels(r),
            transError(tError), rotError(rError), tStep(ts), refID(rID),
            temperature(temp),  heat(h),
            props(msgs)
    {
    }   // State(const..{Formation, Vector, LL<Relationship>, Vector, int,int})
};  // State

#endif
