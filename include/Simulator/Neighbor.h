//
// Filename:        "Neighbor.h"
//
// Description:     This structure defines a robot cell neighbor.
//

// preprocessor directives
#ifndef NEIGHBOR_H
#define NEIGHBOR_H
#include <Simulator/State.h>
using namespace std;



// defines a robot cell neighbor
struct Neighbor: Relationship, State
{
    // Default constructor that initializes
    // this neighbor to the parameterized values.
     Neighbor(const Relationship r = Relationship(),
             const State        s = State()): Relationship(r), State(s)
    {
    }   // Neighbor(const Relationship, const State)


    // Default constructor that sets the specified
    // data members to the appropriate values.
    Neighbor(const int  id,
             const State  s       = State(),
             const Vector desired = Vector(),
             const Vector actual  = Vector())
        : Relationship(desired, actual, id), State(s)
    {
    }   // Neighbor(const int, const State, const Vector, const Vector)



    // Copy constructor that copies the contents of
    // the parameterized neighbor into this neighbor.
    Neighbor(const Neighbor &n): Relationship(n), State(n)
    {
    }   // Neighbor(const Neighbor &)


    // <accessor functions>

    // Returns the relationship to this neighbor.
    Relationship getRelationship() const
    {
        return (Relationship)(*this);
    }   // getRelationship() const


    // Returns the state of this neighbor.
    State getState() const
    {
        return (State)(*this);
    }   // getState() const

    // Copies the contents of the parameterized
    // relationship into this neighbor.
    virtual Neighbor& operator =(const Relationship &r)
    {
        relDesired = r.relDesired;
        relActual  = r.relActual;
        ID         = r.ID;
        return *this;
    }   // =(const Relationship &)
    

    // Copies the contents of the parameterized
    // state into this neighbor.
    virtual Neighbor& operator =(const State &s)
    {
        formation  = s.formation;
        frp   = s.frp;
        rels       = s.rels;
        transError = s.transError;
        rotError   = s.rotError;
        tStep      = s.tStep;
        refID      = s.refID;
        return *this;
    }   // =(const State &)
};  // Neighbor

#endif
