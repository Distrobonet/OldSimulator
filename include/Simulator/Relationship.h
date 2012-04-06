//
// Filename:        "Relationship.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This structure defines a desired and actual vector
//                  relationship to a robot with the specified ID number.
//

// preprocessor directives
#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H
#include <vector>
#include <Simulator/GLIncludes.h>
#include <Simulator/Packet.h>
#include <Simulator/Vector.h>
using namespace std;



// global constants
static const int ID_NO_NBR = ID_BROADCAST;



// defines a desired and actual vector relationship
// to a robot with the specified ID number
struct Relationship
{

    // <data members>
    Vector relDesired, relActual;
    int  ID;



    // <constructors>

    //
    // Relationship(desired, actual, id)
    // Last modified: 04Sep2006
    //
    // Default constructor that initializes
    // this relationship to the parameterized values.
    //
    // Returns:     <none>
    // Parameters:
    //      desired in      the default desired relationship
    //      actual  in      the default actual relationship
    //      id      in      the default ID
    //
    Relationship(const Vector desired  = Vector(),
                 const Vector actual   = Vector(),
                 const int  id       = ID_NO_NBR)
        : relDesired(desired), relActual(actual), ID(id)
    {
    }   // Relationship(const Vector, const Vector, const int)



    // <utility functions>

    //
    // Vector getError(desired, actual, id)
    // Last modified: 12Aug2006
    //
    // Returns the difference (error) between desired and actual relationships.
    //
    // Returns:     difference between desired and actual relationships
    // Parameters:  <none>
    //
    Vector getError()
    {
        return relDesired - relActual;
    }   // getError()



    // <friend functions>

    //
    // Relationship* relWithID(rels, id)
    // Last modified: 29Nov2009
    //
    // Returns a reference to the relationship with the parameterized ID
    // within the parameterized list, NULL otherwise.
    //
    // Returns:     reference to the relationship with the parameterized ID
    // Parameters:
    //      rels    in/out  the list of relationships
    //      id      in      the ID of the relationship to be found
    //
    friend Relationship* relWithID(vector<Relationship> &rels,
                                   const int           id)
    {
        for (int i = 0; i < (int)rels.size(); ++i)
            if (rels[i].ID == id) return &rels[i];
        return NULL;
    }   // relWithID(const LinkedList<Relationship> &, const int)
};  // Relationship

#endif
