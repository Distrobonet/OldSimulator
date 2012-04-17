//
// Filename:        "Packet.h"
//
// Description:     This structure defines a message packet.
//

// preprocessor directives
#ifndef PACKET_H
#define PACKET_H
#include <Simulator/GLIncludes.h>
using namespace std;



// global constants
static const int ID_OPERATOR  = -1;
static const int ID_BROADCAST = -2;
static const int ID_ROBOT     =  0;



// message type redefinition
typedef void* Message;



// defines a message packet
struct Packet
{

    // <data members>
    Message msg;
    int   toID, fromID, type;


    // Default constructor that initializes
    // this packet to the parameterized values.
    Packet(const Message m    = NULL,
           const int   to   = ID_BROADCAST,
           const int   from = ID_OPERATOR,
           const int   t    = 0)
        : msg(m), toID(to), fromID(from), type(t)
    {
    }   // Packet(const Message, const int, const int, const int)



    // Copy constructor that copies the contents of
    // the parameterized packet into this packet.
    Packet(const Packet &p)
        : msg(p.msg), toID(p.toID), fromID(p.fromID), type(p.type)
    {
    }   // Packet(const Packet &)



    ~Packet(){}



    // <utility functions>

    // Returns true if this packet is from an operator, false otherwise.
    bool fromOperator() const
    {
        return fromID == ID_OPERATOR;
    }   // fromOperator() const



    // Returns true if this packet is from a broadcast, false otherwise.
    bool fromBroadcast() const
    {
        return fromID == ID_BROADCAST;
    }   // fromBroadcast() const
};  // Packet

#endif
