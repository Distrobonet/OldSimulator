//
// Filename:        "Auctioning.h"
//
// Programmer:      Rob Long
// Last modified:   05Apr2012
//
// Description:     This file contains structures for auctioning.
//

// preprocessor directives
#ifndef AUCTIONING_H
#define AUCTIONING_H
#include <Simulator/Vector.h>
#include <Simulator/State.h>



//
#define MAX_INT   (2147483647)
#define MAX_FLOAT (999999999)



//
#define E         (1.0f)
#define X         (5.0f)



//
struct Auction_Announcement
{
    Vector p_j;
    State s_i;
    bool right;

    Auction_Announcement(const Vector p = Vector(),
                         const State  s = State(),
                         const bool   r = true): p_j(p), s_i(s), right(r)
    {
    }   // Auction_Announcement(const Vector, const State, const bool)
};  // Auction_Announcement



//
struct Bid
{
    float b_i;
    int   rID;

    Bid(const float b = MAX_FLOAT, const int id = -1): b_i(b), rID(id)
    {
    }   // Bid(const float, const int)
};  // Bid

#endif

