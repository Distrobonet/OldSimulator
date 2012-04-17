//
// Filename:        "Behavior.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class describes a robot behavior.
//

// preprocessor directives#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <cmath>
#include <iostream>
using namespace std;



// enumerated status of a behavior
enum Status
{
    INACTIVE = 0,
    DONE,
    ACTIVE
};  // Status



// describes a robot behavior
class Behavior
{

    public:

        // <constructors>
        Behavior(const Status  s     = INACTIVE,
                 const float t     = 0.0f,
                 const float r     = 0.0f,
                 const float speed = 1.0f);
        Behavior(const Behavior &beh);

        // <public mutator functions>
        bool setStatus(const Status s);
        bool setTransVel(const float t);
        bool setRotVel(const float r);
        bool setDiffVel(const float right, const float left);
        bool setVelocity(const float t, const float r);
        bool setMaxSpeed(const float speed);

        // <public accessor functions>
        Status  getStatus()    const;
        bool    isActive()     const;
        bool    isDone()       const;
        bool    isInactive()   const;
        float getTransVel()  const;
        float getRotVel()    const;
        float getVelocity()  const;
        float getSpeed()     const;
        float getMaxSpeed()  const;

        // <overloaded operators>
        Behavior& operator =(const Behavior &beh);

        // <friend functions>
        friend Behavior subsumeBehaviors(const Behavior behLow,
                                         const Behavior behHigh);
        friend Behavior sumBehaviors(const Behavior beh1, const Behavior beh2);

    protected:

        // <protected data members>
        Status  status;
        float transVel, rotVel, maxSpeed;

        // <protected utility functions>
        float scaleVel();
};  // Behavior

#endif
