//
// Description:     This class implements a robot behavior.
//

// preprocessor directives
#include <Simulator/Behavior.h>



// Default constructor that initializes
// this behavior to the parameterized values.
Behavior::Behavior(const float t,
                   const float r,
                   const float speed)
{
    setMaxSpeed(speed);
    setVelocity(t, r);
}


// Copy constructor that copies the contents of
// the parameterized behavior into this behavior.
Behavior::Behavior(const Behavior &beh)
{
    *this = beh;
}


// Attempts to set the status to the parameterized status,
// returning true if successful, false otherwise.
bool Behavior::setStatus(const Status s)
{
    switch (s)
    {
        case INACTIVE: case DONE: case ACTIVE: status = s; break;
        default: return false;
    }
    return true;
}


// Attempts to set the translational velocity
// to the parameterized translational velocity,
// returning true if successful, false otherwise.
bool Behavior::setTransVel(const float t)
{
    transVel = t;
    scaleVel();
    return true;
}


// Attempts to set the rotational velocity
// to the parameterized rotational velocity,
// returning true if successful, false otherwise.
bool Behavior::setRotVel(const float r)
{
    rotVel = r;
    scaleVel();
    return true;
}


// Attempts to set the translational and rotational velocities
// based on left and right (differential) velocities,
// returning true if successful, false otherwise
bool Behavior::setDiffVel(float right, float left)
{
    return setVelocity(0.5f * (right + left), 1.0f * (right - left));
}


// Attempts to set the translational and rotational velocities
// to the parameterized translational and rotational velocities,
// returning true if successful, false otherwise.
bool Behavior::setVelocity(const float t, const float r)
{
    transVel = t;
    rotVel   = r;
    scaleVel();
    return true;
}


// Attempts to set the max speed to the parameterized max speed,
// returning true if successful, false otherwise.
bool Behavior::setMaxSpeed(const float speed)
{
    maxSpeed = speed;
    scaleVel();
    return true;
}


// Returns the status of this behavior.
Status Behavior::getStatus() const
{
    return status;
}


// Returns true if the status of this behavior is active, false otherwise.
bool Behavior::isActive() const
{
    return status == ACTIVE;
}


// Returns true if the status of this behavior is done, false otherwise.
bool Behavior::isDone() const
{
    return status == DONE;
}


// Returns true if the status of this behavior is inactive, false otherwise.
bool Behavior::isInactive() const
{
    return status == INACTIVE;
}


// Returns the translational velocity of this behavior.
float Behavior::getTransVel() const
{
    return transVel;
}


// Returns the rotational velocity of this behavior.
float Behavior::getRotVel() const
{
    return rotVel;
}


// Returns the velocity of this behavior.
float Behavior::getVelocity() const
{
    return transVel + rotVel;
}


// Returns the speed of this behavior.
float Behavior::getSpeed() const
{
    return abs(getVelocity());
}   // getSpeed() const



//
// float getMaxSpeed() const
// Last modified: 26Aug2006
//
// Returns the max speed of this behavior.
//
// Returns:     the max speed of this behavior
// Parameters:  <none>
//
float Behavior::getMaxSpeed() const
{
    return maxSpeed;
}


// Copies the contents of the parameterized behavior into this behavior.
Behavior& Behavior::operator =(const Behavior &beh)
{
    setMaxSpeed(beh.maxSpeed);
    setVelocity(beh.transVel, beh.rotVel);
    setStatus(beh.status);
    return *this;
}


// Returns the parameterized behavior with the highest activation level.
Behavior subsumeBehaviors(const Behavior behLow, const Behavior behHigh)
{
    return (behHigh.status > behLow.status) ? behHigh : behLow;
}


// Returns the sum of the parameterized behaviors.
Behavior sumBehaviors(const Behavior beh1, const Behavior beh2)
{
    return Behavior (beh1.transVel + beh2.transVel,
                     beh1.rotVel   + beh2.rotVel,
                     min(beh1.maxSpeed, beh2.maxSpeed));
}


// Scales the velocity of this behavior based on the max speed.
float Behavior::scaleVel()
{
    if (getSpeed() == 0.0f) return 0.0f;
    float scale = maxSpeed / getSpeed();
    if (scale < 1.0f)
    {
        transVel *= scale;
        rotVel   *= scale;
    }
    return scale;
}
