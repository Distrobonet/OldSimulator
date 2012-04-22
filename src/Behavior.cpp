//
// Filename:        "Behavior.cpp"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class implements a robot behavior.
//

// preprocessor directives
#include <Simulator/Behavior.h>



// <constructors>

//
// Behavior(s, t, r, speed)
// Last modified: 04Sep2006
//
// Default constructor that initializes
// this behavior to the parameterized values.
//
// Returns:     <none>
// Parameters:
//      s       in      the status of the behavior (default INACTIVE)
//      t       in      the translational velocity of the behavior (default 0)
//      r       in      the rotational velocity of the behavior (default 0)
//      speed   in      the maximum speed of the behavior (default 1)
//
Behavior::Behavior(const float t,
                   const float r,
                   const float speed)
{
    setMaxSpeed(speed);
    setVelocity(t, r);
}   // Behavior(const Status, const float, const float, const float)



//
// Behavior(beh)
// Last modified: 27Aug2006
//
// Copy constructor that copies the contents of
// the parameterized behavior into this behavior.
//
// Returns:     <none>
// Parameters:
//      beh     in/out      the behavior being copied
//
Behavior::Behavior(const Behavior &beh)
{
    *this = beh;
}   // Behavior(const Behavior &)



// <public mutator functions>

//
// bool setStatus(s)
// Last modified: 26Aug2006
//
// Attempts to set the status to the parameterized status,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      s       in      the status to be set to
//
bool Behavior::setStatus(const Status s)
{
    switch (s)
    {
        case INACTIVE: case DONE: case ACTIVE: status = s; break;
        default: return false;
    }
    return true;
}   // setStatus(const Status)



//
// bool setTransVel(t)
// Last modified: 26Aug2006
//
// Attempts to set the translational velocity
// to the parameterized translational velocity,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      t       in      the translational velocity to be set to
//
bool Behavior::setTransVel(const float t)
{
    transVel = t;
    scaleVel();
    return true;
}   // setTransVel(const float)



//
// bool setRotVel(r)
// Last modified: 26Aug2006
//
// Attempts to set the rotational velocity
// to the parameterized rotational velocity,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      r       in      the rotational velocity to be set to
//
bool Behavior::setRotVel(const float r)
{
    rotVel = r;
    scaleVel();
    return true;
}   // setRotVel(const float)



//
// bool setDiffVel(right, left)
// Last modified: 07Nov2009
//
// Attempts to set the translational and rotational velocities
// based on left and right (differential) velocities,
// returning true if successful, false otherwise
//
// Returns:     true if successful, false otherwise
// Parameters:
//      right   in      the right velocity to be set to
//      left    in      the left velocity to be set to
//
bool Behavior::setDiffVel(float right, float left)
{
    return setVelocity(0.5f * (right + left), 1.0f * (right - left));
}   // setDiffVel(float, float)



//
// bool setVel(t, r)
// Last modified: 26Aug2006
//
// Attempts to set the translational and rotational velocities
// to the parameterized translational and rotational velocities,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      t       in      the translational velocity to be set to
//      r       in      the rotational velocity to be set to
//
bool Behavior::setVelocity(const float t, const float r)
{
    transVel = t;
    rotVel   = r;
    scaleVel();
    return true;
}   // setVelocity(const float, const float)



//
// bool setMaxSpeed(speed)
// Last modified: 26Aug2006
//
// Attempts to set the max speed to the parameterized max speed,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      speed   in      the max speed to be set to
//
bool Behavior::setMaxSpeed(const float speed)
{
    maxSpeed = speed;
    scaleVel();
    return true;
}   // setMaxSpeed(const float)



// <public accessor functions>

//
// Status getStatus() const
// Last modified: 26Aug2006
//
// Returns the status of this behavior.
//
// Returns:     the status of this behavior
// Parameters:  <none>
//
Status Behavior::getStatus() const
{
    return status;
}   // getStatus() const



//
// bool isActive() const
// Last modified: 26Aug2006
//
// Returns true if the status of this behavior is active, false otherwise.
//
// Returns:     true if the status is active, false otherwise
// Parameters:  <none>
//
bool Behavior::isActive() const
{
    return status == ACTIVE;
}   // isActive() const



//
// bool isDone() const
// Last modified: 26Aug2006
//
// Returns true if the status of this behavior is done, false otherwise.
//
// Returns:     true if the status is done, false otherwise
// Parameters:  <none>
//
bool Behavior::isDone() const
{
    return status == DONE;
}   // isDone() const



//
// bool isInactive() const
// Last modified: 26Aug2006
//
// Returns true if the status of this behavior is inactive, false otherwise.
//
// Returns:     true if the status is inactive, false otherwise
// Parameters:  <none>
//
bool Behavior::isInactive() const
{
    return status == INACTIVE;
}   // isActive() const



//
// float getTransVel() const
// Last modified: 26Aug2006
//
// Returns the translational velocity of this behavior.
//
// Returns:     the translational velocity of this behavior
// Parameters:  <none>
//
float Behavior::getTransVel() const
{
    return transVel;
}   // getTransVel() const



//
// float getRotVel() const
// Last modified: 26Aug2006
//
// Returns the rotational velocity of this behavior.
//
// Returns:     the rotational velocity of this behavior
// Parameters:  <none>
//
float Behavior::getRotVel() const
{
    return rotVel;
}   // getRotVel() const



//
// float getVelocity() const
// Last modified: 26Aug2006
//
// Returns the velocity of this behavior.
//
// Returns:     the velocity of this behavior
// Parameters:  <none>
//
float Behavior::getVelocity() const
{
    return transVel + rotVel;
}   // getVelocity() const



//
// float getSpeed() const
// Last modified: 26Aug2006
//
// Returns the speed of this behavior.
//
// Returns:     the speed of this behavior
// Parameters:  <none>
//
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
}   // getMaxSpeed() const



// <overloaded operators>

//
// Behavior& =(beh)
// Last modified: 26Aug2006
//
// Copies the contents of the parameterized behavior into this behavior.
//
// Returns:     this behavior
// Parameters:
//      beh     in/out      the behavior being copied
//
Behavior& Behavior::operator =(const Behavior &beh)
{
    setMaxSpeed(beh.maxSpeed);
    setVelocity(beh.transVel, beh.rotVel);
    setStatus(beh.status);
    return *this;
}   // =(const Behavior &)



// <friend functions>

//
// Behavior subsumeBehaviors(behLow, behHigh)
// Last modified: 14May2006
//
// Returns the parameterized behavior with the highest activation level.
//
// Returns:     the parameterized behavior with the highest activation level
// Parameters:
//      low     in      the lower priority behavior to compare to
//      high    in      the higher priority behavior to compare to
//
Behavior subsumeBehaviors(const Behavior behLow, const Behavior behHigh)
{
    return (behHigh.status > behLow.status) ? behHigh : behLow;
}   // subsumeBehaviors(const Behavior, const Behavior)



//
// Behavior sumBehaviors(beh1, beh2)
// Last modified: 16May2006
//
// Returns the sum of the parameterized behaviors.
//
// Returns:     the sum of the parameterized behaviors
// Parameters:
//      beh1    in      the first behavior to sum
//      beh2    in      the second behavior to sum
//
Behavior sumBehaviors(const Behavior beh1, const Behavior beh2)
{
    return Behavior (beh1.transVel + beh2.transVel,
                     beh1.rotVel   + beh2.rotVel,
                     min(beh1.maxSpeed, beh2.maxSpeed));
}   // subsumeBehaviors(const Behavior, const Behavior)



// <protected utility functions>

//
// float scaleVel()
// Last modified: 26Aug2006
//
// Scales the velocity of this behavior based on the max speed.
//
// Returns:     the scaled angle (in degrees)
// Parameters:  <none>
//
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
}   // scaleVel()
