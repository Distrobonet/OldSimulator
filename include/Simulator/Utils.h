//
// Filename:        "Utils.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This file contains various utility functions.
//

// preprocessor directives
#ifndef UTILS_H
#define UTILS_H
#include <cstdlib>
#include <iostream>
using namespace std;



// debug definitions
//#define DEBUG (1)



// math pi definition
#ifndef PI
#define PI (M_PI)
#endif

// global constants
static const float TWO_PI      = 2.0f * PI;
static const float PI_OVER_180 = PI / 180.0f;


// Scales the parameterized angle (in degrees) to an angle [-180, 180].
inline float scaleDegrees(float theta)
{
    if         (theta >   0.0f)
        while ((theta >=  360.0f) || (theta >  180.0f)) theta -= 360.0f;
    else if    (theta <   0.0f)
        while ((theta <= -360.0f) || (theta < -180.0f)) theta += 360.0f;
    return theta;
}   // scaleDegrees(float)



// Scales the parameterized angle (in radians) to an angle [-2 * PI, 2 * PI].
inline float scaleRadians(float theta)
{
    if         (theta >   0.0f)
        while ((theta >=  TWO_PI) || (theta >  PI)) theta -= TWO_PI;
    else if    (theta <   0.0f)
        while ((theta <= -TWO_PI) || (theta < -PI)) theta += TWO_PI;
    return theta;
}   // scaleRadians(float)



// Converts the parameterized angle (in degrees) to an angle in radians.
inline float degreesToRadians(float theta)
{
    return scaleDegrees(theta) * PI_OVER_180;
}   // degreesToRadians(float)



// Converts the parameterized angle (in radians) to an angle in degrees.
inline float radiansToDegrees(float theta)
{
    return scaleRadians(theta) / PI_OVER_180;
}   // radiansToDegrees(float)



// Returns a floating-point number [min, max].
inline float frand(const float min = 0.0f, const float max = 1.0f)
{
    return min + (max - min) * float(rand()) / float(RAND_MAX);
}   // frand()



// Returns an integer number [min, max].
inline int irand(const int min = 0, const int max = 1)
{
    return min + rand() % (max + 1);
}   // irand()


// Returns -1 or 1.
inline float randSign()
{
    return (rand() % 2) ? -1.0f : 1.0f;
}   // randSign()


// Returns the sign of the parameterized number.
inline float sign(const float f)
{
    return (f < 0.0f) ? -1.0f : 1.0f;
}   // sign(const float)

#endif
