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
#include <Simulator/GLIncludes.h>
using namespace std;



// debug definitions
//#define DEBUG (1)



// math pi definition
//#ifndef PI
//#define PI (M_PI)
//#endif
const double PI = 3.14159;//ADDED BY KEVIN



// global constants
static const float TWO_PI      = 2.0f * PI;
static const float PI_OVER_180 = PI / 180.0f;



//
// float scaleDegrees(theta)
// Last modified: 26Aug2006
//
// Scales the parameterized angle (in degrees) to an angle [-180, 180].
//
// Returns:     the scaled angle (in degrees)
// Parameters:
//      theta   in      the angle (in degrees) to be scaled
//
inline float scaleDegrees(float theta)
{
    if         (theta >   0.0f)
        while ((theta >=  360.0f) || (theta >  180.0f)) theta -= 360.0f;
    else if    (theta <   0.0f)
        while ((theta <= -360.0f) || (theta < -180.0f)) theta += 360.0f;
    return theta;
}   // scaleDegrees(float)



//
// float scaleRadians(theta)
// Last modified: 26Aug2006
//
// Scales the parameterized angle (in radians) to an angle [-2 * PI, 2 * PI].
//
// Returns:     the scaled angle (in radians)
// Parameters:
//      theta   in      the angle (in radians) to be scaled
//
inline float scaleRadians(float theta)
{
    if         (theta >   0.0f)
        while ((theta >=  TWO_PI) || (theta >  PI)) theta -= TWO_PI;
    else if    (theta <   0.0f)
        while ((theta <= -TWO_PI) || (theta < -PI)) theta += TWO_PI;
    return theta;
}   // scaleRadians(float)



//
// float degreesToRadians(theta)
// Last modified: 26Aug2006
//
// Converts the parameterized angle (in degrees) to an angle in radians.
//
// Returns:     the converted angle (in radians)
// Parameters:
//      theta   in      the angle (in degrees) to converted to radians
//
inline float degreesToRadians(float theta)
{
    return scaleDegrees(theta) * PI_OVER_180;
}   // degreesToRadians(float)



//
// float radiansToDegrees(theta)
// Last modified: 26Aug2006
//
// Converts the parameterized angle (in radians) to an angle in degrees.
//
// Returns:     the converted angle (in degrees)
// Parameters:
//      theta   in      the angle (in radians) to converted to degrees
//
inline float radiansToDegrees(float theta)
{
    return scaleRadians(theta) / PI_OVER_180;
}   // radiansToDegrees(float)



//
// float frand(min, max)
// Last modified: 08Nov2009
//
// Returns a floating-point number [min, max].
//
// Returns:     a floating-point number [min, max]
// Parameters:
//      min     in      the minimum of the number being returned
//      max     in      the maximum of the number being returned
//
inline float frand(const float min = 0.0f, const float max = 1.0f)
{
    return min + (max - min) * float(rand()) / float(RAND_MAX);
}   // frand()



//
// int irand(min, max)
// Last modified: 08Nov2009
//
// Returns an integer number [min, max].
//
// Returns:     an integer number [min, max]
// Parameters:
//      min     in      the minimum of the number being returned
//      max     in      the maximum of the number being returned
//
inline int irand(const int min = 0, const int max = 1)
{
    return min + rand() % (max + 1);
}   // irand()



//
// float randSign()
// Last modified: 26Aug2006
//
// Returns -1 or 1.
//
// Returns:     -1 or 1
// Parameters:  <none>
//
inline float randSign()
{
    return (rand() % 2) ? -1.0f : 1.0f;
}   // randSign()



//
// float sign()
// Last modified: 26Aug2006
//
// Returns the sign of the parameterized number.
//
// Returns:     the sign of the parameterized number
// Parameters:
//      f       in      the number to determine the sign of
//
inline float sign(const float f)
{
    return (f < 0.0f) ? -1.0f : 1.0f;
}   // sign(const float)

#endif
