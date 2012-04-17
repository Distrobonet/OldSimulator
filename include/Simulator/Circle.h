//
// Filename:        "Circle.h"
//
// Description:     This class describes a 2-dimensional circle.
//

// preprocessor directives
#ifndef CIRCLE_H
#define CIRCLE_H
#include <Simulator/Vector.h>
using namespace std;



// global constants
static const Color   DEFAULT_CIRCLE_COLOR       = DEFAULT_VECTOR_COLOR;
static const float   DEFAULT_CIRCLE_RADIUS      = 1.0f;
static const bool    DEFAULT_CIRCLE_SHOW_POS    = false;
static const bool    DEFAULT_CIRCLE_SHOW_FILLED = false;
static const int     CIRCLE_N_LINKS             = 24;  // polygonal circle
static const float   CIRCLE_THETA               = 360.0f / CIRCLE_N_LINKS;
static const float   CIRCLE_COS_THETA           = cos(degreesToRadians(
                                                           CIRCLE_THETA));
static const float   CIRCLE_SIN_THETA           = sin(degreesToRadians(
                                                           CIRCLE_THETA));
static const float   CIRCLE_TAN_THETA           = tan(degreesToRadians(
                                                           CIRCLE_THETA));



// describes a 2-dimensional circle
class Circle: public Vector
{
	public:

        // <public data members>
        bool    showPos;       // shows the vector position of the circle
        bool    showFilled;    // shows the circle filled

        // <constructors>
        Circle(const float dx         = 0.0f,
               const float dy         = 0.0f,
               const float dz         = 0.0f,
               const float r          = DEFAULT_CIRCLE_RADIUS,
               const Color   colorIndex = DEFAULT_CIRCLE_COLOR);
        Circle(const Vector &c, const float r = DEFAULT_CIRCLE_RADIUS);
        Circle(const Circle &c);

        // <destructors>
        virtual ~Circle();

        // <public mutator functions>
        bool setRadius(const float r = DEFAULT_CIRCLE_RADIUS);
        bool setDiameter(const float d);
        bool setCircumference(const float c);
        bool setArea(const float a);

        // <public accessor functions>
        float getRadius()        const;
        float getDiameter()      const;
        float getCircumference() const;
        float getArea()          const;


        // <virtual overloaded operators>
        virtual Circle operator = (const Circle &c);

    protected:

        // <protected data members>
        float radius;     // radius of circle

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f,
                          const float r          = DEFAULT_CIRCLE_RADIUS,
                          const Color   colorIndex = DEFAULT_CIRCLE_COLOR);
};  // Circle
#endif
