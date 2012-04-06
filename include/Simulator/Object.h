//
// Filename:        "Object.h"
//
// Programmer:      Ross Mead
// Last modified:   22Dec2010
//
// Description:     This class describes a 2-dimensional object.
//

// preprocessor directives
#ifndef OBJECT_H
#define OBJECT_H
#include <Simulator/Circle.h>

using namespace std;



// global constants
static const Color    DEFAULT_OBJECT_COLOR        = DEFAULT_CIRCLE_COLOR;
static const float  DEFAULT_OBJECT_RADIUS       = 0.03f;
static const bool     DEFAULT_OBJECT_SHOW_FILLED  = false;
static const Behavior DEFAULT_OBJECT_BEHAVIOR = Behavior();


// forward declaration of an object environment
class Environment;



// describes a 2-dimensional object
class Object: public Circle
{

    public:

        // <constructors>
        Object(const float dx         = 0.0f,
               const float dy         = 0.0f,
               const float dz         = 0.0f,
               const float r          = DEFAULT_OBJECT_RADIUS,
               const Color   colorIndex = DEFAULT_OBJECT_COLOR);
        Object(const Object &obj);

        // <destructors>
        virtual ~Object();

        // <virtual public mutator functions>
        virtual bool setRadius(const float r = DEFAULT_OBJECT_RADIUS);
        virtual bool setEnvironment(Environment *e);

        // <virtual public accessor functions>
        virtual Environment* getEnvironment() const;

        // <public accessor functions>
        int getID() const;

        // <virtual public utility functions>
        virtual void draw();

    protected:

        // <protected data members>
        int         ID;     // identification number of object
        Environment  *env;    // the environment of the object

        // <protected static data members>
        static int  nObjects;    // number of total objects

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f,
                          const float r          = DEFAULT_OBJECT_RADIUS,
                          const Color   colorIndex = DEFAULT_OBJECT_COLOR);
};  // Object

#endif
