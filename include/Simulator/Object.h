//
// Filename:        "Object.h"
//
// Description:     This class describes a 2-dimensional object.
//

// preprocessor directives
#ifndef OBJECT_H
#define OBJECT_H

using namespace std;



// global constants
static const float    DEFAULT_OBJECT_RADIUS       = 0.03f;
static const bool     DEFAULT_OBJECT_SHOW_FILLED  = false;
static const Behavior DEFAULT_OBJECT_BEHAVIOR = Behavior();


// forward declaration of an object environment
class Environment;



// describes a 2-dimensional object
class Object
{

    public:

        // <constructors>
        Object(const float dx         = 0.0f,
               const float dy         = 0.0f,
               const float dz         = 0.0f,
               const float r          = DEFAULT_OBJECT_RADIUS);
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


    protected:
        // <protected data members>
        int          ID;      // identification number of object
        Environment  *env;    // the environment of the object

        // <protected static data members>
        static int  nObjects;    // number of total objects

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f,
                          const float r          = DEFAULT_OBJECT_RADIUS);
};  // Object

#endif
