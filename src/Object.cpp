//
// Filename:        "Object.cpp"
//
// Description:     This class implements a 2-dimensional object.
//

// preprocessor directives
#include <Simulator/Environment.h>
#include <Simulator/Object.h>

// <protected static data members>
int Object::nObjects = 0;   // initializes the number of objects to 0


// Default constructor that initializes this object to the parameterized values.
Object::Object(const float dx, const float dy, const float dz,
               const float r)
{
    init(dx, dy, dz, r);
    ID = ++nObjects;
}   // Object(const float, const float, const float, const Color)


// Copy constructor that copies the contents of
// the parameterized object into this object.
Object::Object(const Object &obj)
{
//    init(obj.x, obj.y, obj.z);
//    //setColor(obj.color);
//    for (int i = 0; i < 3; ++i)
//    {
//        translate[i] = obj.translate[i];
//        rotate[i]    = obj.rotate[i];
//        scale[i]     = obj.scale[i];
//    }
//    showFilled  = obj.showFilled;
//    showPos     = obj.showPos;
//    ID          = obj.ID;
//    env         = obj.env;
}   // Object(const Object &)



Object::~Object(){}


// Attempts to set the radius to the parameterized radius,
// returning true if successful, false otherwise.
bool Object::setRadius(const float r)
{
  //return Circle::setRadius(r);
}   // setRadius(const float)


// Attempts to set the environment to the parameterized environment,
// returning true if successful, false otherwise.
bool Object::setEnvironment(Environment *e)
{
    env = e;
    return true;
}   // setEnvironment(Environment *)



// Returns the environment of this object.
Environment* Object::getEnvironment() const
{
    return env;
}   // getEnvironment() const



// Returns the ID of this object.

int Object::getID() const
{
    return ID;
}   // getID()



// Initializes the object to the parameterized values,
// returning true if successful, false otherwise.
bool Object::init(const float dx, const float dy, const float dz,
                  const float r)
{
    //Circle::init(dx, dy, dz, r);
    //showFilled = DEFAULT_OBJECT_SHOW_FILLED;
    setEnvironment(NULL);
    return true;
}   // init(const float..<4>, const Color)
