//
// Filename:        "State.h"
//
// Programmer:      Ross Mead
// Last modified:   13Apr2011
//
// Description:     This structure defines a robot cell state.
//

// preprocessor directives
#ifndef STATE_H
#define STATE_H
#include <Simulator/Formation.h>
#include <Simulator/Vector.h>
#include <ros/serialization.h>
//#include <ros/message_traits.h>

using namespace std;



// defines a robot cell state
struct State
{

    // <data members>
    Formation            formation;     // the current formation
    Vector               gradient;      // the formation gradient
    vector<Relationship> rels;          // the formation relationships
    Vector               transError;    // the summed translational error
    float                rotError;      // the summed rotational error
    int                  tStep;         // the time step in the formation
    int                  refID;         // the ID of the reference nbr
    float                temperature;   // the current temperature
    float                heat;          // the current heat
    static const string __s_getMD5Sum() { return "4a842b65f413084dc2b10fb484ea7f17"; }
    static const string __getMD5Sum() { return "4a842b65f413084dc2b10fb484ea7f17"; }
    static const string __s_getDataType() { return "Simulator/State"; }
    static const string __getDataType() { return "Simulator/State"; }
    static const string __s_getMessageDefinition() { return "A state messsge"; }

//    vector<PropMsg>      props;         // maintaining responses of the
                                        // from the request msgs sent


    // <constructors>


    // Default constructor that initializes
    // this state to the parameterized values.
    State(const Formation            f      = Formation(),
          const Vector               grad   = Vector(),
          const vector<Relationship> r      = vector<Relationship>(),
          const Vector               tError = Vector(),
          const float                rError = 0.0f,
          const int                  ts     = 0,
          const int                  rID    = -1,
          const float                temp   = 0.0f,
          const float                h      = 0.0f)
          : formation(f),       gradient(grad),   rels(r),
            transError(tError), rotError(rError), tStep(ts), refID(rID),
            temperature(temp),  heat(h)
    {
    }   // State(const..{Formation, Vector, LL<Relationship>, Vector, int,int})



    // TODO: this needs to be fixed
//    StateMsg getStateAsMsg()
//    {
//    	StateMsg s;
//    	s.formation = formation;
//    	s.frp = frp;
//    	s.relationship = relationships;
//    	...
//    	return s;
//    }

};



namespace ros
{
namespace serialization
{


template<>
struct Serializer<State>
{
  template<typename Stream>
  inline static void write(Stream& stream, const State& t)
  {
    stream.next(t.formation);
    stream.next(t.gradient);
    stream.next(t.rels);
    stream.next(t.transError);
    stream.next(t.rotError);
    stream.next(t.tStep);
    stream.next(t.refID);
    stream.next(t.temperature);
    stream.next(t.heat);
//    stream.next(t.__s_getDataType());
//    stream.next(t.__s_getMD5Sum());
  }

  template<typename Stream>
  inline static void read(Stream& stream, State& t)
  {
	stream.next(t.formation);
	stream.next(t.gradient);
	stream.next(t.rels);
	stream.next(t.transError);
	stream.next(t.rotError);
	stream.next(t.tStep);
	stream.next(t.refID);
	stream.next(t.temperature);
	stream.next(t.heat);
//    stream.next(t.__s_getDataType());
//    stream.next(t.__s_getMD5Sum());
  }

  inline static uint32_t serializedLength(const State& t)
  {
    uint32_t size = 0;
    size += serializationLength(t.formation);
    size += serializationLength(t.gradient);
    size += serializationLength(t.rels);
    size += serializationLength(t.transError);
    size += serializationLength(t.rotError);
    size += serializationLength(t.tStep);
    size += serializationLength(t.refID);
    size += serializationLength(t.temperature);
    size += serializationLength(t.heat);
//    size += serializationLength(t.__s_getDataType());
//    size += serializationLength(t.__s_getMD5Sum());
    return size;
  }

  template<typename T, typename Stream>
  inline void serialize(Stream& stream, const T& t)
  {
    Serializer<T>::write(stream, t);
  }

  /**
   * \brief Deserialize an object.  Stream here should normally be a ros::serialization::IStream
   */
  template<typename T, typename Stream>
  inline void deserialize(Stream& stream, T& t)
  {
    Serializer<T>::read(stream, t);
  }
};

} // namespace serialization
}

namespace ros
{
namespace serialization
{

template<>
struct Serializer<Vector>
{
  template<typename Stream>
  inline static void write(Stream& stream, const Vector& t)
  {
	  stream.next(t.x);
	  stream.next(t.y);
	  stream.next(t.z);
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.color[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.rotate[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.scale[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.translate[i]);
		}
	  stream.next(t.showHead);
	  stream.next(t.showLine);
  }

  template<typename Stream>
  inline static void read(Stream& stream, Vector& t)
  {
	  stream.next(t.x);
	  stream.next(t.y);
	  stream.next(t.z);
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.color[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.rotate[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.scale[i]);
		}
		for (size_t i = 0; i < 3; i++)
		{
		  stream.next(t.translate[i]);
		}
	  stream.next(t.showHead);
	  stream.next(t.showLine);
  }

  inline static uint32_t serializedLength(const Vector& t)
  {
    uint32_t size = 0;
    size += serializationLength(t.x);
    size += serializationLength(t.y);
    size += serializationLength(t.z);
	for (size_t i = 0; i < 3; i++)
	{
		size += serializationLength(t.color[i]);
	}
	for (size_t i = 0; i < 3; i++)
	{
		size += serializationLength(t.rotate[i]);
	}
	for (size_t i = 0; i < 3; i++)
	{
		size += serializationLength(t.scale[i]);
	}
	for (size_t i = 0; i < 3; i++)
	{
		size += serializationLength(t.translate[i]);
	}
    size += serializationLength(t.showHead);
    size += serializationLength(t.showLine);
    return size;
  }
};

} // namespace serialization
} // namespace ros

namespace ros
{
namespace serialization
{

template<>
struct Serializer<Relationship>
{
  template<typename Stream>
  inline static void write(Stream& stream, const Relationship& t)
  {
	  stream.next(t.relActual);
	  stream.next(t.relDesired);
	  stream.next(t.ID);
  }

  template<typename Stream>
  inline static void read(Stream& stream, Relationship& t)
  {
	  stream.next(t.relActual);
	  stream.next(t.relDesired);
	  stream.next(t.ID);
  }

  inline static uint32_t serializedLength(const Relationship& t)
  {
    uint32_t size = 0;
    size += serializationLength(t.relActual);
    size += serializationLength(t.relDesired);
    size += serializationLength(t.ID);
    return size;
  }
};

} // namespace serialization
} // namespace ros

namespace ros
{
namespace serialization
{

template<>
struct Serializer<Formation>
{
  template<typename Stream>
  inline static void write(Stream& stream, const Formation& t)
  {
	  stream.next(t.radius);
	  stream.next(t.heading);
	  stream.next(t.seedGradient);
	  stream.next(t.seedID);
	  stream.next(t.formationID);
  }

  template<typename Stream>
  inline static void read(Stream& stream, Formation& t)
  {
	  stream.next(t.radius);
	  stream.next(t.heading);
	  stream.next(t.seedGradient);
	  stream.next(t.seedID);
	  stream.next(t.formationID);
  }

  inline static uint32_t serializedLength(const Formation& t)
  {
    uint32_t size = 0;
    size += serializationLength(t.radius);
    size += serializationLength(t.heading);
    size += serializationLength(t.seedGradient);
    size += serializationLength(t.seedID);
    size += serializationLength(t.formationID);
    return size;
  }
};

} // namespace serialization
} // namespace ros



#endif
