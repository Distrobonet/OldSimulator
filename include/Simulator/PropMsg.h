//
// Filename:        "PropMsg.h"
//
// Description:     This structure describes a property message that
//                  is used as a state variable to keep track if a property
//                  message that was requested has had a response.
//

#ifndef PROPMSG_H
#define PROPMSG_H
#include <vector>
#include <Simulator/Vector.h>

using namespace std;

struct PropMsg
{

  // <data members>
  int 		toID;      //ID of the robot the message was sent to.
  Vector 	frp;  //count of the number of responses summed up so far.
  Vector 	distance;  //distance of the cell who is currently closest to the FCNTR
  float 	radius;
  int 		count;
  bool 		response;  //boolean of whether or not the msg has been received.
  	  	  	  	  	   //msgID as a (long)timestamp to ensure that the message you send out for
  	  	  	  	  	   //a request is the same you're getting back on a response


  // This is the default constructor that initializes
  // this class to its parameterized values
  PropMsg(const int     to         = -1,
          const Vector    grad       = Vector(),
          const Vector    dist       = Vector(),
          const float   rad        = 0.0f,
          const int     num        = 0,
          const bool      answer     = false)
          : toID(to), frp(grad),distance(dist) ,radius(rad),
          count(num), response(answer)
  {
  }//PropMsg(const..{int, int,bool})
}; //PropMsg
#endif
