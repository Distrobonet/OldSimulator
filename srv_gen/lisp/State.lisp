; Auto-generated. Do not edit!


(cl:in-package Simulator-srv)


;//! \htmlinclude State-request.msg.html

(cl:defclass <State-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass State-request (<State-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<State-request> is deprecated: use Simulator-srv:State-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State-request>) ostream)
  "Serializes a message object of type '<State-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State-request>) istream)
  "Deserializes a message object of type '<State-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State-request>)))
  "Returns string type for a service object of type '<State-request>"
  "Simulator/StateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State-request)))
  "Returns string type for a service object of type 'State-request"
  "Simulator/StateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State-request>)))
  "Returns md5sum for a message object of type '<State-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State-request)))
  "Returns md5sum for a message object of type 'State-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State-request>)))
  "Returns full string definition for message of type '<State-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State-request)))
  "Returns full string definition for message of type 'State-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State-request>))
  "Converts a ROS message object to a list"
  (cl:list 'State-request
))
;//! \htmlinclude State-response.msg.html

(cl:defclass <State-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass State-response (<State-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<State-response> is deprecated: use Simulator-srv:State-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State-response>) ostream)
  "Serializes a message object of type '<State-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State-response>) istream)
  "Deserializes a message object of type '<State-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State-response>)))
  "Returns string type for a service object of type '<State-response>"
  "Simulator/StateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State-response)))
  "Returns string type for a service object of type 'State-response"
  "Simulator/StateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State-response>)))
  "Returns md5sum for a message object of type '<State-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State-response)))
  "Returns md5sum for a message object of type 'State-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State-response>)))
  "Returns full string definition for message of type '<State-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State-response)))
  "Returns full string definition for message of type 'State-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State-response>))
  "Converts a ROS message object to a list"
  (cl:list 'State-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'State)))
  'State-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'State)))
  'State-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State)))
  "Returns string type for a service object of type '<State>"
  "Simulator/State")