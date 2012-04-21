; Auto-generated. Do not edit!


(cl:in-package Simulator-srv)


;//! \htmlinclude CurrentFormation-request.msg.html

(cl:defclass <CurrentFormation-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CurrentFormation-request (<CurrentFormation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CurrentFormation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CurrentFormation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<CurrentFormation-request> is deprecated: use Simulator-srv:CurrentFormation-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CurrentFormation-request>) ostream)
  "Serializes a message object of type '<CurrentFormation-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CurrentFormation-request>) istream)
  "Deserializes a message object of type '<CurrentFormation-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CurrentFormation-request>)))
  "Returns string type for a service object of type '<CurrentFormation-request>"
  "Simulator/CurrentFormationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CurrentFormation-request)))
  "Returns string type for a service object of type 'CurrentFormation-request"
  "Simulator/CurrentFormationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CurrentFormation-request>)))
  "Returns md5sum for a message object of type '<CurrentFormation-request>"
  "1b8c322a6316abec4738c71c0df103d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CurrentFormation-request)))
  "Returns md5sum for a message object of type 'CurrentFormation-request"
  "1b8c322a6316abec4738c71c0df103d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CurrentFormation-request>)))
  "Returns full string definition for message of type '<CurrentFormation-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CurrentFormation-request)))
  "Returns full string definition for message of type 'CurrentFormation-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CurrentFormation-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CurrentFormation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CurrentFormation-request
))
;//! \htmlinclude CurrentFormation-response.msg.html

(cl:defclass <CurrentFormation-response> (roslisp-msg-protocol:ros-message)
  ((formation
    :reader formation
    :initarg :formation
    :type Simulator-msg:FormationMessage
    :initform (cl:make-instance 'Simulator-msg:FormationMessage)))
)

(cl:defclass CurrentFormation-response (<CurrentFormation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CurrentFormation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CurrentFormation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<CurrentFormation-response> is deprecated: use Simulator-srv:CurrentFormation-response instead.")))

(cl:ensure-generic-function 'formation-val :lambda-list '(m))
(cl:defmethod formation-val ((m <CurrentFormation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-srv:formation-val is deprecated.  Use Simulator-srv:formation instead.")
  (formation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CurrentFormation-response>) ostream)
  "Serializes a message object of type '<CurrentFormation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'formation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CurrentFormation-response>) istream)
  "Deserializes a message object of type '<CurrentFormation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'formation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CurrentFormation-response>)))
  "Returns string type for a service object of type '<CurrentFormation-response>"
  "Simulator/CurrentFormationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CurrentFormation-response)))
  "Returns string type for a service object of type 'CurrentFormation-response"
  "Simulator/CurrentFormationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CurrentFormation-response>)))
  "Returns md5sum for a message object of type '<CurrentFormation-response>"
  "1b8c322a6316abec4738c71c0df103d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CurrentFormation-response)))
  "Returns md5sum for a message object of type 'CurrentFormation-response"
  "1b8c322a6316abec4738c71c0df103d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CurrentFormation-response>)))
  "Returns full string definition for message of type '<CurrentFormation-response>"
  (cl:format cl:nil "FormationMessage formation~%~%================================================================================~%MSG: Simulator/FormationMessage~%float64 radius~%float64 heading~%VectorMessage  seed_frp~%int32   seed_id~%int32   formation_id~%================================================================================~%MSG: Simulator/VectorMessage~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CurrentFormation-response)))
  "Returns full string definition for message of type 'CurrentFormation-response"
  (cl:format cl:nil "FormationMessage formation~%~%================================================================================~%MSG: Simulator/FormationMessage~%float64 radius~%float64 heading~%VectorMessage  seed_frp~%int32   seed_id~%int32   formation_id~%================================================================================~%MSG: Simulator/VectorMessage~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CurrentFormation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'formation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CurrentFormation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CurrentFormation-response
    (cl:cons ':formation (formation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CurrentFormation)))
  'CurrentFormation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CurrentFormation)))
  'CurrentFormation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CurrentFormation)))
  "Returns string type for a service object of type '<CurrentFormation>"
  "Simulator/CurrentFormation")