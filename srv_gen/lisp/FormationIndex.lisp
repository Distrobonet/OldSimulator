; Auto-generated. Do not edit!


(cl:in-package Simulator-srv)


;//! \htmlinclude FormationIndex-request.msg.html

(cl:defclass <FormationIndex-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FormationIndex-request (<FormationIndex-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FormationIndex-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FormationIndex-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<FormationIndex-request> is deprecated: use Simulator-srv:FormationIndex-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FormationIndex-request>) ostream)
  "Serializes a message object of type '<FormationIndex-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FormationIndex-request>) istream)
  "Deserializes a message object of type '<FormationIndex-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FormationIndex-request>)))
  "Returns string type for a service object of type '<FormationIndex-request>"
  "Simulator/FormationIndexRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FormationIndex-request)))
  "Returns string type for a service object of type 'FormationIndex-request"
  "Simulator/FormationIndexRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FormationIndex-request>)))
  "Returns md5sum for a message object of type '<FormationIndex-request>"
  "3e0302d241547a13e5cebd81ed40641e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FormationIndex-request)))
  "Returns md5sum for a message object of type 'FormationIndex-request"
  "3e0302d241547a13e5cebd81ed40641e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FormationIndex-request>)))
  "Returns full string definition for message of type '<FormationIndex-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FormationIndex-request)))
  "Returns full string definition for message of type 'FormationIndex-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FormationIndex-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FormationIndex-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FormationIndex-request
))
;//! \htmlinclude FormationIndex-response.msg.html

(cl:defclass <FormationIndex-response> (roslisp-msg-protocol:ros-message)
  ((formationIndex
    :reader formationIndex
    :initarg :formationIndex
    :type cl:integer
    :initform 0))
)

(cl:defclass FormationIndex-response (<FormationIndex-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FormationIndex-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FormationIndex-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-srv:<FormationIndex-response> is deprecated: use Simulator-srv:FormationIndex-response instead.")))

(cl:ensure-generic-function 'formationIndex-val :lambda-list '(m))
(cl:defmethod formationIndex-val ((m <FormationIndex-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-srv:formationIndex-val is deprecated.  Use Simulator-srv:formationIndex instead.")
  (formationIndex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FormationIndex-response>) ostream)
  "Serializes a message object of type '<FormationIndex-response>"
  (cl:let* ((signed (cl:slot-value msg 'formationIndex)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FormationIndex-response>) istream)
  "Deserializes a message object of type '<FormationIndex-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'formationIndex) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FormationIndex-response>)))
  "Returns string type for a service object of type '<FormationIndex-response>"
  "Simulator/FormationIndexResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FormationIndex-response)))
  "Returns string type for a service object of type 'FormationIndex-response"
  "Simulator/FormationIndexResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FormationIndex-response>)))
  "Returns md5sum for a message object of type '<FormationIndex-response>"
  "3e0302d241547a13e5cebd81ed40641e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FormationIndex-response)))
  "Returns md5sum for a message object of type 'FormationIndex-response"
  "3e0302d241547a13e5cebd81ed40641e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FormationIndex-response>)))
  "Returns full string definition for message of type '<FormationIndex-response>"
  (cl:format cl:nil "int64 formationIndex~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FormationIndex-response)))
  "Returns full string definition for message of type 'FormationIndex-response"
  (cl:format cl:nil "int64 formationIndex~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FormationIndex-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FormationIndex-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FormationIndex-response
    (cl:cons ':formationIndex (formationIndex msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FormationIndex)))
  'FormationIndex-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FormationIndex)))
  'FormationIndex-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FormationIndex)))
  "Returns string type for a service object of type '<FormationIndex>"
  "Simulator/FormationIndex")