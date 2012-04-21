; Auto-generated. Do not edit!


(cl:in-package Simulator-msg)


;//! \htmlinclude FormationMessage.msg.html

(cl:defclass <FormationMessage> (roslisp-msg-protocol:ros-message)
  ((radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (seed_id
    :reader seed_id
    :initarg :seed_id
    :type cl:integer
    :initform 0)
   (formation_id
    :reader formation_id
    :initarg :formation_id
    :type cl:integer
    :initform 0))
)

(cl:defclass FormationMessage (<FormationMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FormationMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FormationMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-msg:<FormationMessage> is deprecated: use Simulator-msg:FormationMessage instead.")))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <FormationMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:radius-val is deprecated.  Use Simulator-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <FormationMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:heading-val is deprecated.  Use Simulator-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'seed_id-val :lambda-list '(m))
(cl:defmethod seed_id-val ((m <FormationMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:seed_id-val is deprecated.  Use Simulator-msg:seed_id instead.")
  (seed_id m))

(cl:ensure-generic-function 'formation_id-val :lambda-list '(m))
(cl:defmethod formation_id-val ((m <FormationMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:formation_id-val is deprecated.  Use Simulator-msg:formation_id instead.")
  (formation_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FormationMessage>) ostream)
  "Serializes a message object of type '<FormationMessage>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'seed_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'formation_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FormationMessage>) istream)
  "Deserializes a message object of type '<FormationMessage>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seed_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'formation_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FormationMessage>)))
  "Returns string type for a message object of type '<FormationMessage>"
  "Simulator/FormationMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FormationMessage)))
  "Returns string type for a message object of type 'FormationMessage"
  "Simulator/FormationMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FormationMessage>)))
  "Returns md5sum for a message object of type '<FormationMessage>"
  "bd72cb9b2bb0c98007298d13c50d1bb0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FormationMessage)))
  "Returns md5sum for a message object of type 'FormationMessage"
  "bd72cb9b2bb0c98007298d13c50d1bb0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FormationMessage>)))
  "Returns full string definition for message of type '<FormationMessage>"
  (cl:format cl:nil "float64 radius~%float64 heading~%#Vector  seed_frp~%int32   seed_id~%int32   formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FormationMessage)))
  "Returns full string definition for message of type 'FormationMessage"
  (cl:format cl:nil "float64 radius~%float64 heading~%#Vector  seed_frp~%int32   seed_id~%int32   formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FormationMessage>))
  (cl:+ 0
     8
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FormationMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'FormationMessage
    (cl:cons ':radius (radius msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':seed_id (seed_id msg))
    (cl:cons ':formation_id (formation_id msg))
))
