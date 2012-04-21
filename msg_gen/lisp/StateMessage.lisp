; Auto-generated. Do not edit!


(cl:in-package Simulator-msg)


;//! \htmlinclude StateMessage.msg.html

(cl:defclass <StateMessage> (roslisp-msg-protocol:ros-message)
  ((angular_error
    :reader angular_error
    :initarg :angular_error
    :type cl:float
    :initform 0.0)
   (timestep
    :reader timestep
    :initarg :timestep
    :type cl:integer
    :initform 0)
   (reference_id
    :reader reference_id
    :initarg :reference_id
    :type cl:integer
    :initform 0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (heat
    :reader heat
    :initarg :heat
    :type cl:float
    :initform 0.0))
)

(cl:defclass StateMessage (<StateMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-msg:<StateMessage> is deprecated: use Simulator-msg:StateMessage instead.")))

(cl:ensure-generic-function 'angular_error-val :lambda-list '(m))
(cl:defmethod angular_error-val ((m <StateMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:angular_error-val is deprecated.  Use Simulator-msg:angular_error instead.")
  (angular_error m))

(cl:ensure-generic-function 'timestep-val :lambda-list '(m))
(cl:defmethod timestep-val ((m <StateMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:timestep-val is deprecated.  Use Simulator-msg:timestep instead.")
  (timestep m))

(cl:ensure-generic-function 'reference_id-val :lambda-list '(m))
(cl:defmethod reference_id-val ((m <StateMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:reference_id-val is deprecated.  Use Simulator-msg:reference_id instead.")
  (reference_id m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <StateMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:temperature-val is deprecated.  Use Simulator-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'heat-val :lambda-list '(m))
(cl:defmethod heat-val ((m <StateMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:heat-val is deprecated.  Use Simulator-msg:heat instead.")
  (heat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateMessage>) ostream)
  "Serializes a message object of type '<StateMessage>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'timestep)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'reference_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateMessage>) istream)
  "Deserializes a message object of type '<StateMessage>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestep) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reference_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heat) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateMessage>)))
  "Returns string type for a message object of type '<StateMessage>"
  "Simulator/StateMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateMessage)))
  "Returns string type for a message object of type 'StateMessage"
  "Simulator/StateMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateMessage>)))
  "Returns md5sum for a message object of type '<StateMessage>"
  "199c3c439236ac8d1a99f8080aafbf52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateMessage)))
  "Returns md5sum for a message object of type 'StateMessage"
  "199c3c439236ac8d1a99f8080aafbf52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateMessage>)))
  "Returns full string definition for message of type '<StateMessage>"
  (cl:format cl:nil "#FormationMessage formation~%#VectorMessage frp~%#Relationship[] relationships~%#VectorMessage linear_error~%float64 angular_error~%int32 timestep~%int32 reference_id~%float64 temperature~%float64 heat~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateMessage)))
  "Returns full string definition for message of type 'StateMessage"
  (cl:format cl:nil "#FormationMessage formation~%#VectorMessage frp~%#Relationship[] relationships~%#VectorMessage linear_error~%float64 angular_error~%int32 timestep~%int32 reference_id~%float64 temperature~%float64 heat~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateMessage>))
  (cl:+ 0
     8
     4
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'StateMessage
    (cl:cons ':angular_error (angular_error msg))
    (cl:cons ':timestep (timestep msg))
    (cl:cons ':reference_id (reference_id msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':heat (heat msg))
))
