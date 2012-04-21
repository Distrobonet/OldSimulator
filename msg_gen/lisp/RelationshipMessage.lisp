; Auto-generated. Do not edit!


(cl:in-package Simulator-msg)


;//! \htmlinclude RelationshipMessage.msg.html

(cl:defclass <RelationshipMessage> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass RelationshipMessage (<RelationshipMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelationshipMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelationshipMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Simulator-msg:<RelationshipMessage> is deprecated: use Simulator-msg:RelationshipMessage instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RelationshipMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Simulator-msg:id-val is deprecated.  Use Simulator-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelationshipMessage>) ostream)
  "Serializes a message object of type '<RelationshipMessage>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelationshipMessage>) istream)
  "Deserializes a message object of type '<RelationshipMessage>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelationshipMessage>)))
  "Returns string type for a message object of type '<RelationshipMessage>"
  "Simulator/RelationshipMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelationshipMessage)))
  "Returns string type for a message object of type 'RelationshipMessage"
  "Simulator/RelationshipMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelationshipMessage>)))
  "Returns md5sum for a message object of type '<RelationshipMessage>"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelationshipMessage)))
  "Returns md5sum for a message object of type 'RelationshipMessage"
  "c5e4a7d59c68f74eabcec876a00216aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelationshipMessage>)))
  "Returns full string definition for message of type '<RelationshipMessage>"
  (cl:format cl:nil "#Vector desired~%#Vector actual~%int32 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelationshipMessage)))
  "Returns full string definition for message of type 'RelationshipMessage"
  (cl:format cl:nil "#Vector desired~%#Vector actual~%int32 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelationshipMessage>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelationshipMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'RelationshipMessage
    (cl:cons ':id (id msg))
))
