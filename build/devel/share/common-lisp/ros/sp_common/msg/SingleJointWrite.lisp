; Auto-generated. Do not edit!


(cl:in-package sp_common-msg)


;//! \htmlinclude SingleJointWrite.msg.html

(cl:defclass <SingleJointWrite> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:float
    :initform 0.0)
   (num
    :reader num
    :initarg :num
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SingleJointWrite (<SingleJointWrite>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SingleJointWrite>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SingleJointWrite)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sp_common-msg:<SingleJointWrite> is deprecated: use sp_common-msg:SingleJointWrite instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SingleJointWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sp_common-msg:state-val is deprecated.  Use sp_common-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <SingleJointWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sp_common-msg:num-val is deprecated.  Use sp_common-msg:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SingleJointWrite>) ostream)
  "Serializes a message object of type '<SingleJointWrite>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SingleJointWrite>) istream)
  "Deserializes a message object of type '<SingleJointWrite>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'state) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SingleJointWrite>)))
  "Returns string type for a message object of type '<SingleJointWrite>"
  "sp_common/SingleJointWrite")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SingleJointWrite)))
  "Returns string type for a message object of type 'SingleJointWrite"
  "sp_common/SingleJointWrite")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SingleJointWrite>)))
  "Returns md5sum for a message object of type '<SingleJointWrite>"
  "a8df87d7f5c9d7fe4d040df6e38c801b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SingleJointWrite)))
  "Returns md5sum for a message object of type 'SingleJointWrite"
  "a8df87d7f5c9d7fe4d040df6e38c801b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SingleJointWrite>)))
  "Returns full string definition for message of type '<SingleJointWrite>"
  (cl:format cl:nil "# This message is for....~%~%float64 state~%uint16 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SingleJointWrite)))
  "Returns full string definition for message of type 'SingleJointWrite"
  (cl:format cl:nil "# This message is for....~%~%float64 state~%uint16 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SingleJointWrite>))
  (cl:+ 0
     8
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SingleJointWrite>))
  "Converts a ROS message object to a list"
  (cl:list 'SingleJointWrite
    (cl:cons ':state (state msg))
    (cl:cons ':num (num msg))
))
