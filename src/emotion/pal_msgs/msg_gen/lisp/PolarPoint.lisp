; Auto-generated. Do not edit!


(cl:in-package pal_msgs-msg)


;//! \htmlinclude PolarPoint.msg.html

(cl:defclass <PolarPoint> (roslisp-msg-protocol:ros-message)
  ((rho
    :reader rho
    :initarg :rho
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass PolarPoint (<PolarPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolarPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolarPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_msgs-msg:<PolarPoint> is deprecated: use pal_msgs-msg:PolarPoint instead.")))

(cl:ensure-generic-function 'rho-val :lambda-list '(m))
(cl:defmethod rho-val ((m <PolarPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_msgs-msg:rho-val is deprecated.  Use pal_msgs-msg:rho instead.")
  (rho m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <PolarPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_msgs-msg:theta-val is deprecated.  Use pal_msgs-msg:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolarPoint>) ostream)
  "Serializes a message object of type '<PolarPoint>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rho))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolarPoint>) istream)
  "Deserializes a message object of type '<PolarPoint>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rho) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolarPoint>)))
  "Returns string type for a message object of type '<PolarPoint>"
  "pal_msgs/PolarPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolarPoint)))
  "Returns string type for a message object of type 'PolarPoint"
  "pal_msgs/PolarPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolarPoint>)))
  "Returns md5sum for a message object of type '<PolarPoint>"
  "a880a9e05853650b5374502b1e3413e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolarPoint)))
  "Returns md5sum for a message object of type 'PolarPoint"
  "a880a9e05853650b5374502b1e3413e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolarPoint>)))
  "Returns full string definition for message of type '<PolarPoint>"
  (cl:format cl:nil "float64 rho~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolarPoint)))
  "Returns full string definition for message of type 'PolarPoint"
  (cl:format cl:nil "float64 rho~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolarPoint>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolarPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'PolarPoint
    (cl:cons ':rho (rho msg))
    (cl:cons ':theta (theta msg))
))
