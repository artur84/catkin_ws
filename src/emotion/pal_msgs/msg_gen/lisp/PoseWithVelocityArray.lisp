; Auto-generated. Do not edit!


(cl:in-package pal_msgs-msg)


;//! \htmlinclude PoseWithVelocityArray.msg.html

(cl:defclass <PoseWithVelocityArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector pal_msgs-msg:PoseWithVelocity)
   :initform (cl:make-array 0 :element-type 'pal_msgs-msg:PoseWithVelocity :initial-element (cl:make-instance 'pal_msgs-msg:PoseWithVelocity))))
)

(cl:defclass PoseWithVelocityArray (<PoseWithVelocityArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseWithVelocityArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseWithVelocityArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_msgs-msg:<PoseWithVelocityArray> is deprecated: use pal_msgs-msg:PoseWithVelocityArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PoseWithVelocityArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_msgs-msg:header-val is deprecated.  Use pal_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <PoseWithVelocityArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_msgs-msg:poses-val is deprecated.  Use pal_msgs-msg:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseWithVelocityArray>) ostream)
  "Serializes a message object of type '<PoseWithVelocityArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseWithVelocityArray>) istream)
  "Deserializes a message object of type '<PoseWithVelocityArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pal_msgs-msg:PoseWithVelocity))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseWithVelocityArray>)))
  "Returns string type for a message object of type '<PoseWithVelocityArray>"
  "pal_msgs/PoseWithVelocityArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseWithVelocityArray)))
  "Returns string type for a message object of type 'PoseWithVelocityArray"
  "pal_msgs/PoseWithVelocityArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseWithVelocityArray>)))
  "Returns md5sum for a message object of type '<PoseWithVelocityArray>"
  "bec2ec3b87f0fff0279f4d17d9bc574d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseWithVelocityArray)))
  "Returns md5sum for a message object of type 'PoseWithVelocityArray"
  "bec2ec3b87f0fff0279f4d17d9bc574d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseWithVelocityArray>)))
  "Returns full string definition for message of type '<PoseWithVelocityArray>"
  (cl:format cl:nil "Header header~%PoseWithVelocity[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pal_msgs/PoseWithVelocity~%Header header~%geometry_msgs/Pose2D pose~%float64 vx~%float64 vy~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseWithVelocityArray)))
  "Returns full string definition for message of type 'PoseWithVelocityArray"
  (cl:format cl:nil "Header header~%PoseWithVelocity[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pal_msgs/PoseWithVelocity~%Header header~%geometry_msgs/Pose2D pose~%float64 vx~%float64 vy~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseWithVelocityArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseWithVelocityArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseWithVelocityArray
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
))
