; Auto-generated. Do not edit!


(cl:in-package social_filter-msg)


;//! \htmlinclude humanPoses.msg.html

(cl:defclass <humanPoses> (roslisp-msg-protocol:ros-message)
  ((humans
    :reader humans
    :initarg :humans
    :type (cl:vector social_filter-msg:humanPose)
   :initform (cl:make-array 0 :element-type 'social_filter-msg:humanPose :initial-element (cl:make-instance 'social_filter-msg:humanPose))))
)

(cl:defclass humanPoses (<humanPoses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <humanPoses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'humanPoses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name social_filter-msg:<humanPoses> is deprecated: use social_filter-msg:humanPoses instead.")))

(cl:ensure-generic-function 'humans-val :lambda-list '(m))
(cl:defmethod humans-val ((m <humanPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:humans-val is deprecated.  Use social_filter-msg:humans instead.")
  (humans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <humanPoses>) ostream)
  "Serializes a message object of type '<humanPoses>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'humans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'humans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <humanPoses>) istream)
  "Deserializes a message object of type '<humanPoses>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'humans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'humans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'social_filter-msg:humanPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<humanPoses>)))
  "Returns string type for a message object of type '<humanPoses>"
  "social_filter/humanPoses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'humanPoses)))
  "Returns string type for a message object of type 'humanPoses"
  "social_filter/humanPoses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<humanPoses>)))
  "Returns md5sum for a message object of type '<humanPoses>"
  "af15ed28aa6352fb91773931f13957ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'humanPoses)))
  "Returns md5sum for a message object of type 'humanPoses"
  "af15ed28aa6352fb91773931f13957ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<humanPoses>)))
  "Returns full string definition for message of type '<humanPoses>"
  (cl:format cl:nil "humanPose[] humans~%================================================================================~%MSG: social_filter/humanPose~%Header header~%~%int32 id~%float32 x~%float32 y~%float32 theta~%float32 linear_velocity~%float32 angular_velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'humanPoses)))
  "Returns full string definition for message of type 'humanPoses"
  (cl:format cl:nil "humanPose[] humans~%================================================================================~%MSG: social_filter/humanPose~%Header header~%~%int32 id~%float32 x~%float32 y~%float32 theta~%float32 linear_velocity~%float32 angular_velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <humanPoses>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'humans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <humanPoses>))
  "Converts a ROS message object to a list"
  (cl:list 'humanPoses
    (cl:cons ':humans (humans msg))
))
