; Auto-generated. Do not edit!


(cl:in-package social_filter-msg)


;//! \htmlinclude humanSocialSpaces.msg.html

(cl:defclass <humanSocialSpaces> (roslisp-msg-protocol:ros-message)
  ((socialSpaces
    :reader socialSpaces
    :initarg :socialSpaces
    :type (cl:vector social_filter-msg:humanSocialSpace)
   :initform (cl:make-array 0 :element-type 'social_filter-msg:humanSocialSpace :initial-element (cl:make-instance 'social_filter-msg:humanSocialSpace))))
)

(cl:defclass humanSocialSpaces (<humanSocialSpaces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <humanSocialSpaces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'humanSocialSpaces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name social_filter-msg:<humanSocialSpaces> is deprecated: use social_filter-msg:humanSocialSpaces instead.")))

(cl:ensure-generic-function 'socialSpaces-val :lambda-list '(m))
(cl:defmethod socialSpaces-val ((m <humanSocialSpaces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:socialSpaces-val is deprecated.  Use social_filter-msg:socialSpaces instead.")
  (socialSpaces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <humanSocialSpaces>) ostream)
  "Serializes a message object of type '<humanSocialSpaces>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'socialSpaces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'socialSpaces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <humanSocialSpaces>) istream)
  "Deserializes a message object of type '<humanSocialSpaces>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'socialSpaces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'socialSpaces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'social_filter-msg:humanSocialSpace))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<humanSocialSpaces>)))
  "Returns string type for a message object of type '<humanSocialSpaces>"
  "social_filter/humanSocialSpaces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'humanSocialSpaces)))
  "Returns string type for a message object of type 'humanSocialSpaces"
  "social_filter/humanSocialSpaces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<humanSocialSpaces>)))
  "Returns md5sum for a message object of type '<humanSocialSpaces>"
  "5f9825a90c7f72b892aaab742bafc102")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'humanSocialSpaces)))
  "Returns md5sum for a message object of type 'humanSocialSpaces"
  "5f9825a90c7f72b892aaab742bafc102")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<humanSocialSpaces>)))
  "Returns full string definition for message of type '<humanSocialSpaces>"
  (cl:format cl:nil "humanSocialSpace[] socialSpaces~%~%================================================================================~%MSG: social_filter/humanSocialSpace~%Header header~%~%int32 id~%int32 human_id~%float32 size~%float32 sigma_h~%float32 sigma_r~%float32 sigma_s~%int32   attractiveness~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'humanSocialSpaces)))
  "Returns full string definition for message of type 'humanSocialSpaces"
  (cl:format cl:nil "humanSocialSpace[] socialSpaces~%~%================================================================================~%MSG: social_filter/humanSocialSpace~%Header header~%~%int32 id~%int32 human_id~%float32 size~%float32 sigma_h~%float32 sigma_r~%float32 sigma_s~%int32   attractiveness~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <humanSocialSpaces>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'socialSpaces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <humanSocialSpaces>))
  "Converts a ROS message object to a list"
  (cl:list 'humanSocialSpaces
    (cl:cons ':socialSpaces (socialSpaces msg))
))
