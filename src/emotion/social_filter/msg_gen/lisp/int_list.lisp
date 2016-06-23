; Auto-generated. Do not edit!


(cl:in-package social_filter-msg)


;//! \htmlinclude int_list.msg.html

(cl:defclass <int_list> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (formation
    :reader formation
    :initarg :formation
    :type (cl:vector social_filter-msg:int_data)
   :initform (cl:make-array 0 :element-type 'social_filter-msg:int_data :initial-element (cl:make-instance 'social_filter-msg:int_data))))
)

(cl:defclass int_list (<int_list>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int_list>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int_list)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name social_filter-msg:<int_list> is deprecated: use social_filter-msg:int_list instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <int_list>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:header-val is deprecated.  Use social_filter-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'formation-val :lambda-list '(m))
(cl:defmethod formation-val ((m <int_list>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:formation-val is deprecated.  Use social_filter-msg:formation instead.")
  (formation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int_list>) ostream)
  "Serializes a message object of type '<int_list>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'formation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'formation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int_list>) istream)
  "Deserializes a message object of type '<int_list>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'formation) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'formation)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'social_filter-msg:int_data))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int_list>)))
  "Returns string type for a message object of type '<int_list>"
  "social_filter/int_list")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int_list)))
  "Returns string type for a message object of type 'int_list"
  "social_filter/int_list")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int_list>)))
  "Returns md5sum for a message object of type '<int_list>"
  "8c6dca5d36f6382490c6b1f8163c65b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int_list)))
  "Returns md5sum for a message object of type 'int_list"
  "8c6dca5d36f6382490c6b1f8163c65b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int_list>)))
  "Returns full string definition for message of type '<int_list>"
  (cl:format cl:nil "Header header~%int_data[] formation ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: social_filter/int_data~%uint8 type~%uint8[] id_members~%float32 media_x~%float32 media_y~%float32 sd_x~%float32 sd_y~%float32 angle~%float32[] meet_points~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int_list)))
  "Returns full string definition for message of type 'int_list"
  (cl:format cl:nil "Header header~%int_data[] formation ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: social_filter/int_data~%uint8 type~%uint8[] id_members~%float32 media_x~%float32 media_y~%float32 sd_x~%float32 sd_y~%float32 angle~%float32[] meet_points~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int_list>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'formation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int_list>))
  "Converts a ROS message object to a list"
  (cl:list 'int_list
    (cl:cons ':header (header msg))
    (cl:cons ':formation (formation msg))
))
