; Auto-generated. Do not edit!


(cl:in-package ghmm-msg)


;//! \htmlinclude ProbabilisticGrid.msg.html

(cl:defclass <ProbabilisticGrid> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (info
    :reader info
    :initarg :info
    :type nav_msgs-msg:MapMetaData
    :initform (cl:make-instance 'nav_msgs-msg:MapMetaData))
   (time_step
    :reader time_step
    :initarg :time_step
    :type cl:real
    :initform 0)
   (entity
    :reader entity
    :initarg :entity
    :type cl:fixnum
    :initform 0)
   (n_steps
    :reader n_steps
    :initarg :n_steps
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ProbabilisticGrid (<ProbabilisticGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProbabilisticGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProbabilisticGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ghmm-msg:<ProbabilisticGrid> is deprecated: use ghmm-msg:ProbabilisticGrid instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:header-val is deprecated.  Use ghmm-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:info-val is deprecated.  Use ghmm-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'time_step-val :lambda-list '(m))
(cl:defmethod time_step-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:time_step-val is deprecated.  Use ghmm-msg:time_step instead.")
  (time_step m))

(cl:ensure-generic-function 'entity-val :lambda-list '(m))
(cl:defmethod entity-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:entity-val is deprecated.  Use ghmm-msg:entity instead.")
  (entity m))

(cl:ensure-generic-function 'n_steps-val :lambda-list '(m))
(cl:defmethod n_steps-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:n_steps-val is deprecated.  Use ghmm-msg:n_steps instead.")
  (n_steps m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ProbabilisticGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ghmm-msg:data-val is deprecated.  Use ghmm-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProbabilisticGrid>) ostream)
  "Serializes a message object of type '<ProbabilisticGrid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'info) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_step)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_step) (cl:floor (cl:slot-value msg 'time_step)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'entity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'n_steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProbabilisticGrid>) istream)
  "Deserializes a message object of type '<ProbabilisticGrid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'info) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_step) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'entity) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_steps) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProbabilisticGrid>)))
  "Returns string type for a message object of type '<ProbabilisticGrid>"
  "ghmm/ProbabilisticGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProbabilisticGrid)))
  "Returns string type for a message object of type 'ProbabilisticGrid"
  "ghmm/ProbabilisticGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProbabilisticGrid>)))
  "Returns md5sum for a message object of type '<ProbabilisticGrid>"
  "314d5cf59a894d0c0bb26f180ce40072")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProbabilisticGrid)))
  "Returns md5sum for a message object of type 'ProbabilisticGrid"
  "314d5cf59a894d0c0bb26f180ce40072")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProbabilisticGrid>)))
  "Returns full string definition for message of type '<ProbabilisticGrid>"
  (cl:format cl:nil "Header header ~%nav_msgs/MapMetaData info~%duration time_step~%int16 entity~%int8 n_steps~%float32[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProbabilisticGrid)))
  "Returns full string definition for message of type 'ProbabilisticGrid"
  (cl:format cl:nil "Header header ~%nav_msgs/MapMetaData info~%duration time_step~%int16 entity~%int8 n_steps~%float32[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProbabilisticGrid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'info))
     8
     2
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProbabilisticGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'ProbabilisticGrid
    (cl:cons ':header (header msg))
    (cl:cons ':info (info msg))
    (cl:cons ':time_step (time_step msg))
    (cl:cons ':entity (entity msg))
    (cl:cons ':n_steps (n_steps msg))
    (cl:cons ':data (data msg))
))
