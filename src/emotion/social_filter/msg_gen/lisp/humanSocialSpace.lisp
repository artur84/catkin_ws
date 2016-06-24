; Auto-generated. Do not edit!


(cl:in-package social_filter-msg)


;//! \htmlinclude humanSocialSpace.msg.html

(cl:defclass <humanSocialSpace> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (human_id
    :reader human_id
    :initarg :human_id
    :type cl:integer
    :initform 0)
   (size
    :reader size
    :initarg :size
    :type cl:float
    :initform 0.0)
   (sigma_h
    :reader sigma_h
    :initarg :sigma_h
    :type cl:float
    :initform 0.0)
   (sigma_r
    :reader sigma_r
    :initarg :sigma_r
    :type cl:float
    :initform 0.0)
   (sigma_s
    :reader sigma_s
    :initarg :sigma_s
    :type cl:float
    :initform 0.0)
   (attractiveness
    :reader attractiveness
    :initarg :attractiveness
    :type cl:integer
    :initform 0))
)

(cl:defclass humanSocialSpace (<humanSocialSpace>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <humanSocialSpace>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'humanSocialSpace)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name social_filter-msg:<humanSocialSpace> is deprecated: use social_filter-msg:humanSocialSpace instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:header-val is deprecated.  Use social_filter-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:id-val is deprecated.  Use social_filter-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'human_id-val :lambda-list '(m))
(cl:defmethod human_id-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:human_id-val is deprecated.  Use social_filter-msg:human_id instead.")
  (human_id m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:size-val is deprecated.  Use social_filter-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'sigma_h-val :lambda-list '(m))
(cl:defmethod sigma_h-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:sigma_h-val is deprecated.  Use social_filter-msg:sigma_h instead.")
  (sigma_h m))

(cl:ensure-generic-function 'sigma_r-val :lambda-list '(m))
(cl:defmethod sigma_r-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:sigma_r-val is deprecated.  Use social_filter-msg:sigma_r instead.")
  (sigma_r m))

(cl:ensure-generic-function 'sigma_s-val :lambda-list '(m))
(cl:defmethod sigma_s-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:sigma_s-val is deprecated.  Use social_filter-msg:sigma_s instead.")
  (sigma_s m))

(cl:ensure-generic-function 'attractiveness-val :lambda-list '(m))
(cl:defmethod attractiveness-val ((m <humanSocialSpace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:attractiveness-val is deprecated.  Use social_filter-msg:attractiveness instead.")
  (attractiveness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <humanSocialSpace>) ostream)
  "Serializes a message object of type '<humanSocialSpace>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'human_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sigma_h))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sigma_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sigma_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'attractiveness)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <humanSocialSpace>) istream)
  "Deserializes a message object of type '<humanSocialSpace>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'human_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sigma_h) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sigma_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sigma_s) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'attractiveness) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<humanSocialSpace>)))
  "Returns string type for a message object of type '<humanSocialSpace>"
  "social_filter/humanSocialSpace")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'humanSocialSpace)))
  "Returns string type for a message object of type 'humanSocialSpace"
  "social_filter/humanSocialSpace")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<humanSocialSpace>)))
  "Returns md5sum for a message object of type '<humanSocialSpace>"
  "69492c24f08da63cba4bfedf7601f9af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'humanSocialSpace)))
  "Returns md5sum for a message object of type 'humanSocialSpace"
  "69492c24f08da63cba4bfedf7601f9af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<humanSocialSpace>)))
  "Returns full string definition for message of type '<humanSocialSpace>"
  (cl:format cl:nil "Header header~%~%int32 id~%int32 human_id~%float32 size~%float32 sigma_h~%float32 sigma_r~%float32 sigma_s~%int32   attractiveness~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'humanSocialSpace)))
  "Returns full string definition for message of type 'humanSocialSpace"
  (cl:format cl:nil "Header header~%~%int32 id~%int32 human_id~%float32 size~%float32 sigma_h~%float32 sigma_r~%float32 sigma_s~%int32   attractiveness~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <humanSocialSpace>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <humanSocialSpace>))
  "Converts a ROS message object to a list"
  (cl:list 'humanSocialSpace
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':human_id (human_id msg))
    (cl:cons ':size (size msg))
    (cl:cons ':sigma_h (sigma_h msg))
    (cl:cons ':sigma_r (sigma_r msg))
    (cl:cons ':sigma_s (sigma_s msg))
    (cl:cons ':attractiveness (attractiveness msg))
))
