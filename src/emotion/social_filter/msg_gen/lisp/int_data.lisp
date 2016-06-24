; Auto-generated. Do not edit!


(cl:in-package social_filter-msg)


;//! \htmlinclude int_data.msg.html

(cl:defclass <int_data> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (id_members
    :reader id_members
    :initarg :id_members
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (media_x
    :reader media_x
    :initarg :media_x
    :type cl:float
    :initform 0.0)
   (media_y
    :reader media_y
    :initarg :media_y
    :type cl:float
    :initform 0.0)
   (sd_x
    :reader sd_x
    :initarg :sd_x
    :type cl:float
    :initform 0.0)
   (sd_y
    :reader sd_y
    :initarg :sd_y
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (meet_points
    :reader meet_points
    :initarg :meet_points
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass int_data (<int_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <int_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'int_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name social_filter-msg:<int_data> is deprecated: use social_filter-msg:int_data instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:type-val is deprecated.  Use social_filter-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'id_members-val :lambda-list '(m))
(cl:defmethod id_members-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:id_members-val is deprecated.  Use social_filter-msg:id_members instead.")
  (id_members m))

(cl:ensure-generic-function 'media_x-val :lambda-list '(m))
(cl:defmethod media_x-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:media_x-val is deprecated.  Use social_filter-msg:media_x instead.")
  (media_x m))

(cl:ensure-generic-function 'media_y-val :lambda-list '(m))
(cl:defmethod media_y-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:media_y-val is deprecated.  Use social_filter-msg:media_y instead.")
  (media_y m))

(cl:ensure-generic-function 'sd_x-val :lambda-list '(m))
(cl:defmethod sd_x-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:sd_x-val is deprecated.  Use social_filter-msg:sd_x instead.")
  (sd_x m))

(cl:ensure-generic-function 'sd_y-val :lambda-list '(m))
(cl:defmethod sd_y-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:sd_y-val is deprecated.  Use social_filter-msg:sd_y instead.")
  (sd_y m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:angle-val is deprecated.  Use social_filter-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'meet_points-val :lambda-list '(m))
(cl:defmethod meet_points-val ((m <int_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader social_filter-msg:meet_points-val is deprecated.  Use social_filter-msg:meet_points instead.")
  (meet_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <int_data>) ostream)
  "Serializes a message object of type '<int_data>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'id_members))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'id_members))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'media_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'media_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sd_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sd_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'meet_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'meet_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <int_data>) istream)
  "Deserializes a message object of type '<int_data>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'id_members) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'id_members)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'media_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'media_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sd_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sd_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'meet_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'meet_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<int_data>)))
  "Returns string type for a message object of type '<int_data>"
  "social_filter/int_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'int_data)))
  "Returns string type for a message object of type 'int_data"
  "social_filter/int_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<int_data>)))
  "Returns md5sum for a message object of type '<int_data>"
  "26b7c9c764058b2cf8750f36b3a55953")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'int_data)))
  "Returns md5sum for a message object of type 'int_data"
  "26b7c9c764058b2cf8750f36b3a55953")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<int_data>)))
  "Returns full string definition for message of type '<int_data>"
  (cl:format cl:nil "uint8 type~%uint8[] id_members~%float32 media_x~%float32 media_y~%float32 sd_x~%float32 sd_y~%float32 angle~%float32[] meet_points~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'int_data)))
  "Returns full string definition for message of type 'int_data"
  (cl:format cl:nil "uint8 type~%uint8[] id_members~%float32 media_x~%float32 media_y~%float32 sd_x~%float32 sd_y~%float32 angle~%float32[] meet_points~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <int_data>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'id_members) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'meet_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <int_data>))
  "Converts a ROS message object to a list"
  (cl:list 'int_data
    (cl:cons ':type (type msg))
    (cl:cons ':id_members (id_members msg))
    (cl:cons ':media_x (media_x msg))
    (cl:cons ':media_y (media_y msg))
    (cl:cons ':sd_x (sd_x msg))
    (cl:cons ':sd_y (sd_y msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':meet_points (meet_points msg))
))
