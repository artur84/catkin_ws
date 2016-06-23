; Auto-generated. Do not edit!


(cl:in-package trajectory_simulator-msg)


;//! \htmlinclude TrajectoryObservation.msg.html

(cl:defclass <TrajectoryObservation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass TrajectoryObservation (<TrajectoryObservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryObservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryObservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_simulator-msg:<TrajectoryObservation> is deprecated: use trajectory_simulator-msg:TrajectoryObservation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajectoryObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_simulator-msg:header-val is deprecated.  Use trajectory_simulator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <TrajectoryObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_simulator-msg:object_id-val is deprecated.  Use trajectory_simulator-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <TrajectoryObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_simulator-msg:type-val is deprecated.  Use trajectory_simulator-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TrajectoryObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_simulator-msg:pose-val is deprecated.  Use trajectory_simulator-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <TrajectoryObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_simulator-msg:velocity-val is deprecated.  Use trajectory_simulator-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryObservation>)))
    "Constants for message type '<TrajectoryObservation>"
  '((:FIRST . 1)
    (:LAST . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryObservation)))
    "Constants for message type 'TrajectoryObservation"
  '((:FIRST . 1)
    (:LAST . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryObservation>) ostream)
  "Serializes a message object of type '<TrajectoryObservation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryObservation>) istream)
  "Deserializes a message object of type '<TrajectoryObservation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryObservation>)))
  "Returns string type for a message object of type '<TrajectoryObservation>"
  "trajectory_simulator/TrajectoryObservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryObservation)))
  "Returns string type for a message object of type 'TrajectoryObservation"
  "trajectory_simulator/TrajectoryObservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryObservation>)))
  "Returns md5sum for a message object of type '<TrajectoryObservation>"
  "9a527b2825637f568c9382ecb8750bba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryObservation)))
  "Returns md5sum for a message object of type 'TrajectoryObservation"
  "9a527b2825637f568c9382ecb8750bba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryObservation>)))
  "Returns full string definition for message of type '<TrajectoryObservation>"
  (cl:format cl:nil "uint8 FIRST = 1 #type should be 1 if this observation correspons to an starting point~%uint8 LAST  = 2 #2 if it is the final point of a trajectory~%Header header~%uint32 object_id~%uint8 type~%geometry_msgs/Pose2D pose~%geometry_msgs/Pose2D velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryObservation)))
  "Returns full string definition for message of type 'TrajectoryObservation"
  (cl:format cl:nil "uint8 FIRST = 1 #type should be 1 if this observation correspons to an starting point~%uint8 LAST  = 2 #2 if it is the final point of a trajectory~%Header header~%uint32 object_id~%uint8 type~%geometry_msgs/Pose2D pose~%geometry_msgs/Pose2D velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryObservation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryObservation>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryObservation
    (cl:cons ':header (header msg))
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
))
