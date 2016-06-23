; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude EnableControl-request.msg.html

(cl:defclass <EnableControl-request> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EnableControl-request (<EnableControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<EnableControl-request> is deprecated: use controller-srv:EnableControl-request instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <EnableControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:enabled-val is deprecated.  Use controller-srv:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableControl-request>) ostream)
  "Serializes a message object of type '<EnableControl-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableControl-request>) istream)
  "Deserializes a message object of type '<EnableControl-request>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableControl-request>)))
  "Returns string type for a service object of type '<EnableControl-request>"
  "controller/EnableControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableControl-request)))
  "Returns string type for a service object of type 'EnableControl-request"
  "controller/EnableControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableControl-request>)))
  "Returns md5sum for a message object of type '<EnableControl-request>"
  "7badc0803f167fb2cf52d5b259ad3ce0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableControl-request)))
  "Returns md5sum for a message object of type 'EnableControl-request"
  "7badc0803f167fb2cf52d5b259ad3ce0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableControl-request>)))
  "Returns full string definition for message of type '<EnableControl-request>"
  (cl:format cl:nil "bool enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableControl-request)))
  "Returns full string definition for message of type 'EnableControl-request"
  (cl:format cl:nil "bool enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableControl-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableControl-request
    (cl:cons ':enabled (enabled msg))
))
;//! \htmlinclude EnableControl-response.msg.html

(cl:defclass <EnableControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EnableControl-response (<EnableControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<EnableControl-response> is deprecated: use controller-srv:EnableControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EnableControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:success-val is deprecated.  Use controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableControl-response>) ostream)
  "Serializes a message object of type '<EnableControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableControl-response>) istream)
  "Deserializes a message object of type '<EnableControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableControl-response>)))
  "Returns string type for a service object of type '<EnableControl-response>"
  "controller/EnableControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableControl-response)))
  "Returns string type for a service object of type 'EnableControl-response"
  "controller/EnableControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableControl-response>)))
  "Returns md5sum for a message object of type '<EnableControl-response>"
  "7badc0803f167fb2cf52d5b259ad3ce0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableControl-response)))
  "Returns md5sum for a message object of type 'EnableControl-response"
  "7badc0803f167fb2cf52d5b259ad3ce0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableControl-response>)))
  "Returns full string definition for message of type '<EnableControl-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableControl-response)))
  "Returns full string definition for message of type 'EnableControl-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnableControl)))
  'EnableControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnableControl)))
  'EnableControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableControl)))
  "Returns string type for a service object of type '<EnableControl>"
  "controller/EnableControl")