
(cl:in-package :asdf)

(defsystem "pal_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PolarPoint" :depends-on ("_package_PolarPoint"))
    (:file "_package_PolarPoint" :depends-on ("_package"))
    (:file "Point2D" :depends-on ("_package_Point2D"))
    (:file "_package_Point2D" :depends-on ("_package"))
    (:file "PoseWithVelocityArray" :depends-on ("_package_PoseWithVelocityArray"))
    (:file "_package_PoseWithVelocityArray" :depends-on ("_package"))
    (:file "PoseWithVelocity" :depends-on ("_package_PoseWithVelocity"))
    (:file "_package_PoseWithVelocity" :depends-on ("_package"))
  ))