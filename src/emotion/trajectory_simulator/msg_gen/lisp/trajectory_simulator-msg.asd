
(cl:in-package :asdf)

(defsystem "trajectory_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrajectoryObservation" :depends-on ("_package_TrajectoryObservation"))
    (:file "_package_TrajectoryObservation" :depends-on ("_package"))
  ))