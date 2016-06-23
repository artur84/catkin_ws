
(cl:in-package :asdf)

(defsystem "social_filter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "humanPoses" :depends-on ("_package_humanPoses"))
    (:file "_package_humanPoses" :depends-on ("_package"))
    (:file "int_list" :depends-on ("_package_int_list"))
    (:file "_package_int_list" :depends-on ("_package"))
    (:file "humanPose" :depends-on ("_package_humanPose"))
    (:file "_package_humanPose" :depends-on ("_package"))
    (:file "int_data" :depends-on ("_package_int_data"))
    (:file "_package_int_data" :depends-on ("_package"))
  ))