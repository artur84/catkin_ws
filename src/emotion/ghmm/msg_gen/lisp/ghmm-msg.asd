
(cl:in-package :asdf)

(defsystem "ghmm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ProbabilisticGrid" :depends-on ("_package_ProbabilisticGrid"))
    (:file "_package_ProbabilisticGrid" :depends-on ("_package"))
  ))