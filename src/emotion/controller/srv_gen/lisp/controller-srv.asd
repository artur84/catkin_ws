
(cl:in-package :asdf)

(defsystem "controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EnableControl" :depends-on ("_package_EnableControl"))
    (:file "_package_EnableControl" :depends-on ("_package"))
  ))