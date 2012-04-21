
(cl:in-package :asdf)

(defsystem "Simulator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :Simulator-msg
)
  :components ((:file "_package")
    (:file "CurrentFormation" :depends-on ("_package_CurrentFormation"))
    (:file "_package_CurrentFormation" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))