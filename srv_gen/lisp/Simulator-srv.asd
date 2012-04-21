
(cl:in-package :asdf)

(defsystem "Simulator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :Simulator-msg
)
  :components ((:file "_package")
    (:file "FormationIndex" :depends-on ("_package_FormationIndex"))
    (:file "_package_FormationIndex" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))