
(cl:in-package :asdf)

(defsystem "Simulator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FormationIndex" :depends-on ("_package_FormationIndex"))
    (:file "_package_FormationIndex" :depends-on ("_package"))
  ))