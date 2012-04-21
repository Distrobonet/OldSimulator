
(cl:in-package :asdf)

(defsystem "Simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FormationMessage" :depends-on ("_package_FormationMessage"))
    (:file "_package_FormationMessage" :depends-on ("_package"))
    (:file "StateMessage" :depends-on ("_package_StateMessage"))
    (:file "_package_StateMessage" :depends-on ("_package"))
    (:file "RelationshipMessage" :depends-on ("_package_RelationshipMessage"))
    (:file "_package_RelationshipMessage" :depends-on ("_package"))
    (:file "VectorMessage" :depends-on ("_package_VectorMessage"))
    (:file "_package_VectorMessage" :depends-on ("_package"))
  ))