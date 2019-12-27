
(cl:in-package :asdf)

(defsystem "active_perception-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Vision_Message" :depends-on ("_package_Vision_Message"))
    (:file "_package_Vision_Message" :depends-on ("_package"))
  ))