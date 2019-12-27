
(cl:in-package :asdf)

(defsystem "active_perception-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :active_perception-msg
)
  :components ((:file "_package")
    (:file "Vision_Service" :depends-on ("_package_Vision_Service"))
    (:file "_package_Vision_Service" :depends-on ("_package"))
  ))