
(cl:in-package :asdf)

(defsystem "crosbot_ogmbicp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetRecentScans" :depends-on ("_package_GetRecentScans"))
    (:file "_package_GetRecentScans" :depends-on ("_package"))
  ))