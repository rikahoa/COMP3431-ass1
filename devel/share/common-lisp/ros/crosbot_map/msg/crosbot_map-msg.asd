
(cl:in-package :asdf)

(defsystem "crosbot_map-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :crosbot-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SnapMsg" :depends-on ("_package_SnapMsg"))
    (:file "_package_SnapMsg" :depends-on ("_package"))
  ))