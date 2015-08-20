
(cl:in-package :asdf)

(defsystem "crosbot_explore-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "FollowPath" :depends-on ("_package_FollowPath"))
    (:file "_package_FollowPath" :depends-on ("_package"))
    (:file "GetPath" :depends-on ("_package_GetPath"))
    (:file "_package_GetPath" :depends-on ("_package"))
    (:file "SetMode" :depends-on ("_package_SetMode"))
    (:file "_package_SetMode" :depends-on ("_package"))
  ))