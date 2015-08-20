
(cl:in-package :asdf)

(defsystem "crosbot_map-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :crosbot_map-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GetSnap" :depends-on ("_package_GetSnap"))
    (:file "_package_GetSnap" :depends-on ("_package"))
    (:file "ListSnaps" :depends-on ("_package_ListSnaps"))
    (:file "_package_ListSnaps" :depends-on ("_package"))
    (:file "ModifySnap" :depends-on ("_package_ModifySnap"))
    (:file "_package_ModifySnap" :depends-on ("_package"))
  ))