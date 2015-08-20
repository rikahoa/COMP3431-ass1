
(cl:in-package :asdf)

(defsystem "crosbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PointCloudMsg" :depends-on ("_package_PointCloudMsg"))
    (:file "_package_PointCloudMsg" :depends-on ("_package"))
    (:file "ColouredCloudMsg" :depends-on ("_package_ColouredCloudMsg"))
    (:file "_package_ColouredCloudMsg" :depends-on ("_package"))
    (:file "ColouredPointMsg" :depends-on ("_package_ColouredPointMsg"))
    (:file "_package_ColouredPointMsg" :depends-on ("_package"))
    (:file "ColourMsg" :depends-on ("_package_ColourMsg"))
    (:file "_package_ColourMsg" :depends-on ("_package"))
  ))