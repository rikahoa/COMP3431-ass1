; Auto-generated. Do not edit!


(cl:in-package crosbot_explore-srv)


;//! \htmlinclude GetPath-request.msg.html

(cl:defclass <GetPath-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GetPath-request (<GetPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<GetPath-request> is deprecated: use crosbot_explore-srv:GetPath-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <GetPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_explore-srv:start-val is deprecated.  Use crosbot_explore-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <GetPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_explore-srv:end-val is deprecated.  Use crosbot_explore-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-request>) ostream)
  "Serializes a message object of type '<GetPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-request>) istream)
  "Deserializes a message object of type '<GetPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-request>)))
  "Returns string type for a service object of type '<GetPath-request>"
  "crosbot_explore/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-request)))
  "Returns string type for a service object of type 'GetPath-request"
  "crosbot_explore/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-request>)))
  "Returns md5sum for a message object of type '<GetPath-request>"
  "41ca099eb8abb09e958f7da45a721668")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-request)))
  "Returns md5sum for a message object of type 'GetPath-request"
  "41ca099eb8abb09e958f7da45a721668")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-request>)))
  "Returns full string definition for message of type '<GetPath-request>"
  (cl:format cl:nil "geometry_msgs/Pose start~%geometry_msgs/Pose end~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-request)))
  "Returns full string definition for message of type 'GetPath-request"
  (cl:format cl:nil "geometry_msgs/Pose start~%geometry_msgs/Pose end~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-request
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
;//! \htmlinclude GetPath-response.msg.html

(cl:defclass <GetPath-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass GetPath-response (<GetPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<GetPath-response> is deprecated: use crosbot_explore-srv:GetPath-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <GetPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_explore-srv:path-val is deprecated.  Use crosbot_explore-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-response>) ostream)
  "Serializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-response>) istream)
  "Deserializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-response>)))
  "Returns string type for a service object of type '<GetPath-response>"
  "crosbot_explore/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-response)))
  "Returns string type for a service object of type 'GetPath-response"
  "crosbot_explore/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-response>)))
  "Returns md5sum for a message object of type '<GetPath-response>"
  "41ca099eb8abb09e958f7da45a721668")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-response)))
  "Returns md5sum for a message object of type 'GetPath-response"
  "41ca099eb8abb09e958f7da45a721668")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-response>)))
  "Returns full string definition for message of type '<GetPath-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-response)))
  "Returns full string definition for message of type 'GetPath-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPath)))
  'GetPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPath)))
  'GetPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath)))
  "Returns string type for a service object of type '<GetPath>"
  "crosbot_explore/GetPath")