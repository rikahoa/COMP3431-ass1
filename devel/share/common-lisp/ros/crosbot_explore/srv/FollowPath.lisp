; Auto-generated. Do not edit!


(cl:in-package crosbot_explore-srv)


;//! \htmlinclude FollowPath-request.msg.html

(cl:defclass <FollowPath-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass FollowPath-request (<FollowPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<FollowPath-request> is deprecated: use crosbot_explore-srv:FollowPath-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <FollowPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_explore-srv:path-val is deprecated.  Use crosbot_explore-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowPath-request>) ostream)
  "Serializes a message object of type '<FollowPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowPath-request>) istream)
  "Deserializes a message object of type '<FollowPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowPath-request>)))
  "Returns string type for a service object of type '<FollowPath-request>"
  "crosbot_explore/FollowPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowPath-request)))
  "Returns string type for a service object of type 'FollowPath-request"
  "crosbot_explore/FollowPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowPath-request>)))
  "Returns md5sum for a message object of type '<FollowPath-request>"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowPath-request)))
  "Returns md5sum for a message object of type 'FollowPath-request"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowPath-request>)))
  "Returns full string definition for message of type '<FollowPath-request>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowPath-request)))
  "Returns full string definition for message of type 'FollowPath-request"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowPath-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude FollowPath-response.msg.html

(cl:defclass <FollowPath-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FollowPath-response (<FollowPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<FollowPath-response> is deprecated: use crosbot_explore-srv:FollowPath-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowPath-response>) ostream)
  "Serializes a message object of type '<FollowPath-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowPath-response>) istream)
  "Deserializes a message object of type '<FollowPath-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowPath-response>)))
  "Returns string type for a service object of type '<FollowPath-response>"
  "crosbot_explore/FollowPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowPath-response)))
  "Returns string type for a service object of type 'FollowPath-response"
  "crosbot_explore/FollowPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowPath-response>)))
  "Returns md5sum for a message object of type '<FollowPath-response>"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowPath-response)))
  "Returns md5sum for a message object of type 'FollowPath-response"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowPath-response>)))
  "Returns full string definition for message of type '<FollowPath-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowPath-response)))
  "Returns full string definition for message of type 'FollowPath-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowPath-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowPath-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FollowPath)))
  'FollowPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FollowPath)))
  'FollowPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowPath)))
  "Returns string type for a service object of type '<FollowPath>"
  "crosbot_explore/FollowPath")