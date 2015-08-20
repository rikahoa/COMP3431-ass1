; Auto-generated. Do not edit!


(cl:in-package crosbot-msg)


;//! \htmlinclude ColouredCloudMsg.msg.html

(cl:defclass <ColouredCloudMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector crosbot-msg:ColouredPointMsg)
   :initform (cl:make-array 0 :element-type 'crosbot-msg:ColouredPointMsg :initial-element (cl:make-instance 'crosbot-msg:ColouredPointMsg))))
)

(cl:defclass ColouredCloudMsg (<ColouredCloudMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColouredCloudMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColouredCloudMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot-msg:<ColouredCloudMsg> is deprecated: use crosbot-msg:ColouredCloudMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ColouredCloudMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:header-val is deprecated.  Use crosbot-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <ColouredCloudMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:points-val is deprecated.  Use crosbot-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColouredCloudMsg>) ostream)
  "Serializes a message object of type '<ColouredCloudMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColouredCloudMsg>) istream)
  "Deserializes a message object of type '<ColouredCloudMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crosbot-msg:ColouredPointMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColouredCloudMsg>)))
  "Returns string type for a message object of type '<ColouredCloudMsg>"
  "crosbot/ColouredCloudMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColouredCloudMsg)))
  "Returns string type for a message object of type 'ColouredCloudMsg"
  "crosbot/ColouredCloudMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColouredCloudMsg>)))
  "Returns md5sum for a message object of type '<ColouredCloudMsg>"
  "78618f7614cedb66eb9280ccbb77aa3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColouredCloudMsg)))
  "Returns md5sum for a message object of type 'ColouredCloudMsg"
  "78618f7614cedb66eb9280ccbb77aa3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColouredCloudMsg>)))
  "Returns full string definition for message of type '<ColouredCloudMsg>"
  (cl:format cl:nil "Header header~%ColouredPointMsg[] points       # The points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: crosbot/ColouredPointMsg~%geometry_msgs/Point p~%ColourMsg c~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColouredCloudMsg)))
  "Returns full string definition for message of type 'ColouredCloudMsg"
  (cl:format cl:nil "Header header~%ColouredPointMsg[] points       # The points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: crosbot/ColouredPointMsg~%geometry_msgs/Point p~%ColourMsg c~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColouredCloudMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColouredCloudMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ColouredCloudMsg
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
))
