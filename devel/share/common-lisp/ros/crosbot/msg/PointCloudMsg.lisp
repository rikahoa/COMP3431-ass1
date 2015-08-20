; Auto-generated. Do not edit!


(cl:in-package crosbot-msg)


;//! \htmlinclude PointCloudMsg.msg.html

(cl:defclass <PointCloudMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (colours
    :reader colours
    :initarg :colours
    :type (cl:vector crosbot-msg:ColourMsg)
   :initform (cl:make-array 0 :element-type 'crosbot-msg:ColourMsg :initial-element (cl:make-instance 'crosbot-msg:ColourMsg))))
)

(cl:defclass PointCloudMsg (<PointCloudMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointCloudMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointCloudMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot-msg:<PointCloudMsg> is deprecated: use crosbot-msg:PointCloudMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PointCloudMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:header-val is deprecated.  Use crosbot-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <PointCloudMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:points-val is deprecated.  Use crosbot-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'colours-val :lambda-list '(m))
(cl:defmethod colours-val ((m <PointCloudMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:colours-val is deprecated.  Use crosbot-msg:colours instead.")
  (colours m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointCloudMsg>) ostream)
  "Serializes a message object of type '<PointCloudMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'colours))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'colours))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointCloudMsg>) istream)
  "Deserializes a message object of type '<PointCloudMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'colours) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'colours)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crosbot-msg:ColourMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointCloudMsg>)))
  "Returns string type for a message object of type '<PointCloudMsg>"
  "crosbot/PointCloudMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointCloudMsg)))
  "Returns string type for a message object of type 'PointCloudMsg"
  "crosbot/PointCloudMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointCloudMsg>)))
  "Returns md5sum for a message object of type '<PointCloudMsg>"
  "d2b0c48c6f0b4cbe0cb88cfafd58a3ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointCloudMsg)))
  "Returns md5sum for a message object of type 'PointCloudMsg"
  "d2b0c48c6f0b4cbe0cb88cfafd58a3ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointCloudMsg>)))
  "Returns full string definition for message of type '<PointCloudMsg>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointCloudMsg)))
  "Returns full string definition for message of type 'PointCloudMsg"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointCloudMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'colours) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointCloudMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PointCloudMsg
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
    (cl:cons ':colours (colours msg))
))
