; Auto-generated. Do not edit!


(cl:in-package crosbot_ogmbicp-srv)


;//! \htmlinclude GetRecentScans-request.msg.html

(cl:defclass <GetRecentScans-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRecentScans-request (<GetRecentScans-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRecentScans-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRecentScans-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_ogmbicp-srv:<GetRecentScans-request> is deprecated: use crosbot_ogmbicp-srv:GetRecentScans-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRecentScans-request>) ostream)
  "Serializes a message object of type '<GetRecentScans-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRecentScans-request>) istream)
  "Deserializes a message object of type '<GetRecentScans-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRecentScans-request>)))
  "Returns string type for a service object of type '<GetRecentScans-request>"
  "crosbot_ogmbicp/GetRecentScansRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRecentScans-request)))
  "Returns string type for a service object of type 'GetRecentScans-request"
  "crosbot_ogmbicp/GetRecentScansRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRecentScans-request>)))
  "Returns md5sum for a message object of type '<GetRecentScans-request>"
  "f920aef0166f130f16d65011603c0109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRecentScans-request)))
  "Returns md5sum for a message object of type 'GetRecentScans-request"
  "f920aef0166f130f16d65011603c0109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRecentScans-request>)))
  "Returns full string definition for message of type '<GetRecentScans-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRecentScans-request)))
  "Returns full string definition for message of type 'GetRecentScans-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRecentScans-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRecentScans-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRecentScans-request
))
;//! \htmlinclude GetRecentScans-response.msg.html

(cl:defclass <GetRecentScans-response> (roslisp-msg-protocol:ros-message)
  ((scans
    :reader scans
    :initarg :scans
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass GetRecentScans-response (<GetRecentScans-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRecentScans-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRecentScans-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_ogmbicp-srv:<GetRecentScans-response> is deprecated: use crosbot_ogmbicp-srv:GetRecentScans-response instead.")))

(cl:ensure-generic-function 'scans-val :lambda-list '(m))
(cl:defmethod scans-val ((m <GetRecentScans-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_ogmbicp-srv:scans-val is deprecated.  Use crosbot_ogmbicp-srv:scans instead.")
  (scans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRecentScans-response>) ostream)
  "Serializes a message object of type '<GetRecentScans-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'scans) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRecentScans-response>) istream)
  "Deserializes a message object of type '<GetRecentScans-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'scans) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRecentScans-response>)))
  "Returns string type for a service object of type '<GetRecentScans-response>"
  "crosbot_ogmbicp/GetRecentScansResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRecentScans-response)))
  "Returns string type for a service object of type 'GetRecentScans-response"
  "crosbot_ogmbicp/GetRecentScansResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRecentScans-response>)))
  "Returns md5sum for a message object of type '<GetRecentScans-response>"
  "f920aef0166f130f16d65011603c0109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRecentScans-response)))
  "Returns md5sum for a message object of type 'GetRecentScans-response"
  "f920aef0166f130f16d65011603c0109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRecentScans-response>)))
  "Returns full string definition for message of type '<GetRecentScans-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 scans~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRecentScans-response)))
  "Returns full string definition for message of type 'GetRecentScans-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 scans~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRecentScans-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'scans))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRecentScans-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRecentScans-response
    (cl:cons ':scans (scans msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRecentScans)))
  'GetRecentScans-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRecentScans)))
  'GetRecentScans-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRecentScans)))
  "Returns string type for a service object of type '<GetRecentScans>"
  "crosbot_ogmbicp/GetRecentScans")