; Auto-generated. Do not edit!


(cl:in-package crosbot_map-srv)


;//! \htmlinclude ListSnaps-request.msg.html

(cl:defclass <ListSnaps-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListSnaps-request (<ListSnaps-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListSnaps-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListSnaps-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<ListSnaps-request> is deprecated: use crosbot_map-srv:ListSnaps-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListSnaps-request>) ostream)
  "Serializes a message object of type '<ListSnaps-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListSnaps-request>) istream)
  "Deserializes a message object of type '<ListSnaps-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListSnaps-request>)))
  "Returns string type for a service object of type '<ListSnaps-request>"
  "crosbot_map/ListSnapsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListSnaps-request)))
  "Returns string type for a service object of type 'ListSnaps-request"
  "crosbot_map/ListSnapsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListSnaps-request>)))
  "Returns md5sum for a message object of type '<ListSnaps-request>"
  "a9950552cda46ef9fb7f384a56cf8cd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListSnaps-request)))
  "Returns md5sum for a message object of type 'ListSnaps-request"
  "a9950552cda46ef9fb7f384a56cf8cd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListSnaps-request>)))
  "Returns full string definition for message of type '<ListSnaps-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListSnaps-request)))
  "Returns full string definition for message of type 'ListSnaps-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListSnaps-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListSnaps-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListSnaps-request
))
;//! \htmlinclude ListSnaps-response.msg.html

(cl:defclass <ListSnaps-response> (roslisp-msg-protocol:ros-message)
  ((snaps
    :reader snaps
    :initarg :snaps
    :type (cl:vector crosbot_map-msg:SnapMsg)
   :initform (cl:make-array 0 :element-type 'crosbot_map-msg:SnapMsg :initial-element (cl:make-instance 'crosbot_map-msg:SnapMsg))))
)

(cl:defclass ListSnaps-response (<ListSnaps-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListSnaps-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListSnaps-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<ListSnaps-response> is deprecated: use crosbot_map-srv:ListSnaps-response instead.")))

(cl:ensure-generic-function 'snaps-val :lambda-list '(m))
(cl:defmethod snaps-val ((m <ListSnaps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:snaps-val is deprecated.  Use crosbot_map-srv:snaps instead.")
  (snaps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListSnaps-response>) ostream)
  "Serializes a message object of type '<ListSnaps-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'snaps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'snaps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListSnaps-response>) istream)
  "Deserializes a message object of type '<ListSnaps-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'snaps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'snaps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crosbot_map-msg:SnapMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListSnaps-response>)))
  "Returns string type for a service object of type '<ListSnaps-response>"
  "crosbot_map/ListSnapsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListSnaps-response)))
  "Returns string type for a service object of type 'ListSnaps-response"
  "crosbot_map/ListSnapsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListSnaps-response>)))
  "Returns md5sum for a message object of type '<ListSnaps-response>"
  "a9950552cda46ef9fb7f384a56cf8cd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListSnaps-response)))
  "Returns md5sum for a message object of type 'ListSnaps-response"
  "a9950552cda46ef9fb7f384a56cf8cd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListSnaps-response>)))
  "Returns full string definition for message of type '<ListSnaps-response>"
  (cl:format cl:nil "crosbot_map/SnapMsg[] snaps~%~%================================================================================~%MSG: crosbot_map/SnapMsg~%Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListSnaps-response)))
  "Returns full string definition for message of type 'ListSnaps-response"
  (cl:format cl:nil "crosbot_map/SnapMsg[] snaps~%~%================================================================================~%MSG: crosbot_map/SnapMsg~%Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListSnaps-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'snaps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListSnaps-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListSnaps-response
    (cl:cons ':snaps (snaps msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListSnaps)))
  'ListSnaps-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListSnaps)))
  'ListSnaps-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListSnaps)))
  "Returns string type for a service object of type '<ListSnaps>"
  "crosbot_map/ListSnaps")