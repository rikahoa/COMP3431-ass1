; Auto-generated. Do not edit!


(cl:in-package crosbot_map-srv)


;//! \htmlinclude GetSnap-request.msg.html

(cl:defclass <GetSnap-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (type
    :reader type
    :initarg :type
    :type std_msgs-msg:UInt8
    :initform (cl:make-instance 'std_msgs-msg:UInt8)))
)

(cl:defclass GetSnap-request (<GetSnap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSnap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSnap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<GetSnap-request> is deprecated: use crosbot_map-srv:GetSnap-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetSnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:id-val is deprecated.  Use crosbot_map-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <GetSnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:type-val is deprecated.  Use crosbot_map-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSnap-request>) ostream)
  "Serializes a message object of type '<GetSnap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'type) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSnap-request>) istream)
  "Deserializes a message object of type '<GetSnap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'type) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSnap-request>)))
  "Returns string type for a service object of type '<GetSnap-request>"
  "crosbot_map/GetSnapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSnap-request)))
  "Returns string type for a service object of type 'GetSnap-request"
  "crosbot_map/GetSnapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSnap-request>)))
  "Returns md5sum for a message object of type '<GetSnap-request>"
  "ed02702e11dc9a36e98cb929d7f780ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSnap-request)))
  "Returns md5sum for a message object of type 'GetSnap-request"
  "ed02702e11dc9a36e98cb929d7f780ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSnap-request>)))
  "Returns full string definition for message of type '<GetSnap-request>"
  (cl:format cl:nil "std_msgs/Int32 id~%std_msgs/UInt8 type~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSnap-request)))
  "Returns full string definition for message of type 'GetSnap-request"
  (cl:format cl:nil "std_msgs/Int32 id~%std_msgs/UInt8 type~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSnap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSnap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSnap-request
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
))
;//! \htmlinclude GetSnap-response.msg.html

(cl:defclass <GetSnap-response> (roslisp-msg-protocol:ros-message)
  ((snap
    :reader snap
    :initarg :snap
    :type crosbot_map-msg:SnapMsg
    :initform (cl:make-instance 'crosbot_map-msg:SnapMsg)))
)

(cl:defclass GetSnap-response (<GetSnap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSnap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSnap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<GetSnap-response> is deprecated: use crosbot_map-srv:GetSnap-response instead.")))

(cl:ensure-generic-function 'snap-val :lambda-list '(m))
(cl:defmethod snap-val ((m <GetSnap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:snap-val is deprecated.  Use crosbot_map-srv:snap instead.")
  (snap m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSnap-response>) ostream)
  "Serializes a message object of type '<GetSnap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'snap) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSnap-response>) istream)
  "Deserializes a message object of type '<GetSnap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'snap) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSnap-response>)))
  "Returns string type for a service object of type '<GetSnap-response>"
  "crosbot_map/GetSnapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSnap-response)))
  "Returns string type for a service object of type 'GetSnap-response"
  "crosbot_map/GetSnapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSnap-response>)))
  "Returns md5sum for a message object of type '<GetSnap-response>"
  "ed02702e11dc9a36e98cb929d7f780ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSnap-response)))
  "Returns md5sum for a message object of type 'GetSnap-response"
  "ed02702e11dc9a36e98cb929d7f780ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSnap-response>)))
  "Returns full string definition for message of type '<GetSnap-response>"
  (cl:format cl:nil "crosbot_map/SnapMsg snap~%~%~%================================================================================~%MSG: crosbot_map/SnapMsg~%Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSnap-response)))
  "Returns full string definition for message of type 'GetSnap-response"
  (cl:format cl:nil "crosbot_map/SnapMsg snap~%~%~%================================================================================~%MSG: crosbot_map/SnapMsg~%Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSnap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'snap))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSnap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSnap-response
    (cl:cons ':snap (snap msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSnap)))
  'GetSnap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSnap)))
  'GetSnap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSnap)))
  "Returns string type for a service object of type '<GetSnap>"
  "crosbot_map/GetSnap")