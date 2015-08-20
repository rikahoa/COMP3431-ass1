; Auto-generated. Do not edit!


(cl:in-package crosbot_map-msg)


;//! \htmlinclude SnapMsg.msg.html

(cl:defclass <SnapMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (robot
    :reader robot
    :initarg :robot
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (images
    :reader images
    :initarg :images
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image)))
   (clouds
    :reader clouds
    :initarg :clouds
    :type (cl:vector crosbot-msg:PointCloudMsg)
   :initform (cl:make-array 0 :element-type 'crosbot-msg:PointCloudMsg :initial-element (cl:make-instance 'crosbot-msg:PointCloudMsg))))
)

(cl:defclass SnapMsg (<SnapMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-msg:<SnapMsg> is deprecated: use crosbot_map-msg:SnapMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:header-val is deprecated.  Use crosbot_map-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:type-val is deprecated.  Use crosbot_map-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:status-val is deprecated.  Use crosbot_map-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:id-val is deprecated.  Use crosbot_map-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:description-val is deprecated.  Use crosbot_map-msg:description instead.")
  (description m))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:robot-val is deprecated.  Use crosbot_map-msg:robot instead.")
  (robot m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:pose-val is deprecated.  Use crosbot_map-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'images-val :lambda-list '(m))
(cl:defmethod images-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:images-val is deprecated.  Use crosbot_map-msg:images instead.")
  (images m))

(cl:ensure-generic-function 'clouds-val :lambda-list '(m))
(cl:defmethod clouds-val ((m <SnapMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-msg:clouds-val is deprecated.  Use crosbot_map-msg:clouds instead.")
  (clouds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapMsg>) ostream)
  "Serializes a message object of type '<SnapMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'images))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'images))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'clouds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'clouds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapMsg>) istream)
  "Deserializes a message object of type '<SnapMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'images) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'images)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'clouds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'clouds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crosbot-msg:PointCloudMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapMsg>)))
  "Returns string type for a message object of type '<SnapMsg>"
  "crosbot_map/SnapMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapMsg)))
  "Returns string type for a message object of type 'SnapMsg"
  "crosbot_map/SnapMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapMsg>)))
  "Returns md5sum for a message object of type '<SnapMsg>"
  "e22c37cf6db46fd5970eb0de2b60055b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapMsg)))
  "Returns md5sum for a message object of type 'SnapMsg"
  "e22c37cf6db46fd5970eb0de2b60055b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapMsg>)))
  "Returns full string definition for message of type '<SnapMsg>"
  (cl:format cl:nil "Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapMsg)))
  "Returns full string definition for message of type 'SnapMsg"
  (cl:format cl:nil "Header header~%uint8 type~%int8 status~%uint32 id~%string description~%geometry_msgs/Pose robot    # Global to given frame~%geometry_msgs/Pose pose     # Robot relative~%~%sensor_msgs/Image[] images~%crosbot/PointCloudMsg[] clouds~%#crosbot/ColouredCloudMsg[] colouredClouds~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: crosbot/PointCloudMsg~%Header header~%geometry_msgs/Point[] points	# The points in the cloud.~%ColourMsg[] colours				# The colours of the points. Can be empty.~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4 (cl:length (cl:slot-value msg 'description))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'images) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'clouds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapMsg
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':status (status msg))
    (cl:cons ':id (id msg))
    (cl:cons ':description (description msg))
    (cl:cons ':robot (robot msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':images (images msg))
    (cl:cons ':clouds (clouds msg))
))
