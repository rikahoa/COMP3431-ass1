; Auto-generated. Do not edit!


(cl:in-package crosbot-msg)


;//! \htmlinclude ColouredPointMsg.msg.html

(cl:defclass <ColouredPointMsg> (roslisp-msg-protocol:ros-message)
  ((p
    :reader p
    :initarg :p
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (c
    :reader c
    :initarg :c
    :type crosbot-msg:ColourMsg
    :initform (cl:make-instance 'crosbot-msg:ColourMsg)))
)

(cl:defclass ColouredPointMsg (<ColouredPointMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColouredPointMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColouredPointMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot-msg:<ColouredPointMsg> is deprecated: use crosbot-msg:ColouredPointMsg instead.")))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <ColouredPointMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:p-val is deprecated.  Use crosbot-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <ColouredPointMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:c-val is deprecated.  Use crosbot-msg:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColouredPointMsg>) ostream)
  "Serializes a message object of type '<ColouredPointMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'c) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColouredPointMsg>) istream)
  "Deserializes a message object of type '<ColouredPointMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'c) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColouredPointMsg>)))
  "Returns string type for a message object of type '<ColouredPointMsg>"
  "crosbot/ColouredPointMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColouredPointMsg)))
  "Returns string type for a message object of type 'ColouredPointMsg"
  "crosbot/ColouredPointMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColouredPointMsg>)))
  "Returns md5sum for a message object of type '<ColouredPointMsg>"
  "d75f9aa7b2fc4712f55c6e6703f1c090")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColouredPointMsg)))
  "Returns md5sum for a message object of type 'ColouredPointMsg"
  "d75f9aa7b2fc4712f55c6e6703f1c090")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColouredPointMsg>)))
  "Returns full string definition for message of type '<ColouredPointMsg>"
  (cl:format cl:nil "geometry_msgs/Point p~%ColourMsg c~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColouredPointMsg)))
  "Returns full string definition for message of type 'ColouredPointMsg"
  (cl:format cl:nil "geometry_msgs/Point p~%ColourMsg c~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: crosbot/ColourMsg~%uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColouredPointMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'c))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColouredPointMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ColouredPointMsg
    (cl:cons ':p (p msg))
    (cl:cons ':c (c msg))
))
