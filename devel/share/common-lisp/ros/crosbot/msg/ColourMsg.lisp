; Auto-generated. Do not edit!


(cl:in-package crosbot-msg)


;//! \htmlinclude ColourMsg.msg.html

(cl:defclass <ColourMsg> (roslisp-msg-protocol:ros-message)
  ((r
    :reader r
    :initarg :r
    :type cl:fixnum
    :initform 0)
   (g
    :reader g
    :initarg :g
    :type cl:fixnum
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:fixnum
    :initform 0)
   (a
    :reader a
    :initarg :a
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ColourMsg (<ColourMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColourMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColourMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot-msg:<ColourMsg> is deprecated: use crosbot-msg:ColourMsg instead.")))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <ColourMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:r-val is deprecated.  Use crosbot-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'g-val :lambda-list '(m))
(cl:defmethod g-val ((m <ColourMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:g-val is deprecated.  Use crosbot-msg:g instead.")
  (g m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <ColourMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:b-val is deprecated.  Use crosbot-msg:b instead.")
  (b m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <ColourMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot-msg:a-val is deprecated.  Use crosbot-msg:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColourMsg>) ostream)
  "Serializes a message object of type '<ColourMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'a)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColourMsg>) istream)
  "Deserializes a message object of type '<ColourMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'a)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColourMsg>)))
  "Returns string type for a message object of type '<ColourMsg>"
  "crosbot/ColourMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColourMsg)))
  "Returns string type for a message object of type 'ColourMsg"
  "crosbot/ColourMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColourMsg>)))
  "Returns md5sum for a message object of type '<ColourMsg>"
  "3a89b17adab5bedef0b554f03235d9b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColourMsg)))
  "Returns md5sum for a message object of type 'ColourMsg"
  "3a89b17adab5bedef0b554f03235d9b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColourMsg>)))
  "Returns full string definition for message of type '<ColourMsg>"
  (cl:format cl:nil "uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColourMsg)))
  "Returns full string definition for message of type 'ColourMsg"
  (cl:format cl:nil "uint8 r~%uint8 g~%uint8 b~%uint8 a~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColourMsg>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColourMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ColourMsg
    (cl:cons ':r (r msg))
    (cl:cons ':g (g msg))
    (cl:cons ':b (b msg))
    (cl:cons ':a (a msg))
))
