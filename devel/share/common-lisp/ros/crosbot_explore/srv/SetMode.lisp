; Auto-generated. Do not edit!


(cl:in-package crosbot_explore-srv)


;//! \htmlinclude SetMode-request.msg.html

(cl:defclass <SetMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass SetMode-request (<SetMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<SetMode-request> is deprecated: use crosbot_explore-srv:SetMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_explore-srv:mode-val is deprecated.  Use crosbot_explore-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMode-request>) ostream)
  "Serializes a message object of type '<SetMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMode-request>) istream)
  "Deserializes a message object of type '<SetMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMode-request>)))
  "Returns string type for a service object of type '<SetMode-request>"
  "crosbot_explore/SetModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode-request)))
  "Returns string type for a service object of type 'SetMode-request"
  "crosbot_explore/SetModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMode-request>)))
  "Returns md5sum for a message object of type '<SetMode-request>"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMode-request)))
  "Returns md5sum for a message object of type 'SetMode-request"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMode-request>)))
  "Returns full string definition for message of type '<SetMode-request>"
  (cl:format cl:nil "int32 mode~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMode-request)))
  "Returns full string definition for message of type 'SetMode-request"
  (cl:format cl:nil "int32 mode~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMode-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetMode-response.msg.html

(cl:defclass <SetMode-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetMode-response (<SetMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_explore-srv:<SetMode-response> is deprecated: use crosbot_explore-srv:SetMode-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMode-response>) ostream)
  "Serializes a message object of type '<SetMode-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMode-response>) istream)
  "Deserializes a message object of type '<SetMode-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMode-response>)))
  "Returns string type for a service object of type '<SetMode-response>"
  "crosbot_explore/SetModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode-response)))
  "Returns string type for a service object of type 'SetMode-response"
  "crosbot_explore/SetModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMode-response>)))
  "Returns md5sum for a message object of type '<SetMode-response>"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMode-response)))
  "Returns md5sum for a message object of type 'SetMode-response"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMode-response>)))
  "Returns full string definition for message of type '<SetMode-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMode-response)))
  "Returns full string definition for message of type 'SetMode-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMode-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMode-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMode)))
  'SetMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMode)))
  'SetMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode)))
  "Returns string type for a service object of type '<SetMode>"
  "crosbot_explore/SetMode")