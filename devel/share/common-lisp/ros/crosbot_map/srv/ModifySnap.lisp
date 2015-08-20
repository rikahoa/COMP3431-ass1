; Auto-generated. Do not edit!


(cl:in-package crosbot_map-srv)


;//! \htmlinclude ModifySnap-request.msg.html

(cl:defclass <ModifySnap-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (type
    :reader type
    :initarg :type
    :type std_msgs-msg:UInt8
    :initform (cl:make-instance 'std_msgs-msg:UInt8))
   (status
    :reader status
    :initarg :status
    :type std_msgs-msg:UInt8
    :initform (cl:make-instance 'std_msgs-msg:UInt8))
   (description
    :reader description
    :initarg :description
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass ModifySnap-request (<ModifySnap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModifySnap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModifySnap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<ModifySnap-request> is deprecated: use crosbot_map-srv:ModifySnap-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ModifySnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:id-val is deprecated.  Use crosbot_map-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ModifySnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:type-val is deprecated.  Use crosbot_map-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ModifySnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:status-val is deprecated.  Use crosbot_map-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <ModifySnap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crosbot_map-srv:description-val is deprecated.  Use crosbot_map-srv:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModifySnap-request>) ostream)
  "Serializes a message object of type '<ModifySnap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'type) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'description) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModifySnap-request>) istream)
  "Deserializes a message object of type '<ModifySnap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'type) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'description) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModifySnap-request>)))
  "Returns string type for a service object of type '<ModifySnap-request>"
  "crosbot_map/ModifySnapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModifySnap-request)))
  "Returns string type for a service object of type 'ModifySnap-request"
  "crosbot_map/ModifySnapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModifySnap-request>)))
  "Returns md5sum for a message object of type '<ModifySnap-request>"
  "c1c64c0c3bf7d1de1b6bf7e6bbdda46c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModifySnap-request)))
  "Returns md5sum for a message object of type 'ModifySnap-request"
  "c1c64c0c3bf7d1de1b6bf7e6bbdda46c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModifySnap-request>)))
  "Returns full string definition for message of type '<ModifySnap-request>"
  (cl:format cl:nil "std_msgs/Int32  id~%std_msgs/UInt8  type~%std_msgs/UInt8  status~%std_msgs/String description~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModifySnap-request)))
  "Returns full string definition for message of type 'ModifySnap-request"
  (cl:format cl:nil "std_msgs/Int32  id~%std_msgs/UInt8  type~%std_msgs/UInt8  status~%std_msgs/String description~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/UInt8~%uint8 data~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModifySnap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModifySnap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModifySnap-request
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':status (status msg))
    (cl:cons ':description (description msg))
))
;//! \htmlinclude ModifySnap-response.msg.html

(cl:defclass <ModifySnap-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ModifySnap-response (<ModifySnap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModifySnap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModifySnap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crosbot_map-srv:<ModifySnap-response> is deprecated: use crosbot_map-srv:ModifySnap-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModifySnap-response>) ostream)
  "Serializes a message object of type '<ModifySnap-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModifySnap-response>) istream)
  "Deserializes a message object of type '<ModifySnap-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModifySnap-response>)))
  "Returns string type for a service object of type '<ModifySnap-response>"
  "crosbot_map/ModifySnapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModifySnap-response)))
  "Returns string type for a service object of type 'ModifySnap-response"
  "crosbot_map/ModifySnapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModifySnap-response>)))
  "Returns md5sum for a message object of type '<ModifySnap-response>"
  "c1c64c0c3bf7d1de1b6bf7e6bbdda46c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModifySnap-response)))
  "Returns md5sum for a message object of type 'ModifySnap-response"
  "c1c64c0c3bf7d1de1b6bf7e6bbdda46c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModifySnap-response>)))
  "Returns full string definition for message of type '<ModifySnap-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModifySnap-response)))
  "Returns full string definition for message of type 'ModifySnap-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModifySnap-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModifySnap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModifySnap-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModifySnap)))
  'ModifySnap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModifySnap)))
  'ModifySnap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModifySnap)))
  "Returns string type for a service object of type '<ModifySnap>"
  "crosbot_map/ModifySnap")