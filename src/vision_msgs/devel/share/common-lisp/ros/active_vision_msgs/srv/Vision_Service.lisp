; Auto-generated. Do not edit!


(cl:in-package active_vision_msgs-srv)


;//! \htmlinclude Vision_Service-request.msg.html

(cl:defclass <Vision_Service-request> (roslisp-msg-protocol:ros-message)
  ((Label
    :reader Label
    :initarg :Label
    :type cl:string
    :initform ""))
)

(cl:defclass Vision_Service-request (<Vision_Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vision_Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vision_Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_vision_msgs-srv:<Vision_Service-request> is deprecated: use active_vision_msgs-srv:Vision_Service-request instead.")))

(cl:ensure-generic-function 'Label-val :lambda-list '(m))
(cl:defmethod Label-val ((m <Vision_Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_vision_msgs-srv:Label-val is deprecated.  Use active_vision_msgs-srv:Label instead.")
  (Label m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vision_Service-request>) ostream)
  "Serializes a message object of type '<Vision_Service-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Label))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vision_Service-request>) istream)
  "Deserializes a message object of type '<Vision_Service-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vision_Service-request>)))
  "Returns string type for a service object of type '<Vision_Service-request>"
  "active_vision_msgs/Vision_ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vision_Service-request)))
  "Returns string type for a service object of type 'Vision_Service-request"
  "active_vision_msgs/Vision_ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vision_Service-request>)))
  "Returns md5sum for a message object of type '<Vision_Service-request>"
  "743c237c8927abfed6464a4d17c2f4fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vision_Service-request)))
  "Returns md5sum for a message object of type 'Vision_Service-request"
  "743c237c8927abfed6464a4d17c2f4fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vision_Service-request>)))
  "Returns full string definition for message of type '<Vision_Service-request>"
  (cl:format cl:nil "string Label~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vision_Service-request)))
  "Returns full string definition for message of type 'Vision_Service-request"
  (cl:format cl:nil "string Label~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vision_Service-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Label))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vision_Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Vision_Service-request
    (cl:cons ':Label (Label msg))
))
;//! \htmlinclude Vision_Service-response.msg.html

(cl:defclass <Vision_Service-response> (roslisp-msg-protocol:ros-message)
  ((object_location
    :reader object_location
    :initarg :object_location
    :type (cl:vector active_vision_msgs-msg:Vision_Message)
   :initform (cl:make-array 0 :element-type 'active_vision_msgs-msg:Vision_Message :initial-element (cl:make-instance 'active_vision_msgs-msg:Vision_Message))))
)

(cl:defclass Vision_Service-response (<Vision_Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vision_Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vision_Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_vision_msgs-srv:<Vision_Service-response> is deprecated: use active_vision_msgs-srv:Vision_Service-response instead.")))

(cl:ensure-generic-function 'object_location-val :lambda-list '(m))
(cl:defmethod object_location-val ((m <Vision_Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_vision_msgs-srv:object_location-val is deprecated.  Use active_vision_msgs-srv:object_location instead.")
  (object_location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vision_Service-response>) ostream)
  "Serializes a message object of type '<Vision_Service-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'object_location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vision_Service-response>) istream)
  "Deserializes a message object of type '<Vision_Service-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_location) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_location)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'active_vision_msgs-msg:Vision_Message))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vision_Service-response>)))
  "Returns string type for a service object of type '<Vision_Service-response>"
  "active_vision_msgs/Vision_ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vision_Service-response)))
  "Returns string type for a service object of type 'Vision_Service-response"
  "active_vision_msgs/Vision_ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vision_Service-response>)))
  "Returns md5sum for a message object of type '<Vision_Service-response>"
  "743c237c8927abfed6464a4d17c2f4fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vision_Service-response)))
  "Returns md5sum for a message object of type 'Vision_Service-response"
  "743c237c8927abfed6464a4d17c2f4fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vision_Service-response>)))
  "Returns full string definition for message of type '<Vision_Service-response>"
  (cl:format cl:nil "Vision_Message[] object_location~%~%~%================================================================================~%MSG: active_vision_msgs/Vision_Message~%string Frameid~%geometry_msgs/Point32 Pos~%bool Found~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vision_Service-response)))
  "Returns full string definition for message of type 'Vision_Service-response"
  (cl:format cl:nil "Vision_Message[] object_location~%~%~%================================================================================~%MSG: active_vision_msgs/Vision_Message~%string Frameid~%geometry_msgs/Point32 Pos~%bool Found~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vision_Service-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_location) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vision_Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Vision_Service-response
    (cl:cons ':object_location (object_location msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Vision_Service)))
  'Vision_Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Vision_Service)))
  'Vision_Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vision_Service)))
  "Returns string type for a service object of type '<Vision_Service>"
  "active_vision_msgs/Vision_Service")