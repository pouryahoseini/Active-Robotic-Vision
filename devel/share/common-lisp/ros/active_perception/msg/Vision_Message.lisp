; Auto-generated. Do not edit!


(cl:in-package active_perception-msg)


;//! \htmlinclude Vision_Message.msg.html

(cl:defclass <Vision_Message> (roslisp-msg-protocol:ros-message)
  ((Frameid
    :reader Frameid
    :initarg :Frameid
    :type cl:string
    :initform "")
   (Pos
    :reader Pos
    :initarg :Pos
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (Found
    :reader Found
    :initarg :Found
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Vision_Message (<Vision_Message>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vision_Message>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vision_Message)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_perception-msg:<Vision_Message> is deprecated: use active_perception-msg:Vision_Message instead.")))

(cl:ensure-generic-function 'Frameid-val :lambda-list '(m))
(cl:defmethod Frameid-val ((m <Vision_Message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_perception-msg:Frameid-val is deprecated.  Use active_perception-msg:Frameid instead.")
  (Frameid m))

(cl:ensure-generic-function 'Pos-val :lambda-list '(m))
(cl:defmethod Pos-val ((m <Vision_Message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_perception-msg:Pos-val is deprecated.  Use active_perception-msg:Pos instead.")
  (Pos m))

(cl:ensure-generic-function 'Found-val :lambda-list '(m))
(cl:defmethod Found-val ((m <Vision_Message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_perception-msg:Found-val is deprecated.  Use active_perception-msg:Found instead.")
  (Found m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vision_Message>) ostream)
  "Serializes a message object of type '<Vision_Message>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Frameid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Frameid))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Pos) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Found) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vision_Message>) istream)
  "Deserializes a message object of type '<Vision_Message>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Frameid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Frameid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Pos) istream)
    (cl:setf (cl:slot-value msg 'Found) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vision_Message>)))
  "Returns string type for a message object of type '<Vision_Message>"
  "active_perception/Vision_Message")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vision_Message)))
  "Returns string type for a message object of type 'Vision_Message"
  "active_perception/Vision_Message")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vision_Message>)))
  "Returns md5sum for a message object of type '<Vision_Message>"
  "9e12f5951169920222965062c93aead0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vision_Message)))
  "Returns md5sum for a message object of type 'Vision_Message"
  "9e12f5951169920222965062c93aead0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vision_Message>)))
  "Returns full string definition for message of type '<Vision_Message>"
  (cl:format cl:nil "string Frameid~%geometry_msgs/Point32 Pos~%bool Found~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vision_Message)))
  "Returns full string definition for message of type 'Vision_Message"
  (cl:format cl:nil "string Frameid~%geometry_msgs/Point32 Pos~%bool Found~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vision_Message>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Frameid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Pos))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vision_Message>))
  "Converts a ROS message object to a list"
  (cl:list 'Vision_Message
    (cl:cons ':Frameid (Frameid msg))
    (cl:cons ':Pos (Pos msg))
    (cl:cons ':Found (Found msg))
))
