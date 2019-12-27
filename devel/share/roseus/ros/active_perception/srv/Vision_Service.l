;; Auto-generated. Do not edit!


(when (boundp 'active_perception::Vision_Service)
  (if (not (find-package "ACTIVE_PERCEPTION"))
    (make-package "ACTIVE_PERCEPTION"))
  (shadow 'Vision_Service (find-package "ACTIVE_PERCEPTION")))
(unless (find-package "ACTIVE_PERCEPTION::VISION_SERVICE")
  (make-package "ACTIVE_PERCEPTION::VISION_SERVICE"))
(unless (find-package "ACTIVE_PERCEPTION::VISION_SERVICEREQUEST")
  (make-package "ACTIVE_PERCEPTION::VISION_SERVICEREQUEST"))
(unless (find-package "ACTIVE_PERCEPTION::VISION_SERVICERESPONSE")
  (make-package "ACTIVE_PERCEPTION::VISION_SERVICERESPONSE"))

(in-package "ROS")





(defclass active_perception::Vision_ServiceRequest
  :super ros::object
  :slots (_Label ))

(defmethod active_perception::Vision_ServiceRequest
  (:init
   (&key
    ((:Label __Label) "")
    )
   (send-super :init)
   (setq _Label (string __Label))
   self)
  (:Label
   (&optional __Label)
   (if __Label (setq _Label __Label)) _Label)
  (:serialization-length
   ()
   (+
    ;; string _Label
    4 (length _Label)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Label
       (write-long (length _Label) s) (princ _Label s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass active_perception::Vision_ServiceResponse
  :super ros::object
  :slots (_object_location ))

(defmethod active_perception::Vision_ServiceResponse
  (:init
   (&key
    ((:object_location __object_location) (instance active_perception::Vision_Message :init))
    )
   (send-super :init)
   (setq _object_location __object_location)
   self)
  (:object_location
   (&rest __object_location)
   (if (keywordp (car __object_location))
       (send* _object_location __object_location)
     (progn
       (if __object_location (setq _object_location (car __object_location)))
       _object_location)))
  (:serialization-length
   ()
   (+
    ;; active_perception/Vision_Message _object_location
    (send _object_location :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; active_perception/Vision_Message _object_location
       (send _object_location :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; active_perception/Vision_Message _object_location
     (send _object_location :deserialize buf ptr-) (incf ptr- (send _object_location :serialization-length))
   ;;
   self)
  )

(defclass active_perception::Vision_Service
  :super ros::object
  :slots ())

(setf (get active_perception::Vision_Service :md5sum-) "743c237c8927abfed6464a4d17c2f4fc")
(setf (get active_perception::Vision_Service :datatype-) "active_perception/Vision_Service")
(setf (get active_perception::Vision_Service :request) active_perception::Vision_ServiceRequest)
(setf (get active_perception::Vision_Service :response) active_perception::Vision_ServiceResponse)

(defmethod active_perception::Vision_ServiceRequest
  (:response () (instance active_perception::Vision_ServiceResponse :init)))

(setf (get active_perception::Vision_ServiceRequest :md5sum-) "743c237c8927abfed6464a4d17c2f4fc")
(setf (get active_perception::Vision_ServiceRequest :datatype-) "active_perception/Vision_ServiceRequest")
(setf (get active_perception::Vision_ServiceRequest :definition-)
      "string Label
---
Vision_Message object_location


================================================================================
MSG: active_perception/Vision_Message
string Frameid
geometry_msgs/Point32 Pos
bool Found

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")

(setf (get active_perception::Vision_ServiceResponse :md5sum-) "743c237c8927abfed6464a4d17c2f4fc")
(setf (get active_perception::Vision_ServiceResponse :datatype-) "active_perception/Vision_ServiceResponse")
(setf (get active_perception::Vision_ServiceResponse :definition-)
      "string Label
---
Vision_Message object_location


================================================================================
MSG: active_perception/Vision_Message
string Frameid
geometry_msgs/Point32 Pos
bool Found

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")



(provide :active_perception/Vision_Service "743c237c8927abfed6464a4d17c2f4fc")


