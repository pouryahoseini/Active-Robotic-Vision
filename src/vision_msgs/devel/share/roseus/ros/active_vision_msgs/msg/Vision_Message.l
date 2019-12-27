;; Auto-generated. Do not edit!


(when (boundp 'active_vision_msgs::Vision_Message)
  (if (not (find-package "ACTIVE_VISION_MSGS"))
    (make-package "ACTIVE_VISION_MSGS"))
  (shadow 'Vision_Message (find-package "ACTIVE_VISION_MSGS")))
(unless (find-package "ACTIVE_VISION_MSGS::VISION_MESSAGE")
  (make-package "ACTIVE_VISION_MSGS::VISION_MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude Vision_Message.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass active_vision_msgs::Vision_Message
  :super ros::object
  :slots (_Frameid _Pos _Found ))

(defmethod active_vision_msgs::Vision_Message
  (:init
   (&key
    ((:Frameid __Frameid) "")
    ((:Pos __Pos) (instance geometry_msgs::Point32 :init))
    ((:Found __Found) nil)
    )
   (send-super :init)
   (setq _Frameid (string __Frameid))
   (setq _Pos __Pos)
   (setq _Found __Found)
   self)
  (:Frameid
   (&optional __Frameid)
   (if __Frameid (setq _Frameid __Frameid)) _Frameid)
  (:Pos
   (&rest __Pos)
   (if (keywordp (car __Pos))
       (send* _Pos __Pos)
     (progn
       (if __Pos (setq _Pos (car __Pos)))
       _Pos)))
  (:Found
   (&optional __Found)
   (if __Found (setq _Found __Found)) _Found)
  (:serialization-length
   ()
   (+
    ;; string _Frameid
    4 (length _Frameid)
    ;; geometry_msgs/Point32 _Pos
    (send _Pos :serialization-length)
    ;; bool _Found
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Frameid
       (write-long (length _Frameid) s) (princ _Frameid s)
     ;; geometry_msgs/Point32 _Pos
       (send _Pos :serialize s)
     ;; bool _Found
       (if _Found (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Frameid
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Frameid (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point32 _Pos
     (send _Pos :deserialize buf ptr-) (incf ptr- (send _Pos :serialization-length))
   ;; bool _Found
     (setq _Found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get active_vision_msgs::Vision_Message :md5sum-) "9e12f5951169920222965062c93aead0")
(setf (get active_vision_msgs::Vision_Message :datatype-) "active_vision_msgs/Vision_Message")
(setf (get active_vision_msgs::Vision_Message :definition-)
      "string Frameid
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
float32 z
")



(provide :active_vision_msgs/Vision_Message "9e12f5951169920222965062c93aead0")


