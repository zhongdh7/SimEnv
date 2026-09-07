;; Auto-generated. Do not edit!


(when (boundp 'hazard_perception_v0::HazardDetection2DArray)
  (if (not (find-package "HAZARD_PERCEPTION_V0"))
    (make-package "HAZARD_PERCEPTION_V0"))
  (shadow 'HazardDetection2DArray (find-package "HAZARD_PERCEPTION_V0")))
(unless (find-package "HAZARD_PERCEPTION_V0::HAZARDDETECTION2DARRAY")
  (make-package "HAZARD_PERCEPTION_V0::HAZARDDETECTION2DARRAY"))

(in-package "ROS")
;;//! \htmlinclude HazardDetection2DArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hazard_perception_v0::HazardDetection2DArray
  :super ros::object
  :slots (_header _image_width _image_height _detections ))

(defmethod hazard_perception_v0::HazardDetection2DArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _image_width (round __image_width))
   (setq _image_height (round __image_height))
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _image_width
    4
    ;; uint32 _image_height
    4
    ;; hazard_perception_v0/HazardDetection2D[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _image_width
       (write-long _image_width s)
     ;; uint32 _image_height
       (write-long _image_height s)
     ;; hazard_perception_v0/HazardDetection2D[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _image_width
     (setq _image_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_height
     (setq _image_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; hazard_perception_v0/HazardDetection2D[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance hazard_perception_v0::HazardDetection2D :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get hazard_perception_v0::HazardDetection2DArray :md5sum-) "a54e8f144b5914c6a26823e2f0656b2a")
(setf (get hazard_perception_v0::HazardDetection2DArray :datatype-) "hazard_perception_v0/HazardDetection2DArray")
(setf (get hazard_perception_v0::HazardDetection2DArray :definition-)
      "std_msgs/Header header
uint32 image_width
uint32 image_height
HazardDetection2D[] detections

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: hazard_perception_v0/HazardDetection2D
uint32 x
uint32 y
uint32 width
uint32 height
float32 center_x
float32 center_y
float32 area
float32 circularity
float32 aspect_ratio

")



(provide :hazard_perception_v0/HazardDetection2DArray "a54e8f144b5914c6a26823e2f0656b2a")


