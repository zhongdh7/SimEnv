;; Auto-generated. Do not edit!


(when (boundp 'hazard_perception::Hazard3DArray)
  (if (not (find-package "HAZARD_PERCEPTION"))
    (make-package "HAZARD_PERCEPTION"))
  (shadow 'Hazard3DArray (find-package "HAZARD_PERCEPTION")))
(unless (find-package "HAZARD_PERCEPTION::HAZARD3DARRAY")
  (make-package "HAZARD_PERCEPTION::HAZARD3DARRAY"))

(in-package "ROS")
;;//! \htmlinclude Hazard3DArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hazard_perception::Hazard3DArray
  :super ros::object
  :slots (_header _hazards ))

(defmethod hazard_perception::Hazard3DArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:hazards __hazards) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _hazards __hazards)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:hazards
   (&rest __hazards)
   (if (keywordp (car __hazards))
       (send* _hazards __hazards)
     (progn
       (if __hazards (setq _hazards (car __hazards)))
       _hazards)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; hazard_perception/Hazard3D[] _hazards
    (apply #'+ (send-all _hazards :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; hazard_perception/Hazard3D[] _hazards
     (write-long (length _hazards) s)
     (dolist (elem _hazards)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; hazard_perception/Hazard3D[] _hazards
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _hazards (let (r) (dotimes (i n) (push (instance hazard_perception::Hazard3D :init) r)) r))
     (dolist (elem- _hazards)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get hazard_perception::Hazard3DArray :md5sum-) "45285fb7a9658ae251ee2d8e7e727d24")
(setf (get hazard_perception::Hazard3DArray :datatype-) "hazard_perception/Hazard3DArray")
(setf (get hazard_perception::Hazard3DArray :definition-)
      "std_msgs/Header header
Hazard3D[] hazards

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
MSG: hazard_perception/Hazard3D
uint32 id
geometry_msgs/Point position
float32 confidence
uint32 observation_count
bool confirmed
string localization_source

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hazard_perception/Hazard3DArray "45285fb7a9658ae251ee2d8e7e727d24")


