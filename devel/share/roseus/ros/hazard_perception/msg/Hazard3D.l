;; Auto-generated. Do not edit!


(when (boundp 'hazard_perception::Hazard3D)
  (if (not (find-package "HAZARD_PERCEPTION"))
    (make-package "HAZARD_PERCEPTION"))
  (shadow 'Hazard3D (find-package "HAZARD_PERCEPTION")))
(unless (find-package "HAZARD_PERCEPTION::HAZARD3D")
  (make-package "HAZARD_PERCEPTION::HAZARD3D"))

(in-package "ROS")
;;//! \htmlinclude Hazard3D.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hazard_perception::Hazard3D
  :super ros::object
  :slots (_id _position _confidence _observation_count _confirmed _localization_source ))

(defmethod hazard_perception::Hazard3D
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:confidence __confidence) 0.0)
    ((:observation_count __observation_count) 0)
    ((:confirmed __confirmed) nil)
    ((:localization_source __localization_source) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position __position)
   (setq _confidence (float __confidence))
   (setq _observation_count (round __observation_count))
   (setq _confirmed __confirmed)
   (setq _localization_source (string __localization_source))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:observation_count
   (&optional __observation_count)
   (if __observation_count (setq _observation_count __observation_count)) _observation_count)
  (:confirmed
   (&optional (__confirmed :null))
   (if (not (eq __confirmed :null)) (setq _confirmed __confirmed)) _confirmed)
  (:localization_source
   (&optional __localization_source)
   (if __localization_source (setq _localization_source __localization_source)) _localization_source)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _confidence
    4
    ;; uint32 _observation_count
    4
    ;; bool _confirmed
    1
    ;; string _localization_source
    4 (length _localization_source)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _observation_count
       (write-long _observation_count s)
     ;; bool _confirmed
       (if _confirmed (write-byte -1 s) (write-byte 0 s))
     ;; string _localization_source
       (write-long (length _localization_source) s) (princ _localization_source s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _observation_count
     (setq _observation_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _confirmed
     (setq _confirmed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _localization_source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _localization_source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get hazard_perception::Hazard3D :md5sum-) "2af06c5e740031a3e6b4af89771e5573")
(setf (get hazard_perception::Hazard3D :datatype-) "hazard_perception/Hazard3D")
(setf (get hazard_perception::Hazard3D :definition-)
      "uint32 id
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



(provide :hazard_perception/Hazard3D "2af06c5e740031a3e6b4af89771e5573")


