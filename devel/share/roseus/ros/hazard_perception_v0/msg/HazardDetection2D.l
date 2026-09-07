;; Auto-generated. Do not edit!


(when (boundp 'hazard_perception_v0::HazardDetection2D)
  (if (not (find-package "HAZARD_PERCEPTION_V0"))
    (make-package "HAZARD_PERCEPTION_V0"))
  (shadow 'HazardDetection2D (find-package "HAZARD_PERCEPTION_V0")))
(unless (find-package "HAZARD_PERCEPTION_V0::HAZARDDETECTION2D")
  (make-package "HAZARD_PERCEPTION_V0::HAZARDDETECTION2D"))

(in-package "ROS")
;;//! \htmlinclude HazardDetection2D.msg.html


(defclass hazard_perception_v0::HazardDetection2D
  :super ros::object
  :slots (_x _y _width _height _center_x _center_y _area _circularity _aspect_ratio ))

(defmethod hazard_perception_v0::HazardDetection2D
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:width __width) 0)
    ((:height __height) 0)
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:area __area) 0.0)
    ((:circularity __circularity) 0.0)
    ((:aspect_ratio __aspect_ratio) 0.0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _area (float __area))
   (setq _circularity (float __circularity))
   (setq _aspect_ratio (float __aspect_ratio))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:circularity
   (&optional __circularity)
   (if __circularity (setq _circularity __circularity)) _circularity)
  (:aspect_ratio
   (&optional __aspect_ratio)
   (if __aspect_ratio (setq _aspect_ratio __aspect_ratio)) _aspect_ratio)
  (:serialization-length
   ()
   (+
    ;; uint32 _x
    4
    ;; uint32 _y
    4
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; float32 _center_x
    4
    ;; float32 _center_y
    4
    ;; float32 _area
    4
    ;; float32 _circularity
    4
    ;; float32 _aspect_ratio
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x
       (write-long _x s)
     ;; uint32 _y
       (write-long _y s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; float32 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _circularity
       (sys::poke _circularity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _aspect_ratio
       (sys::poke _aspect_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _center_x
     (setq _center_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_y
     (setq _center_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _circularity
     (setq _circularity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _aspect_ratio
     (setq _aspect_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hazard_perception_v0::HazardDetection2D :md5sum-) "72b181939be57fb98fb1cb0af591360f")
(setf (get hazard_perception_v0::HazardDetection2D :datatype-) "hazard_perception_v0/HazardDetection2D")
(setf (get hazard_perception_v0::HazardDetection2D :definition-)
      "uint32 x
uint32 y
uint32 width
uint32 height
float32 center_x
float32 center_y
float32 area
float32 circularity
float32 aspect_ratio

")



(provide :hazard_perception_v0/HazardDetection2D "72b181939be57fb98fb1cb0af591360f")


