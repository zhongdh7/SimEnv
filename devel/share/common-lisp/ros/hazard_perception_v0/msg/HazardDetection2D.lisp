; Auto-generated. Do not edit!


(cl:in-package hazard_perception_v0-msg)


;//! \htmlinclude HazardDetection2D.msg.html

(cl:defclass <HazardDetection2D> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (center_x
    :reader center_x
    :initarg :center_x
    :type cl:float
    :initform 0.0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:float
    :initform 0.0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0)
   (circularity
    :reader circularity
    :initarg :circularity
    :type cl:float
    :initform 0.0)
   (aspect_ratio
    :reader aspect_ratio
    :initarg :aspect_ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass HazardDetection2D (<HazardDetection2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HazardDetection2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HazardDetection2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hazard_perception_v0-msg:<HazardDetection2D> is deprecated: use hazard_perception_v0-msg:HazardDetection2D instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:x-val is deprecated.  Use hazard_perception_v0-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:y-val is deprecated.  Use hazard_perception_v0-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:width-val is deprecated.  Use hazard_perception_v0-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:height-val is deprecated.  Use hazard_perception_v0-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:center_x-val is deprecated.  Use hazard_perception_v0-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:center_y-val is deprecated.  Use hazard_perception_v0-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:area-val is deprecated.  Use hazard_perception_v0-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'circularity-val :lambda-list '(m))
(cl:defmethod circularity-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:circularity-val is deprecated.  Use hazard_perception_v0-msg:circularity instead.")
  (circularity m))

(cl:ensure-generic-function 'aspect_ratio-val :lambda-list '(m))
(cl:defmethod aspect_ratio-val ((m <HazardDetection2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:aspect_ratio-val is deprecated.  Use hazard_perception_v0-msg:aspect_ratio instead.")
  (aspect_ratio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HazardDetection2D>) ostream)
  "Serializes a message object of type '<HazardDetection2D>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'circularity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'aspect_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HazardDetection2D>) istream)
  "Deserializes a message object of type '<HazardDetection2D>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'circularity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aspect_ratio) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HazardDetection2D>)))
  "Returns string type for a message object of type '<HazardDetection2D>"
  "hazard_perception_v0/HazardDetection2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HazardDetection2D)))
  "Returns string type for a message object of type 'HazardDetection2D"
  "hazard_perception_v0/HazardDetection2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HazardDetection2D>)))
  "Returns md5sum for a message object of type '<HazardDetection2D>"
  "72b181939be57fb98fb1cb0af591360f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HazardDetection2D)))
  "Returns md5sum for a message object of type 'HazardDetection2D"
  "72b181939be57fb98fb1cb0af591360f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HazardDetection2D>)))
  "Returns full string definition for message of type '<HazardDetection2D>"
  (cl:format cl:nil "uint32 x~%uint32 y~%uint32 width~%uint32 height~%float32 center_x~%float32 center_y~%float32 area~%float32 circularity~%float32 aspect_ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HazardDetection2D)))
  "Returns full string definition for message of type 'HazardDetection2D"
  (cl:format cl:nil "uint32 x~%uint32 y~%uint32 width~%uint32 height~%float32 center_x~%float32 center_y~%float32 area~%float32 circularity~%float32 aspect_ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HazardDetection2D>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HazardDetection2D>))
  "Converts a ROS message object to a list"
  (cl:list 'HazardDetection2D
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':area (area msg))
    (cl:cons ':circularity (circularity msg))
    (cl:cons ':aspect_ratio (aspect_ratio msg))
))
