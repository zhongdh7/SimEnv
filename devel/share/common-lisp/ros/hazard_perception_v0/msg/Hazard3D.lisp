; Auto-generated. Do not edit!


(cl:in-package hazard_perception_v0-msg)


;//! \htmlinclude Hazard3D.msg.html

(cl:defclass <Hazard3D> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (observation_count
    :reader observation_count
    :initarg :observation_count
    :type cl:integer
    :initform 0)
   (confirmed
    :reader confirmed
    :initarg :confirmed
    :type cl:boolean
    :initform cl:nil)
   (localization_source
    :reader localization_source
    :initarg :localization_source
    :type cl:string
    :initform ""))
)

(cl:defclass Hazard3D (<Hazard3D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hazard3D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hazard3D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hazard_perception_v0-msg:<Hazard3D> is deprecated: use hazard_perception_v0-msg:Hazard3D instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:id-val is deprecated.  Use hazard_perception_v0-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:position-val is deprecated.  Use hazard_perception_v0-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:confidence-val is deprecated.  Use hazard_perception_v0-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'observation_count-val :lambda-list '(m))
(cl:defmethod observation_count-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:observation_count-val is deprecated.  Use hazard_perception_v0-msg:observation_count instead.")
  (observation_count m))

(cl:ensure-generic-function 'confirmed-val :lambda-list '(m))
(cl:defmethod confirmed-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:confirmed-val is deprecated.  Use hazard_perception_v0-msg:confirmed instead.")
  (confirmed m))

(cl:ensure-generic-function 'localization_source-val :lambda-list '(m))
(cl:defmethod localization_source-val ((m <Hazard3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception_v0-msg:localization_source-val is deprecated.  Use hazard_perception_v0-msg:localization_source instead.")
  (localization_source m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hazard3D>) ostream)
  "Serializes a message object of type '<Hazard3D>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'observation_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'observation_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'observation_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'observation_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirmed) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'localization_source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'localization_source))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hazard3D>) istream)
  "Deserializes a message object of type '<Hazard3D>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'observation_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'observation_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'observation_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'observation_count)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confirmed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'localization_source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'localization_source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hazard3D>)))
  "Returns string type for a message object of type '<Hazard3D>"
  "hazard_perception_v0/Hazard3D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hazard3D)))
  "Returns string type for a message object of type 'Hazard3D"
  "hazard_perception_v0/Hazard3D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hazard3D>)))
  "Returns md5sum for a message object of type '<Hazard3D>"
  "2af06c5e740031a3e6b4af89771e5573")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hazard3D)))
  "Returns md5sum for a message object of type 'Hazard3D"
  "2af06c5e740031a3e6b4af89771e5573")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hazard3D>)))
  "Returns full string definition for message of type '<Hazard3D>"
  (cl:format cl:nil "uint32 id~%geometry_msgs/Point position~%float32 confidence~%uint32 observation_count~%bool confirmed~%string localization_source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hazard3D)))
  "Returns full string definition for message of type 'Hazard3D"
  (cl:format cl:nil "uint32 id~%geometry_msgs/Point position~%float32 confidence~%uint32 observation_count~%bool confirmed~%string localization_source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hazard3D>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     4
     1
     4 (cl:length (cl:slot-value msg 'localization_source))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hazard3D>))
  "Converts a ROS message object to a list"
  (cl:list 'Hazard3D
    (cl:cons ':id (id msg))
    (cl:cons ':position (position msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':observation_count (observation_count msg))
    (cl:cons ':confirmed (confirmed msg))
    (cl:cons ':localization_source (localization_source msg))
))
