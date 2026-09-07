; Auto-generated. Do not edit!


(cl:in-package hazard_perception-msg)


;//! \htmlinclude Hazard3DArray.msg.html

(cl:defclass <Hazard3DArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hazards
    :reader hazards
    :initarg :hazards
    :type (cl:vector hazard_perception-msg:Hazard3D)
   :initform (cl:make-array 0 :element-type 'hazard_perception-msg:Hazard3D :initial-element (cl:make-instance 'hazard_perception-msg:Hazard3D))))
)

(cl:defclass Hazard3DArray (<Hazard3DArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hazard3DArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hazard3DArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hazard_perception-msg:<Hazard3DArray> is deprecated: use hazard_perception-msg:Hazard3DArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Hazard3DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:header-val is deprecated.  Use hazard_perception-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hazards-val :lambda-list '(m))
(cl:defmethod hazards-val ((m <Hazard3DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:hazards-val is deprecated.  Use hazard_perception-msg:hazards instead.")
  (hazards m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hazard3DArray>) ostream)
  "Serializes a message object of type '<Hazard3DArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'hazards))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'hazards))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hazard3DArray>) istream)
  "Deserializes a message object of type '<Hazard3DArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hazards) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'hazards)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hazard_perception-msg:Hazard3D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hazard3DArray>)))
  "Returns string type for a message object of type '<Hazard3DArray>"
  "hazard_perception/Hazard3DArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hazard3DArray)))
  "Returns string type for a message object of type 'Hazard3DArray"
  "hazard_perception/Hazard3DArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hazard3DArray>)))
  "Returns md5sum for a message object of type '<Hazard3DArray>"
  "45285fb7a9658ae251ee2d8e7e727d24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hazard3DArray)))
  "Returns md5sum for a message object of type 'Hazard3DArray"
  "45285fb7a9658ae251ee2d8e7e727d24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hazard3DArray>)))
  "Returns full string definition for message of type '<Hazard3DArray>"
  (cl:format cl:nil "std_msgs/Header header~%Hazard3D[] hazards~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hazard_perception/Hazard3D~%uint32 id~%geometry_msgs/Point position~%float32 confidence~%uint32 observation_count~%bool confirmed~%string localization_source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hazard3DArray)))
  "Returns full string definition for message of type 'Hazard3DArray"
  (cl:format cl:nil "std_msgs/Header header~%Hazard3D[] hazards~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hazard_perception/Hazard3D~%uint32 id~%geometry_msgs/Point position~%float32 confidence~%uint32 observation_count~%bool confirmed~%string localization_source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hazard3DArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'hazards) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hazard3DArray>))
  "Converts a ROS message object to a list"
  (cl:list 'Hazard3DArray
    (cl:cons ':header (header msg))
    (cl:cons ':hazards (hazards msg))
))
