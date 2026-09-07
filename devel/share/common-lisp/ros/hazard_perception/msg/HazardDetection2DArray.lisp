; Auto-generated. Do not edit!


(cl:in-package hazard_perception-msg)


;//! \htmlinclude HazardDetection2DArray.msg.html

(cl:defclass <HazardDetection2DArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0)
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector hazard_perception-msg:HazardDetection2D)
   :initform (cl:make-array 0 :element-type 'hazard_perception-msg:HazardDetection2D :initial-element (cl:make-instance 'hazard_perception-msg:HazardDetection2D))))
)

(cl:defclass HazardDetection2DArray (<HazardDetection2DArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HazardDetection2DArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HazardDetection2DArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hazard_perception-msg:<HazardDetection2DArray> is deprecated: use hazard_perception-msg:HazardDetection2DArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HazardDetection2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:header-val is deprecated.  Use hazard_perception-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <HazardDetection2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:image_width-val is deprecated.  Use hazard_perception-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <HazardDetection2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:image_height-val is deprecated.  Use hazard_perception-msg:image_height instead.")
  (image_height m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <HazardDetection2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hazard_perception-msg:detections-val is deprecated.  Use hazard_perception-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HazardDetection2DArray>) ostream)
  "Serializes a message object of type '<HazardDetection2DArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HazardDetection2DArray>) istream)
  "Deserializes a message object of type '<HazardDetection2DArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hazard_perception-msg:HazardDetection2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HazardDetection2DArray>)))
  "Returns string type for a message object of type '<HazardDetection2DArray>"
  "hazard_perception/HazardDetection2DArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HazardDetection2DArray)))
  "Returns string type for a message object of type 'HazardDetection2DArray"
  "hazard_perception/HazardDetection2DArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HazardDetection2DArray>)))
  "Returns md5sum for a message object of type '<HazardDetection2DArray>"
  "a54e8f144b5914c6a26823e2f0656b2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HazardDetection2DArray)))
  "Returns md5sum for a message object of type 'HazardDetection2DArray"
  "a54e8f144b5914c6a26823e2f0656b2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HazardDetection2DArray>)))
  "Returns full string definition for message of type '<HazardDetection2DArray>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 image_width~%uint32 image_height~%HazardDetection2D[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hazard_perception/HazardDetection2D~%uint32 x~%uint32 y~%uint32 width~%uint32 height~%float32 center_x~%float32 center_y~%float32 area~%float32 circularity~%float32 aspect_ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HazardDetection2DArray)))
  "Returns full string definition for message of type 'HazardDetection2DArray"
  (cl:format cl:nil "std_msgs/Header header~%uint32 image_width~%uint32 image_height~%HazardDetection2D[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hazard_perception/HazardDetection2D~%uint32 x~%uint32 y~%uint32 width~%uint32 height~%float32 center_x~%float32 center_y~%float32 area~%float32 circularity~%float32 aspect_ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HazardDetection2DArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HazardDetection2DArray>))
  "Converts a ROS message object to a list"
  (cl:list 'HazardDetection2DArray
    (cl:cons ':header (header msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
    (cl:cons ':detections (detections msg))
))
