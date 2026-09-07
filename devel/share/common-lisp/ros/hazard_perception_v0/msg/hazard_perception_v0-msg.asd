
(cl:in-package :asdf)

(defsystem "hazard_perception_v0-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Hazard3D" :depends-on ("_package_Hazard3D"))
    (:file "_package_Hazard3D" :depends-on ("_package"))
    (:file "Hazard3DArray" :depends-on ("_package_Hazard3DArray"))
    (:file "_package_Hazard3DArray" :depends-on ("_package"))
    (:file "HazardDetection2D" :depends-on ("_package_HazardDetection2D"))
    (:file "_package_HazardDetection2D" :depends-on ("_package"))
    (:file "HazardDetection2DArray" :depends-on ("_package_HazardDetection2DArray"))
    (:file "_package_HazardDetection2DArray" :depends-on ("_package"))
  ))