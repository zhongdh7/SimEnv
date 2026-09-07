#!/usr/bin/env python3
"""ROS node for HSV-based red-sphere detection."""

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from hazard_perception.detector import DetectorConfig, RedSphereDetector
from hazard_perception.msg import HazardDetection2D, HazardDetection2DArray


class HazardDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = RedSphereDetector(self._load_config())

        image_topic = rospy.get_param("~image_topic", "/real_sense/rgb/image_raw")
        detections_topic = rospy.get_param(
            "~detections_topic", "/hazard_perception/detections"
        )
        mask_topic = rospy.get_param("~mask_topic", "/hazard_perception/red_mask")
        debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/hazard_perception/debug_image"
        )

        self.detections_publisher = rospy.Publisher(
            detections_topic, HazardDetection2DArray, queue_size=1
        )
        self.mask_publisher = rospy.Publisher(mask_topic, Image, queue_size=1)
        self.debug_image_publisher = rospy.Publisher(
            debug_image_topic, Image, queue_size=1
        )
        self.image_subscriber = rospy.Subscriber(
            image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.loginfo(
            "hazard_detector subscribed to %s; publishing %s, %s and %s",
            image_topic,
            detections_topic,
            mask_topic,
            debug_image_topic,
        )

    @staticmethod
    def _load_config() -> DetectorConfig:
        defaults = DetectorConfig()
        values = {
            field_name: rospy.get_param(f"~{field_name}", getattr(defaults, field_name))
            for field_name in defaults.__dataclass_fields__
        }
        return DetectorConfig(**values)

    def _image_callback(self, image_message: Image) -> None:
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            mask, detections = self.detector.detect(bgr_image)
        except (CvBridgeError, ValueError) as error:
            rospy.logerr_throttle(2.0, "Cannot process RGB image: %s", error)
            return

        result_message = HazardDetection2DArray()
        result_message.header = image_message.header
        result_message.image_width = image_message.width
        result_message.image_height = image_message.height
        result_message.detections = [self._to_ros_message(item) for item in detections]
        self.detections_publisher.publish(result_message)

        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = image_message.header
        self.mask_publisher.publish(mask_message)

        debug_image = self.detector.annotate(bgr_image, detections)
        debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_message.header = image_message.header
        self.debug_image_publisher.publish(debug_message)

    @staticmethod
    def _to_ros_message(detection):
        message = HazardDetection2D()
        message.x = detection.x
        message.y = detection.y
        message.width = detection.width
        message.height = detection.height
        message.center_x = detection.center_x
        message.center_y = detection.center_y
        message.area = detection.area
        message.circularity = detection.circularity
        message.aspect_ratio = detection.aspect_ratio
        return message


def main():
    rospy.init_node("hazard_detector")
    try:
        HazardDetectorNode()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Invalid hazard detector configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
