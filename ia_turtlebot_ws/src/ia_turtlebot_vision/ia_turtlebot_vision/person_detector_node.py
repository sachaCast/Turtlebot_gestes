import os
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge
import cv2
import numpy as np

FILE_DIR = os.path.dirname(__file__)  
MODELS_DIR = os.path.join(FILE_DIR, 'models')

MODEL_PROTOTXT = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.prototxt.txt")
MODEL_WEIGHTS  = os.path.join(MODELS_DIR, "MobileNetSSD_deploy.caffemodel")

CLASSES = [
"background", "aeroplane", "bicycle", "bird", "boat",
"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
"dog", "horse", "motorbike", "person", "pottedplant",
"sheep", "sofa", "train", "tvmonitor"
]

PERSON_CLASS_ID = CLASSES.index("person")

class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # example but might change
            self.image_callback,
            10
        )

        # data = [person_present, center_x_norm, center_y_norm, size_norm] (might also change)
        self.person_pub = self.create_publisher(
            Float32MultiArray,
            '/person_detection',
            10
        )

        self.get_logger().info('Chargement du modèle MobileNet SSD...')
        self.net = cv2.dnn.readNetFromCaffe(MODEL_PROTOTXT, MODEL_WEIGHTS)
        self.get_logger().info('Modèle MobileNet SSD chargé.')
        self.get_logger().info('PersonDetectorNode initialisé')

    def image_callback(self, msg: Image) -> None:
        # ROS -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erreur cv_bridge: {e}')
            return

        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)),
            0.007843,  
            (300, 300),
            127.5
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        best_conf = 0.0
        best_box = None

        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence < 0.5:
                continue

            class_id = int(detections[0, 0, i, 1])
            if class_id != PERSON_CLASS_ID:
                continue

            box = detections[0, 0, i, 3:7]
            x1 = int(box[0] * w)
            y1 = int(box[1] * h)
            x2 = int(box[2] * w)
            y2 = int(box[3] * h)

            if confidence > best_conf:
                best_conf = confidence
                best_box = (x1, y1, x2, y2)

        if best_box is None:
            person_present = 0.0
            center_x_norm = 0.0
            center_y_norm = 0.0
            size_norm = 0.0
        else:
            x1, y1, x2, y2 = best_box
            person_present = 1.0
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            center_x_norm = cx / w
            center_y_norm = cy / h
            box_area = max(0, x2 - x1) * max(0, y2 - y1)
            size_norm = box_area / float(w * h)

        out_msg = Float32MultiArray()
        out_msg.data = [person_present, center_x_norm, center_y_norm, size_norm]
        self.person_pub.publish(out_msg)

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f'person {best_conf:.2f}',
                (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

        cv2.imshow('person_detector_debug', frame)
        cv2.waitKey(1)

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PersonDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == 'main':
    main()