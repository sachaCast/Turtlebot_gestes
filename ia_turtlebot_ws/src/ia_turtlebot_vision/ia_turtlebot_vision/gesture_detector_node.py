import os
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.python._framework_bindings import image as mp_image
from mediapipe.python._framework_bindings import image_frame as mp_image_frame



class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__('gesture_detector_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/gesture_cmd',
            10
        )

        self.get_logger().info('Chargement du modèle de gestes MediaPipe...')

        FILE_DIR = os.path.dirname(__file__)  
        MODELS_DIR = os.path.join(FILE_DIR, 'models')
        MODEL_PATH = os.path.join(MODELS_DIR, 'gesture_recognizer.task')

        base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.IMAGE,
        )
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self.get_logger().info('Modèle de gestes chargé.')
        self.get_logger().info('GestureDetectorNode initialisé.')

    def image_callback(self, msg: Image) -> None:
        self.get_logger().info('image_callback called')

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erreur cv_bridge: {e}')
            return

        if frame is None:
            self.get_logger().warn('Frame is None')
            return

        try:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # creating MediaPipe image
            mp_img = mp_image.Image(
                image_format=mp_image_frame.ImageFormat.SRGB,
                data=rgb_image
            )

            result = self.recognizer.recognize(mp_img)
        except Exception as e:
            self.get_logger().error(f'Erreur MediaPipe recognize: {e}')
            return

        gesture_label = self.extract_top_gesture(result)
        if gesture_label is None:
            return

        cmd = self.gesture_to_command(gesture_label)
        if cmd is None:
            return

        msg_cmd = String()
        msg_cmd.data = cmd
        self.cmd_pub.publish(msg_cmd)
        self.get_logger().info(f'Geste détecté: {gesture_label} -> commande: {cmd}')

    @staticmethod
    def extract_top_gesture(result):
        if not result.gestures:
            return None
        main_hand_gestures = result.gestures[0]
        if not main_hand_gestures:
            return None
        top_gesture = max(main_hand_gestures, key=lambda g: g.score)
        if top_gesture.category_name in ['None', 'Unknown']:
            return None
        return top_gesture.category_name

    @staticmethod
    def gesture_to_command(gesture_label: str) -> Optional[str]:
        mapping = {
            'Thumb_Up': 'forward',
            'Thumb_Down': 'backward',
            'Open_Palm': 'stop',
            'Pointing_Up': 'right',
            'Closed_Fist': 'left',
            'Victory': 'listen'
        }
        return mapping.get(gesture_label, None)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GestureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
