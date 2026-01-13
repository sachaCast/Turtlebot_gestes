#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor')

        self.gesture_sub = self.create_subscription(String, '/gesture_cmd', self.gesture_callback, 10)
        self.human_sub = self.create_subscription(Bool, '/human_detected', self.human_callback, 10)

        self.move_pub = self.create_publisher(String, '/robot_operation_mode', 10)

        self.current_state = 'EXPLORING'
        self.get_logger().info("Superviseur initialisé. État : EXPLORING")

    def human_callback(self, msg: Bool):
        if msg.data and self.current_state == 'EXPLORING':
            self.get_logger().info("Humain détecté ! Passage en mode approche/attente.")
            self.current_state = 'WAITING_FOR_GESTURE'
            self.send_move_command("listen")

    def gesture_callback(self, msg: String):
        command = msg.data.lower()

        if command == "victory":
            self.current_state = 'WAITING_FOR_GESTURE'
            self.get_logger().info("Mode écoute activé via geste.")

        elif command == "stop":
            self.current_state = 'EXPLORING'

        self.send_move_command(command)

    def send_move_command(self, cmd_string):
        msg = String()
        msg.data = cmd_string
        self.move_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
