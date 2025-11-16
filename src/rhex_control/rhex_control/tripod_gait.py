#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class TripodGait(Node):
    def __init__(self):
        super().__init__('tripod_gait')
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controllers/commands', 10)

        # Joint order in config: L_FL, L_FR, L_ML, L_MR, L_RL, L_RR
        # Define tripods:
        self.tripod1 = [0, 3, 4]  # FL, MR, RL
        self.tripod2 = [1, 2, 5]  # FR, ML, RR

        self.angle = 0.0
        self.direction = 1
        self.timer = self.create_timer(3.0, self.update_motion)

        self.get_logger().info('Tripod gait controller started')

    def update_motion(self):
        msg = Float64MultiArray()
        pos = [0.0] * 6

        # Move one tripod at a time
        active_tripod = self.tripod1 if self.direction == 1 else self.tripod2
        for i in active_tripod:
            pos[i] = math.radians(360)  # one full rotation

        msg.data = pos
        self.publisher.publish(msg)

        self.get_logger().info(f'Moved {"Tripod 1" if self.direction == 1 else "Tripod 2"}')
        self.direction *= -1  # Alternate


def main(args=None):
    rclpy.init(args=args)
    node = TripodGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
