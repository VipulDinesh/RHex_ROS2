#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TripodGait(Node):
    def __init__(self):
        super().__init__('tripod_gait')
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controllers/commands', 10)

        # Joint order: L_FL, L_FR, L_ML, L_MR, L_RL, L_RR
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Tripods
        self.tripod1 = [0, 3, 4]  # FL, MR, RL
        self.tripod2 = [1, 2, 5]  # FR, ML, RR

        self.direction = 1  # Start with tripod1
        self.increment = 6.28  # Full rotation in radians

        self.timer = self.create_timer(3.0, self.update_motion)
        self.get_logger().info('Tripod gait controller started')

    def update_motion(self):
        # Select active tripod
        active_tripod = self.tripod1 if self.direction == 1 else self.tripod2

        # Apply increments for active tripod
        for i in active_tripod:
            if i % 2 == 0:  # Left side joints: FL, ML, RL
                self.joint_positions[i] -= self.increment
            else:  # Right side joints: FR, MR, RR
                self.joint_positions[i] += self.increment

        # Publish updated positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)

        self.get_logger().info(f'Moved {"Tripod 1" if self.direction == 1 else "Tripod 2"}')
        self.direction *= -1  # Alternate tripods

def main(args=None):
    rclpy.init(args=args)
    node = TripodGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
