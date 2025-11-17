#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class AlternatingTripodGait(Node):
    def __init__(self):
        super().__init__('alternating_tripod_gait')
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controllers/commands', 10)

        # Joint order: L_FL, L_FR, L_ML, L_MR, L_RL, L_RR
        self.joint_positions = [0.0] * 6

        # Tripods
        self.tripod1 = [0, 3, 4]  # FL, MR, RL
        self.tripod2 = [1, 2, 5]  # FR, ML, RR

        self.active_tripod = self.tripod1
        self.increment = 0.05  # radians per timer tick
        self.timer_period = 0.01  # seconds
        self.rotation_done = 0.0  # track rotation of active tripod

        self.timer = self.create_timer(self.timer_period, self.update_motion)
        self.get_logger().info('Alternating tripod gait controller started')

    def update_motion(self):
        # Move only the active tripod
        for i in self.active_tripod:
            if i % 2 == 0:  # Left side
                self.joint_positions[i] -= self.increment
            else:  # Right side
                self.joint_positions[i] += self.increment

        self.rotation_done += self.increment  # Track active tripod rotation

        # Publish joint positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)

        # Switch tripod after full rotation (~6.28 rad)
        if self.rotation_done >= 6.28:
            self.rotation_done = 0.0
            self.active_tripod = self.tripod2 if self.active_tripod == self.tripod1 else self.tripod1

def main(args=None):
    rclpy.init(args=args)
    node = AlternatingTripodGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
