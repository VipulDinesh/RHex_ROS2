#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading
import sys
import tty
import termios
from math import pi

class RHexTeleop(Node):
    def __init__(self):
        super().__init__('rhex_teleop')

        # Publisher to joint controller
        self.pub = self.create_publisher(Float64MultiArray, '/position_controllers/commands', 10)

        # Joint order: L_FL, L_FR, L_ML, L_MR, L_RL, L_RR
        self.joint_positions = [0.0]*6
        self.tripod1 = [0, 3, 4]
        self.tripod2 = [1, 2, 5]
        self.active_tripod = self.tripod1

        # Motion variables
        self.increment = 0.05
        self.rotation_done = 0.0
        self.timer_period = 0.01

        # Teleop state
        self.current_cmd = 'x'  # stop initially
        self.next_cmd = None
        self.running = True

        # Timer to update joint positions
        self.create_timer(self.timer_period, self.update_motion)

        # Start keyboard listener in separate thread
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

        self.get_logger().info("RHex teleop started.\nControls:\n  w: forward\n  s: backward\n  a: left\n  d: right\n  x: stop\n  q: quit")

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while self.running:
                ch = sys.stdin.read(1)
                if ch in ['w','a','s','d','x','q']:
                    self.next_cmd = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def update_motion(self):
        if not self.running:
            return

        # Stop state
        if self.current_cmd == 'x':
            self.publish_joints()
            if self.next_cmd is not None:
                if self.next_cmd == 'q':
                    self.running = False
                    rclpy.shutdown()
                    return
                else:
                    self.current_cmd = self.next_cmd
                    self.next_cmd = None
            return

        # Compute increments for tripod
        left_inc = 0.0
        right_inc = 0.0
        if self.current_cmd == 'w':  # forward
            left_inc = -self.increment
            right_inc = self.increment
        elif self.current_cmd == 's':  # backward
            left_inc = self.increment
            right_inc = -self.increment
        elif self.current_cmd == 'a':  # left turn
            left_inc = -self.increment
            right_inc = -self.increment
        elif self.current_cmd == 'd':  # right turn
            left_inc = self.increment
            right_inc = self.increment

        # Apply increment to active tripod
        for i in self.active_tripod:
            if i % 2 == 0:  # left side joints
                self.joint_positions[i] += left_inc
            else:           # right side joints
                self.joint_positions[i] += right_inc

        self.rotation_done += self.increment

        # Switch tripod after full rotation
        if self.rotation_done >= 2*pi:
            self.rotation_done = 0.0
            self.active_tripod = self.tripod2 if self.active_tripod == self.tripod1 else self.tripod1

            # Accept new command after rotation
            if self.next_cmd is not None:
                if self.next_cmd == 'q':
                    self.running = False
                    rclpy.shutdown()
                    return
                elif self.next_cmd == 'x':
                    self.current_cmd = 'x'
                else:
                    self.current_cmd = self.next_cmd
                self.next_cmd = None

        # Publish joint positions
        self.publish_joints()

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
