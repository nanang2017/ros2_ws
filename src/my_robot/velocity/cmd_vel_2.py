#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class OmniWheelController(Node):
    def __init__(self):
        super().__init__('omni_wheel_controller')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # 로봇 크기
        self.Lx = 0.4
        self.Ly = 0.24
        self.R = 0.05  # 바퀴 반지름

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z

        # 옴니휠 역기구학
        fl = (1/self.R) * (vx - vy - (self.Lx+self.Ly)*w)
        fr = (1/self.R) * (vx + vy + (self.Lx+self.Ly)*w)
        rl = (1/self.R) * (vx + vy - (self.Lx+self.Ly)*w)
        rr = (1/self.R) * (vx - vy + (self.Lx+self.Ly)*w)

        cmd = Float64MultiArray()
        cmd.data = [fl, fr, rl, rr]

        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
