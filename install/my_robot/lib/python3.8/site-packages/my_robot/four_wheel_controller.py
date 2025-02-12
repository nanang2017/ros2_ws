import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FourWheelControl(Node):
    def __init__(self):
        super().__init__('four_wheel_control')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Linear: {msg.linear.x}, {msg.linear.y}, {msg.linear.z} | Angular: {msg.angular.x}, {msg.angular.y}, {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = FourWheelControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
