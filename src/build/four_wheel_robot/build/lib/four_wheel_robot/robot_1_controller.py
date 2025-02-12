import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class Robot1Controller(Node):
    def __init__(self):
        super().__init__('robot_1_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        wheel_radius = 0.075
        base_width = 0.6
        base_length = 0.6

        front_left_speed = (linear_x - linear_y - angular_z * (base_width + base_length)) / wheel_radius
        front_right_speed = (linear_x + linear_y + angular_z * (base_width + base_length)) / wheel_radius
        rear_left_speed = (linear_x + linear_y - angular_z * (base_width + base_length)) / wheel_radius
        rear_right_speed = (linear_x - linear_y + angular_z * (base_width + base_length)) / wheel_radius

        joint_state = JointState()
        joint_state.name = ["front_left_wheel_joint", "front_right_wheel_joint",
                            "rear_left_wheel_joint", "rear_right_wheel_joint"]
        joint_state.velocity = [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]

        self.joint_pub.publish(joint_state)
        self.get_logger().info(f'FL: {front_left_speed}, FR: {front_right_speed}, RL: {rear_left_speed}, RR: {rear_right_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = Robot1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
