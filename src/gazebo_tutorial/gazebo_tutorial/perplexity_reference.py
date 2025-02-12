#1. urdf parsing
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file("path_to_your_urdf_file.urdf")


#2. extract joint informations
joint_names = []
joint_types = {}
joint_limits = {}

for joint in robot.joints:
    if joint.type != "fixed":
        joint_names.append(joint.name)
        joint_types[joint.name] = joint.type
        if joint.limit:
            joint_limits[joint.name] = {
                "lower": joint.limit.lower,
                "upper": joint.limit.upper,
                "effort": joint.limit.effort,
                "velocity": joint.limit.velocity
            }


#3. create node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

#QoS: quality of service, communication protocol

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [0.0] * len(joint_names)  # Initialize with zeros
        msg.velocity = []
        msg.effort = []
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_state_publisher = RobotStatePublisher()
    rclpy.spin(robot_state_publisher)
    robot_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#4. update joint positions using susbcription
#make controller object to simplify creating signals
from std_msgs.msg import Float64MultiArray

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10)
        self.joint_positions = [0.0] * len(joint_names)

    def joint_command_callback(self, msg):
        for i, position in enumerate(msg.data):
            if i < len(self.joint_positions):
                joint_name = joint_names[i]
                if joint_name in joint_limits:
                    lower = joint_limits[joint_name]['lower']
                    upper = joint_limits[joint_name]['upper']
                    self.joint_positions[i] = max(lower, min(upper, position))
                else:
                    self.joint_positions[i] = position

    def get_joint_positions(self):
        return self.joint_positions


#5. final, gathered
class RobotStatePublisher(Node):
    def __init__(self, controller):
        super().__init__('robot_state_publisher')
        self.controller = controller
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = self.controller.get_joint_positions()
        msg.velocity = []
        msg.effort = []
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    robot_state_publisher = RobotStatePublisher(controller)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(robot_state_publisher)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
