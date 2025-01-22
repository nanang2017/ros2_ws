#1. urdf parsing
from urdf_parser_py.urdf import URDF
import os
from ament_index_python.packages import get_package_share_directory
pkg_path = os.path.join(get_package_share_directory('gazebo_tutorial'))
urdf_file = os.path.join(pkg_path, "vision60", "vision60_single.urdf")
robot = URDF.from_xml_file(urdf_file)

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

"""
output

ros2 run gazebo_tutorial node_test
Unknown tag "contact" in /robot[@name='ngr']/link[@name='toe0']
Unknown tag "contact" in /robot[@name='ngr']/link[@name='toe1']
Unknown tag "contact" in /robot[@name='ngr']/link[@name='toe2']
Unknown tag "contact" in /robot[@name='ngr']/link[@name='toe3']
till here -> error code

['8', '0', '1', '9', '2', '3', '10', '4', '5', '11', '6', '7']
{'8': 'revolute', '0': 'revolute', '1': 'revolute', '9': 'revolute', '2': 'revolute', '3': 'revolute', '10': 'revolute', '4': 'revolute', '5': 'revolute', '11': 'revolute', '6': 'revolute', '7': 'revolute'}
{'8': {'lower': -0.43, 'upper': 0.43, 'effort': 375.0, 'velocity': 8.6}, '0': {'lower': -3.14159265359, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '1': {'lower': 0.0, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '9': {'lower': -0.43, 'upper': 0.43, 'effort': 375.0, 'velocity': 8.6}, '2': {'lower': -3.14159265359, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '3': {'lower': 0.0, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '10': {'lower': -0.43, 'upper': 0.43, 'effort': 375.0, 'velocity': 8.6}, '4': {'lower': -3.14159265359, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '5': {'lower': 0.0, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '11': {'lower': -0.43, 'upper': 0.43, 'effort': 375.0, 'velocity': 8.6}, '6': {'lower': -3.14159265359, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}, '7': {'lower': 0.0, 'upper': 3.14159265359, 'effort': 87.5, 'velocity': 30.0}}

"""

#specifications
"""
get random position value from np.random
compare it to the joint_limits & cut values

"""



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

position = np.zeros(14).tolist()

class controller(Node):
    def __init__(self):
        super().__init__('contoller_joint')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_forces)

    def publish_joint_forces(self):

        force = []

        force = 10*(np.random.rand(12))
        force = force.tolist()

        position_value = np.random.randn(12)
        position_value = position_value.tolist()

        try:
            self.position += position_value
            print("adding successful")
            print(position_value)
        except:
            self.position = np.zeros(12).tolist()
            print("initialize")

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.effort = [] #force
        joint_state.position =self.position
        joint_state.velocity = []
        #print(joint_state.position)
        #print(joint_state.effort)
        self.publisher.publish(joint_state)



def main(args=None):
    #main function call
    print(joint_names, joint_types, joint_limits)
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    """main function"""
    main()