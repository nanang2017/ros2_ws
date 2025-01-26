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

length = len(joint_names)

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
substitute the values to message.name & message.position

perplexity code summary
1.robot state publisher class
: create publisher using JointState, create msg topic and publish -> directedly used in main
2. robot controller class
:when subscription joint_commands arrive, callback -> check joint name & position cut & array creation
3. final robot state publisher class
: create controller, create publisher, get position from controller 
"""



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.qos import QoSProfile

position = np.zeros(14).tolist()

class controller(Node):
    def __init__(self):
        super().__init__('contoller')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_joint_position)
        self.count = 50

    def publish_joint_position(self):

        self.count += 1

        if self.count > 5:
            self.count = 0
            position_diff = np.random.randn(length)/10
            #position_value = position_value.tolist()

            try:
                self.position += position_diff
                print("adding successful")
                for i, name in enumerate(joint_names):
                    upper_lim = joint_limits[name]["upper"]
                    lower_lim = joint_limits[name]["lower"]
                    self.position[i] = min(upper_lim, max(lower_lim, self.position[i]))

                #self.position = min(limit checking..) #limit check needed
                #print(position_value)
            except:
                self.position = np.zeros(length)
                print("initialize")

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = joint_names
            joint_state.effort = [] #force
            joint_state.position =self.position.tolist()
            joint_state.velocity = []
            #print(joint_state.position)
            #print(joint_state.effort)
            self.publisher.publish(joint_state)
            print(joint_state.position) # -> increasing!! what..?
            #invalid issue solved using numpy



def main(args=None):
    #main function call
    #print(joint_names, joint_types, joint_limits)
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    """main function"""
    main()