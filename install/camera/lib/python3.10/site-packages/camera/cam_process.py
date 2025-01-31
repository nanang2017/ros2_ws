"""
Job Description

/camera/image_raw, compressed, compressed depth, theora topics
proess camera frame

1) visualize using CV2
2) check number of pixels -> new topic publish
3) make environment -> reward calculation, state reward action done_mask return
"""


import numpy as np
import cv2
import rclpy
from rclpy.node import Node
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

class image_processor(Node):
    def __init__(self):
        super().__init__('image_processor')
        qos_profile = QoSProfile(depth=10)
        #subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.show_image,
            qos_profile
        )
        self.subscription #prevent unused variable warning

    def show_image(self, msg):
        self.get_logger().info('image received')
        solved_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('test', solved_image)
        cv2.waitKey(1)



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = image_processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    """main function"""
    main()