import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        qos_profile = QoSProfile(depth=10)

        # Subscriber: 카메라 이미지 데이터 수신
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.show_image,
            qos_profile
        )

        # Publisher: RGB 데이터를 새로운 토픽으로 퍼블리시
        self.publisher = self.create_publisher(String, '/camera/rgb_values', qos_profile)

        self.get_logger().info("Image Processor Initialized")
        self.bridge = CvBridge()

    def show_image(self, msg):
        self.get_logger().info('Image received')

        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # OpenCV 창으로 이미지 출력
        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)

        # 전체 이미지의 평균 RGB 값 출력
        avg_r, avg_g, avg_b = self.get_avg_rgb(frame)

        # 특정 색상(예: 초록색) 픽셀 개수 계산
        green_pixel, percentage = self.get_green_pixel(frame)

        # 로그 출력
        self.get_logger().info(f"Average RGB: R={int(avg_r)}, G={int(avg_g)}, B={int(avg_b)}")
        self.get_logger().info(f"Green Pixel Count: {green_pixel}, Green Percentage: {percentage:.4f}")

        # RGB 데이터를 새로운 토픽으로 퍼블리시
        msg = String()
        msg.data = f"Average RGB: R={int(avg_r)}, G={int(avg_g)}, B={int(avg_b)}, Green Pixels: {green_pixel}"
        self.publisher.publish(msg)

    def get_avg_rgb(self, image):
        """ 전체 이미지의 평균 R, G, B 값 계산 """
        avg_r = np.mean(image[:, :, 2])  # R 채널 평균
        avg_g = np.mean(image[:, :, 1])  # G 채널 평균
        avg_b = np.mean(image[:, :, 0])  # B 채널 평균
        return avg_r, avg_g, avg_b

    def get_green_pixel(self, image):
        """ 초록색 픽셀 개수 및 비율 계산 """
        MIN_VALUE = np.array([0, 100, 0], np.uint8)
        MAX_VALUE = np.array([100, 255, 100], np.uint8)

        green_mask = cv2.inRange(image, MIN_VALUE, MAX_VALUE)
        pixel_num = cv2.countNonZero(green_mask)

        height, width, _ = image.shape
        percent = pixel_num / (height * width)

        return pixel_num, percent

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
