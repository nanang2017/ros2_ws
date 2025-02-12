import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('cmd_vel_xy_example')

    # cmd_vel 토픽에 Twist 메시지를 발행할 퍼블리셔 생성
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    # Twist 메시지 생성 및 원하는 속도 값 설정 (x, y 축 속도)
    twist = Twist()
    twist.linear.x = 0.5   # x축 선형 속도 (m/s)
    twist.linear.y = 0.3   # y축 선형 속도 (m/s)
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    node.get_logger().info('Publishing cmd_vel (x, y) for 2 seconds...')

    # 2초 동안 10Hz 주기로 속도 명령 발행
    start_time = time.time()
    while time.time() - start_time < 2.0:
        publisher.publish(twist)
        node.get_logger().info('Publishing: linear.x=0.5, linear.y=0.3')
        time.sleep(0.1)  # 0.1초 간격 (10Hz)

    # 2초 후에 정지 명령 발행 (속도를 0으로)
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    publisher.publish(twist)
    node.get_logger().info('Publishing stop command: linear.x=0.0, linear.y=0.0')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
