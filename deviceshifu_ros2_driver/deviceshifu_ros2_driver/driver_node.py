import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Image  # 导入Image消息类型
import cv2
from cv_bridge import CvBridge  # 导入CvBridge用于图像转换
import numpy as np  # 导入numpy用于生成纯色图像
import time

class DeviceShifuROS2Driver(Node):
    def __init__(self):
        super().__init__('deviceshifu_ros2_driver')
        # 创建一个发布者，用于发布控制指令
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 创建一个发布者，用于发布摄像头图像
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        # 创建一个订阅者，用于接收远程控制指令
        self.command_sub = self.create_subscription(Int32, 'remote_command', self.command_callback, 10)
        # 创建一个定时器，用于定时发布指令和图像
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.current_velocity = Twist()
        self.bridge = CvBridge()

    def timer_callback(self):
        """定时器回调函数，用于发布速度和图像"""
        # 发布当前速度
        self.publisher_.publish(self.current_velocity)
        
        # 发布默认图像
        self.publish_default_image()

    def command_callback(self, msg):
        """处理远程控制指令"""
        if msg.data == 0:  # 停止
            self.stop()
        elif msg.data == 1:  # 向前移动
            self.move_forward(0.5)
        elif msg.data == 2:  # 向后移动
            self.move_backward(0.5)
        elif msg.data == 3:  # 旋转
            self.rotate(0.2)

    def publish_default_image(self):
        """发布720x480的纯色图像"""
        try:
            # 创建纯色图像 (#39c5bb 在BGR格式下是 [187, 197, 57])
            default_image = np.zeros((480, 720, 3), dtype=np.uint8)
            # OpenCV使用BGR格式，所以颜色值需要反转
            default_image[:, :] = [187, 197, 57]  # BGR格式的 #39c5bb
            
            # 转换为ROS消息并发布
            image_msg = self.bridge.cv2_to_imgmsg(default_image, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"
            self.image_pub.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing default image: {str(e)}')

    def move_forward(self, speed=0.5):
        """向前移动"""
        self.current_velocity.linear.x = speed
        self.current_velocity.angular.z = 0.0

    def move_backward(self, speed=0.5):
        """向后移动"""
        self.current_velocity.linear.x = -speed
        self.current_velocity.angular.z = 0.0

    def rotate(self, angular_speed):
        """旋转"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = angular_speed

    def stop(self):
        """停止"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    driver_node = DeviceShifuROS2Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()