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
        # 创建一个定时器，用于定时发布指令
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.current_velocity = Twist()
        self.current_velocity = Twist()
        self.bridge = CvBridge()  # 初始化CvBridge

        # 捕获摄像头图像
        self.capture_camera_image()

    def timer_callback(self):
        # 发布当前速度
        self.publisher_.publish(self.current_velocity)
        self.get_logger().info('Publishing: "%s"' % self.current_velocity)

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

    def capture_camera_image(self):
        """捕获摄像头图像并发布"""
        try:
            cap = cv2.VideoCapture(0)  # 尝试打开默认摄像头
            if not cap.isOpened():
                raise Exception("Camera not opened")  # 如果摄像头未打开，抛出异常

            while rclpy.ok():
                ret, frame = cap.read()
                if ret:
                    # 将图像转换为ROS消息并发布
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.image_pub.publish(image_msg)
                    self.get_logger().info('Publishing camera image')
                else:
                    self.get_logger().error('Failed to capture image')
                time.sleep(0.1)  # 控制发布频率

        except Exception as e:
            self.get_logger().warn(f'Using default image due to: {str(e)}')
            self.publish_default_image()

    def publish_default_image(self):
        """发布720x480的纯色图像"""
        color = (57, 197, 187)  # RGB颜色 #39c5bb
        default_image = np.full((480, 720, 3), color, dtype=np.uint8)  # 创建纯色图像
        image_msg = self.bridge.cv2_to_imgmsg(default_image, encoding='bgr8')
        self.image_pub.publish(image_msg)
        self.get_logger().info('Publishing default color image')

    def move_forward(self, speed=0.5):
        """Move forward"""
        cmd = Twist()
        cmd.linear.x = speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def move_backward(self, speed=0.5):
        """Move backward"""
        cmd = Twist()
        cmd.linear.x = -speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def rotate(self, angular_speed):
        """Rotate"""
        cmd = Twist()
        cmd.angular.z = angular_speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)
        
    def stop(self):
        """Stop"""
        cmd = Twist()
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    driver_node = DeviceShifuROS2Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()