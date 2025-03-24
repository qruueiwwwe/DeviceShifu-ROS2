import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Image  # 导入Image消息类型
import cv2
from cv_bridge import CvBridge  # 导入CvBridge用于图像转换

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
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.current_velocity = Twist()
        self.bridge = CvBridge()  # 初始化CvBridge

        # 假设有一个方法来获取摄像头图像
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
        # 这里假设您有一个方法来获取图像，例如使用OpenCV
        cap = cv2.VideoCapture(0)  # 打开默认摄像头
        while True:
            ret, frame = cap.read()
            if ret:
                # 将图像转换为ROS消息并发布
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(image_msg)
                self.get_logger().info('Publishing camera image')
            else:
                self.get_logger().error('Failed to capture image')
            rclpy.sleep(0.1)  # 控制发布频率

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