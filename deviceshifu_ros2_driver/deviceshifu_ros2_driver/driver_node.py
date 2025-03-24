#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from pynput import keyboard
import threading

class DeviceShifuDriver(Node):
    def __init__(self):
        super().__init__('deviceshifu_driver')
        
        # 初始化参数
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('camera_fps', 30)
        
        # 获取参数值
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.camera_fps = self.get_parameter('camera_fps').value
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_pub = self.create_publisher(
            Image, 
            'camera/image',  # 确保使用正确的话题名称
            10
        )
        
        # 创建订阅者 - 用于接收控制命令
        self.command_sub = self.create_subscription(
            Int32,
            'remote_command',
            self.command_callback,
            10)
            
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.init_camera()
        
        # 创建定时器用于发布图像
        self.create_timer(1.0/self.camera_fps, self.publish_image)
        
        # 初始化键盘状态
        self.key_states = {
            'w': False,
            'a': False,
            's': False,
            'd': False
        }
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.start_keyboard_listener)
        self.keyboard_thread.daemon = True  # 设置为守护线程
        self.keyboard_thread.start()
        
        # 创建定时器用于处理键盘控制
        self.create_timer(0.1, self.keyboard_control_callback)
        
        self.get_logger().info('DeviceShifu驱动已初始化')

    def init_camera(self):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.get_logger().info('成功连接摄像头')
                self.has_camera = True
            else:
                raise Exception("无法打开摄像头")
        except Exception as e:
            self.get_logger().warn(f'摄像头初始化失败: {str(e)}')
            self.get_logger().info('将使用模拟图像')
            self.has_camera = False
            self.cap = None

    def publish_image(self):
        """发布图像数据"""
        if self.has_camera:
            # 使用实际摄像头
            ret, frame = self.cap.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(msg)
                    self.get_logger().debug('已发布实际摄像头图像')
                except Exception as e:
                    self.get_logger().error(f'图像发布失败: {str(e)}')
        else:
            # 发布模拟图像
            try:
                # 创建纯色图像 (#39c5bb)
                image = np.zeros((480, 720, 3), dtype=np.uint8)
                # 注意：OpenCV使用BGR格式
                image[:] = [187, 197, 57]  # BGR格式的 #39c5bb
                
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)
                # self.get_logger().info('已发布模拟图像')  # 添加日志
            except Exception as e:
                self.get_logger().error(f'模拟图像发布失败: {str(e)}')

    def command_callback(self, msg):
        """处理控制命令"""
        cmd = Twist()
        
        if msg.data == 0:    # 停止
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('执行停止命令')
        elif msg.data == 1:  # 前进
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('执行前进命令')
        elif msg.data == 2:  # 后退
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('执行后退命令')
        elif msg.data == 3:  # 左转
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info('执行左转命令')
        elif msg.data == 4:  # 右转
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.get_logger().info('执行右转命令')
        else:
            self.get_logger().warn(f'未知命令: {msg.data}')
            return
            
        self.cmd_vel_pub.publish(cmd)

    def on_press(self, key):
        """键盘按下回调"""
        try:
            key_char = key.char.lower()
            if key_char in self.key_states:
                self.key_states[key_char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        """键盘释放回调"""
        try:
            key_char = key.char.lower()
            if key_char in self.key_states:
                self.key_states[key_char] = False
        except AttributeError:
            pass

    def start_keyboard_listener(self):
        """启动键盘监听"""
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def keyboard_control_callback(self):
        """处理键盘控制"""
        cmd = Twist()
        
        # 处理前后移动
        if self.key_states['w'] and not self.key_states['s']:
            cmd.linear.x = self.linear_speed
        elif self.key_states['s'] and not self.key_states['w']:
            cmd.linear.x = -self.linear_speed
        
        # 处理左右转向
        if self.key_states['a'] and not self.key_states['d']:
            cmd.angular.z = self.angular_speed
        elif self.key_states['d'] and not self.key_states['a']:
            cmd.angular.z = -self.angular_speed
        
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """清理资源"""
        if self.has_camera and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = DeviceShifuDriver()
        rclpy.spin(driver)
    except Exception as e:
        print(f'发生错误: {str(e)}')
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
