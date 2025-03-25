#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class DeviceShifuDriver:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # 初始化参数
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.camera_fps = rospy.get_param('~camera_fps', 30)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        
        # 创建订阅者 - 用于接收控制命令
        self.command_sub = rospy.Subscriber('remote_command', Int32, self.command_callback)
            
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.init_camera()
        
        # 创建定时器用于发布图像
        self.rate = rospy.Rate(self.camera_fps)
        
        rospy.loginfo('DeviceShifu驱动已初始化')

    def init_camera(self):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                rospy.loginfo('成功连接摄像头')
                self.has_camera = True
            else:
                raise Exception("无法打开摄像头")
        except Exception as e:
            rospy.logwarn(f'摄像头初始化失败: {str(e)}')
            rospy.loginfo('将使用模拟图像')
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
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(msg)
                    rospy.logdebug('已发布实际摄像头图像')
                except Exception as e:
                    rospy.logerr(f'图像发布失败: {str(e)}')
        else:
            # 发布模拟图像
            try:
                # 创建纯色图像 (#39c5bb)
                image = np.zeros((480, 720, 3), dtype=np.uint8)
                # 注意：OpenCV使用BGR格式
                image[:] = [187, 197, 57]  # BGR格式的 #39c5bb
                
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)
            except Exception as e:
                rospy.logerr(f'模拟图像发布失败: {str(e)}')

    def command_callback(self, msg):
        """处理控制命令"""
        cmd = Twist()
        
        if msg.data == 0:    # 停止
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo('执行停止命令')
        elif msg.data == 1:  # 前进
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo('执行前进命令')
        elif msg.data == 2:  # 后退
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo('执行后退命令')
        elif msg.data == 3:  # 左转
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            rospy.loginfo('执行左转命令')
        elif msg.data == 4:  # 右转
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            rospy.loginfo('执行右转命令')
        else:
            rospy.logwarn(f'未知命令: {msg.data}')
            return
            
        self.cmd_vel_pub.publish(cmd)

    def run(self):
        """运行节点"""
        try:
            while not rospy.is_shutdown():
                self.publish_image()
                self.rate.sleep()
        except Exception as e:
            rospy.logerr(f'发生错误: {str(e)}')
        finally:
            if self.has_camera and self.cap is not None:
                self.cap.release()

def main():
    try:
        driver = DeviceShifuDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 