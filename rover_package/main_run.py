import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
import os
from random import randint
from rclpy.qos import qos_profile_sensor_data
from flask import Flask,render_template_string,Response
import signal, sys
from threading import Thread, Event
import time
from threading import Lock

MAX_RANGE = 0.1
n = 1

cv_image_lock = Lock()

cv_image = None # Global variable frame (the holy image)
app = Flask(__name__)
print("Start")


class MainRun(Node):
    def __init__(self):
        super().__init__('main_run')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        command_message = Twist()

        while rclpy.ok():
            #self.get_logger().info(f'Testing...')
            command_message.linear.x = 0.1
            self.__publisher.publish(command_message)
        #self.bridge = CvBridge()
        #self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Image, '/zedtwo_realsensecolor/image_raw', self.__camera_callback, 1)
        #self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

    def __camera_callback(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            img_name = "saved_image.jpg"
            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(img_gray, 127, 255, 0)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)             
            # draw contours on the original image
            cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)    
            cv_image = cv2.resize(cv_image, (1000, 800)) 
            cv2.imshow("Black hole sun", cv_image)
            cv2.waitKey(1)
            #cv2.imwrite(img_name, cv_image)
            self.get_logger().info(f'Saved camera image: {img_name}')
        except Exception as e:
            self.get_logger().error('Failed to process image: ' + str(e))

'''
    def __left_sensor_callback(self, message):
        global n
        self.__left_sensor_value = message.range

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__left_sensor_value < 0.9 * MAX_RANGE:
            if n:
                command_message.angular.z = -2.0
            else:
                command_message.angular.z = 2.0
        else:
            n = 1
        self.__publisher.publish(command_message)

    

    def __right_sensor_callback(self, message):
        global n
        self.__right_sensor_value = message.range

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            if n:
                command_message.angular.z = -2.0
            else:
                command_message.angular.z = 2.0
        else:
            n = 1

        self.__publisher.publish(command_message)'''


def main(args=None):
    rclpy.init(args=args)
    avoider = MainRun()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  