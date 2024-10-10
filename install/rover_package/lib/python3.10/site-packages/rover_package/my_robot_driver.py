'''import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.1029

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__wheel1 = self.__robot.getDevice('one_drive_joint')
        self.__wheel2 = self.__robot.getDevice('two_drive_joint')
        self.__wheel3 = self.__robot.getDevice('three_drive_joint')
        self.__wheel4 = self.__robot.getDevice('four_drive_joint')
        self.__wheel5 = self.__robot.getDevice('five_drive_joint')
        self.__wheel6 = self.__robot.getDevice('six_drive_joint')

        self.__wheel1.setPosition(float('inf'))
        self.__wheel1.setVelocity(0)
        self.__wheel2.setPosition(float('inf'))
        self.__wheel2.setVelocity(0)
        self.__wheel3.setPosition(float('inf'))
        self.__wheel3.setVelocity(0)
        self.__wheel4.setPosition(float('inf'))
        self.__wheel4.setVelocity(0)
        self.__wheel5.setPosition(float('inf'))
        self.__wheel5.setVelocity(0)
        self.__wheel6.setPosition(float('inf'))
        self.__wheel6.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed) / WHEEL_RADIUS
        self.get_logger().info(f'Testing...')
        self.__wheel1.setVelocity(command_motor_left)
        self.__wheel3.setVelocity(command_motor_left)
        self.__wheel5.setVelocity(command_motor_left)
        self.__wheel2.setVelocity(command_motor_right)
        self.__wheel4.setVelocity(command_motor_right)
        self.__wheel6.setVelocity(command_motor_right)'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from webots_ros2_driver import Robot

WHEEL_RADIUS = 0.1029

class MyRobotDriver(Node):
    def __init__(self):
        super().__init__('my_robot_driver')
        self.__robot = Robot()  # Initialize the Webots Robot
        self.__wheel1 = self.__robot.getDevice('one_drive_joint')
        self.__wheel2 = self.__robot.getDevice('two_drive_joint')
        self.__wheel3 = self.__robot.getDevice('three_drive_joint')
        self.__wheel4 = self.__robot.getDevice('four_drive_joint')
        self.__wheel5 = self.__robot.getDevice('five_drive_joint')
        self.__wheel6 = self.__robot.getDevice('six_drive_joint')

        # Set wheels to velocity control mode
        for wheel in [self.__wheel1, self.__wheel2, self.__wheel3, self.__wheel4, self.__wheel5, self.__wheel6]:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0)

        # Create a subscription to cmd_vel topic
        self.__subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.__cmd_vel_callback,
            10
        )
        self.__target_twist = Twist()
        self.get_logger().info('MyRobotDriver node has been started.')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        self.__robot.step(int(self.__robot.getBasicTimeStep()))

        # Compute motor commands based on target velocities
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed) / WHEEL_RADIUS

        # Set motor velocities
        self.__wheel1.setVelocity(command_motor_left)
        self.__wheel3.setVelocity(command_motor_left)
        self.__wheel5.setVelocity(command_motor_left)
        self.__wheel2.setVelocity(command_motor_right)
        self.__wheel4.setVelocity(command_motor_right)
        self.__wheel6.setVelocity(command_motor_right)

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotDriver()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.step()  # Advance the Webots simulation
    finally:
        node.destroy_node()
        rclpy.shutdown()
