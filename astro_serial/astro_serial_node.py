#!/usr/bin/env python3
import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class AstroSerial(Node):
    def __init__(self):
        super().__init__("astro_serial_node")
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState, 
            "/joint_states",
            10)
        
        period = 0.0001
        self.timer = self.create_timer(period, self.timer_callback, clock=self.get_clock())

        self.wheelRadius = 0.036
        self.wheelSeparation = 0.289
        self.maxLinearSpeed = 0.75
        self.maxAngularSpeed = 5.1
        self.serial = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

    
    def sendVelocity(self, linearX, angularZ):
        w_l = (linearX / self.wheelRadius) - ((angularZ * self.wheelSeparation) / (2 * self.wheelRadius))
        w_r = (linearX / self.wheelRadius) + ((angularZ * self.wheelSeparation) / (2 * self.wheelRadius))
        if (abs(w_l) < 1):
            w_l = 0
        if (abs(w_r) < 1):
            w_r = 0
        command = f"v {w_l} {w_r}\n"

        print(f"Sending linear: {linearX} angular: {angularZ} w_l: {w_l} w_r: {w_r}")
        self.serial.write(bytes(command, 'utf-8'))

    def callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.sendVelocity(linearX=linear_x, angularZ=angular_z)


    def timer_callback(self):
        if self.serial.in_waiting:
            rawJointStates = self.serial.readline().decode().strip('\n')
            rawJointStates_list = [float(e) for e in rawJointStates.split()]

            msg = JointState()
            msg.name = ["left_motor_joint", "right_motor_joint"]
            msg.position = rawJointStates_list[0:2]
            msg.velocity = rawJointStates_list[2:4]
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            self.publisher.publish(msg)    

def main(args=None):
    rclpy.init(args=args)

    astroSerial = AstroSerial()
    rclpy.spin(astroSerial)

    astroSerial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
