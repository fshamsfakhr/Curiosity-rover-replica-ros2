#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import math

class ScanControl(Node):
    def __init__(self):
        self.start_flg = 0
        self.a = 6.13
        self.b = .3
        self.speed = 1.0
        self.counter = 0
        super().__init__('scan_control')
        self.scan_pos_publisher = self.create_publisher(Float64, '/scan_position', 10)
        self.scan_vel_publisher = self.create_publisher(Float64, '/scan_vel', 10)
        self.subscription = self.create_subscription(JointState,'/joint_states',self.joint_states_callback,10)
        print(f"Subscribing to /joint_states topic. Press Ctrl+C to exit.")

    def wrapto2pi(self, angle):
        return angle % (2 * np.pi)

    def joint_states_callback(self, msg):
        if len(msg.position) >= 11:
            angle = self.wrapto2pi(msg.position[10])
            # print(f"Wrapped Position at index 10: {angle}")
            self.counter = self.counter + 1
            if(self.start_flg==0):
                self.direction = self.set_direction(angle, self.a, self.b)
                self.start_flg = 1
            else:
                if(self.is_angle_in_range(angle, self.a, self.b)!=0):
                    self.direction = self.set_direction(angle, self.a, self.b)
                    self.counter = 0
                    
            self.publish_scan_pos(angle)
            self.publish_scan_vel(self.speed*self.direction)

    def publish_scan_pos(self, value):
        self.scan_pos_publisher.publish(Float64(data=value))
        
    def publish_scan_vel(self, value):
        self.scan_vel_publisher.publish(Float64(data=value))

    def angdiff(self, a, b):
        diff = (b - a + math.pi) % (2 * math.pi) - math.pi
        
        return diff 
    
    def is_angle_in_range(self, angle, range_start, range_end):
        angle = angle % (2 * math.pi)
        range_start = range_start % (2 * math.pi)
        range_end = range_end % (2 * math.pi)
        # Check if the angle is within the range
        if range_start <= angle <= range_end or range_start > range_end and (range_start <= angle or angle <= range_end):
            offset = 0
        else:
            diff_start = self.angdiff(angle,range_start)
            diff_end = self.angdiff(angle,range_end)
            if(abs(diff_start)<=abs(diff_end)):
                offset = diff_start
            else:
                offset = diff_end
    
        return offset

    def set_direction(self, c, a, b):
        # Set up initial values
        offset = self.is_angle_in_range(c, a, b)
        if(offset==0):
            direction = 1
        elif(offset<0):
            direction = -1
        else:
            direction = 1
        
        return direction

def main(args=None):
    rclpy.init(args=args)
    scan_control = ScanControl()
    rclpy.spin(scan_control)
    scan_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()