#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64  # for a floating-point scalar
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist  # Import Twist message for cmd_vel

import math

class JointVelocityPublisher(Node):
    def __init__(self, L_input, W_input, ell_input):
        self.L_input = L_input  # robot length
        self.W_input = W_input # robot width
        self.ell_input = ell_input  # wheel radius
        self.V_input = 0.0
        self.OMG_input = 0.0
        self.psi = 0.0001
        self.omg_fl = 0.0001
        self.omg_fr = 0.0001
        self.omg_ml = 0.0001
        self.omg_mr = 0.0001 
        self.scan_rot_vel = 0.0
        
        super().__init__('joint_velocity_publisher')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        self.scan_vel_publisher = self.create_publisher(Float64, 'scan_vel', 1)
        self.scan_vel_subscription = self.create_subscription(Float64, 'scan_vel', self.scan_vel_callback, 1)   

        
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 11)
        

        
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback,11)
        self.joint_state_subscription  # prevent unused variable warning
        
    def control(self):
        if ((self.V_input!=0) and (self.OMG_input==0)):
            psi = 0
            v_fl = self.V_input
            v_fr = self.V_input
            v_ml = self.V_input
            v_mr = self.V_input
        elif(abs(self.V_input)<=1.e-5):
            psi = 0
            v_fl = 0
            v_fr = 0
            v_ml = 0
            v_mr = 0           
        else:
            psi = math.pi / 2 - math.copysign(1, (self.V_input))*math.atan2(2 * (self.V_input) , (self.OMG_input * self.L_input))
            phi = math.pi / 2 - psi
            R = self.L_input * math.tan(phi) / 2
            l_fl = math.copysign(1, (self.V_input*self.OMG_input))*math.sqrt((R - self.W_input / 2) ** 2 + (self.L_input / 2) ** 2)
            l_fr = math.copysign(1, (self.V_input*self.OMG_input))*math.sqrt((R + self.W_input / 2) ** 2 + (self.L_input / 2) ** 2)
            l_ml = math.copysign(1, (self.V_input))*(R - self.W_input / 2)
            l_mr = math.copysign(1, (self.V_input))*(R + self.W_input / 2)
            v_fl = (l_fl * self.OMG_input)
            v_fr = (l_fr * self.OMG_input)
            v_ml = (l_ml * self.OMG_input)
            v_mr = (l_mr * self.OMG_input)
            
        omg_fl = v_fl / self.ell_input
        omg_fr = v_fr / self.ell_input
        omg_ml = v_ml / self.ell_input
        omg_mr = v_mr / self.ell_input
        #self.get_logger().info(f'self.V_input:, {self.V_input}, self.OMG_input:, {self.OMG_input}, ')
        return psi, omg_fl, omg_fr, omg_ml, omg_mr

    def round_to_precision(self, number, precision):
        return round(number / precision) * precision
        
    def angdiff(self, a, b):
        diff = (b - a + math.pi) % (2 * math.pi) - math.pi
        
        return diff

    def check(self, a, b):
        if (abs(self.angdiff(a, b)) <= 0.005):
            output = 0
        elif ((abs(self.angdiff(a, b)) > 0.005) and (self.angdiff(a, b) > 0.01)):
            output = -.5
        elif ((abs(self.angdiff(a, b)) > 0.005) and (self.angdiff(a, b) < 0.01)):
            output = .5
        return output

    def cmd_vel_callback(self, msg):
        # Update V_input and OMG_input from the Twist message
        self.V_input = self.round_to_precision(msg.linear.x, 0.01)  # Assuming linear.x represents V_input
        self.OMG_input = self.round_to_precision(msg.angular.z, 0.01)  # Assuming angular.z represents OMG_input
        self.get_logger().info(f'V:, {self.V_input}, OMG: , {self.OMG_input}, ')
        psi, omg_fl, omg_fr, omg_ml, omg_mr = self.control()
        self.psi = psi
        self.omg_fl = omg_fl
        self.omg_fr = omg_fr
        self.omg_ml = omg_ml
        self.omg_mr = omg_mr 

    def scan_vel_callback(self, msg):
        self.scan_rot_vel = msg.data
        
    def joint_state_callback(self, msg):
        pv_ahr = msg.position[msg.name.index('la_r_1_to_hand_fr_1')]
        pv_ahl = msg.position[msg.name.index('la_l_1_to_h_fl_1')]
        pv_shr = msg.position[msg.name.index('base_link_to_h_rr_1')]
        pv_shl = msg.position[msg.name.index('base_link_to_h_rl_1')]
        commands = Float64MultiArray()
        
        feedback = [-pv_ahr, -pv_ahl, pv_shr, pv_shl]
        steering_vel = []
        for fb in feedback:
            steering_vel.append(self.check(self.psi, fb))
            
        commands.data = [float(-self.omg_fr), float(self.omg_fl), float(-self.omg_mr), float(self.omg_ml), float(-self.omg_fr), float(self.omg_fl), float(-steering_vel[0]), float(-steering_vel[1]), float(steering_vel[2]), float(steering_vel[3]), float(self.scan_rot_vel)]
        self.publisher.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    joint_velocity_publisher = JointVelocityPublisher(0.580, 0.680, 0.09)
    rclpy.spin(joint_velocity_publisher)
    joint_velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
