import pygame
import sys
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def round_to_precision(number, precision):
    return round(number / precision) * precision

rclpy.init()
node = rclpy.create_node('velocity_subscriber')
velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 1)
velocity_msg = Twist()
# Initialize Pygame
pygame.init()

# Screen dimensions
screen_width, screen_height = 800, 600
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

# Velocity parameters
max_velocity = 1.5
acceleration_rate_v = 0.01
acceleration_rate_w = 0.05
V = 0
W = 0  # Additional V variable

# Main game loop
running = True
key_up_pressed = False
key_down_pressed = False
key_left_pressed = False
key_right_pressed = False
shift_pressed = False

while running:
    screen.fill((255, 255, 255))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                key_up_pressed = True
            elif event.key == pygame.K_DOWN:
                key_down_pressed = True
            elif event.key == pygame.K_LEFT:
                key_left_pressed = True
            elif event.key == pygame.K_RIGHT:
                key_right_pressed = True
            elif event.key == pygame.K_RSHIFT:  # Check for left shift key
                if not shift_pressed:
                    shift_pressed = True
                else:
                    shift_pressed = False    
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                key_up_pressed = False
                if not shift_pressed:
                    V = 0
            elif event.key == pygame.K_DOWN:
                key_down_pressed = False
                if not shift_pressed:
                    V = 0
            elif event.key == pygame.K_LEFT:
                key_left_pressed = False
                # if not shift_pressed:
                W = 0
            elif event.key == pygame.K_RIGHT:
                key_right_pressed = False
                # if not shift_pressed:
                W = 0
    print(shift_pressed)
    if key_up_pressed:
        if V < max_velocity:
            V += acceleration_rate_v
        else:
            V = max_velocity
        if((W>=round_to_precision(V, .1)**2*0.5+0.25) or (W>=max_velocity)):
            W -= acceleration_rate_w 
        if((-W>=round_to_precision(V, .1)**2*0.5+0.25) or (-W>=max_velocity)): 
            W += acceleration_rate_w        
    if key_down_pressed:
        if V > -max_velocity:
            V -= acceleration_rate_v
        else:
            V = -max_velocity
        if((W>=round_to_precision(V, .1)**2*0.5+0.25) or (W>=max_velocity)):
            W -= acceleration_rate_w 
        if((-W>=round_to_precision(V, .1)**2*0.5+0.25) or (-W>=max_velocity)): 
            W += acceleration_rate_w  
            
    if key_left_pressed:
        if((W<round_to_precision(V, .1)**2*0.5+0.25) and (W<max_velocity)): 
            W += acceleration_rate_w
            
    if key_right_pressed:
        if((-W<round_to_precision(V, .1)**2*0.5+0.25) and (-W<max_velocity)): 
            W -= acceleration_rate_w


    # Update position based on velocities (for demonstration, just print velocities)
    print(f"V: {V} - W: {W}")
    velocity_msg.linear.x = float(V)  # Linear velocity (m/s)
    velocity_msg.angular.z = float(W) # Angular velocity (rad/s)
    velocity_publisher.publish(velocity_msg)
    pygame.display.flip()
    clock.tick(60)
    
rclpy.spin_once(node)
node.destroy_node()
rclpy.shutdown()
pygame.quit()
sys.exit()
