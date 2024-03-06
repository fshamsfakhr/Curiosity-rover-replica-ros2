import rclpy
from std_msgs.msg import Float64
from std_msgs.msg import Empty

def main():
    rclpy.init()
    node = rclpy.create_node('scan_velocity_subscriber')
    velocity_publisher = node.create_publisher(Float64, 'scan_vel', 1)
    velocity_msg = Float64()

    while rclpy.ok():
        velocity_msg.data = 1.1  # Linear velocity (m/s)
        velocity_publisher.publish(velocity_msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
