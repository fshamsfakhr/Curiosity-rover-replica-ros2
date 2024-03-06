import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/cartopole/scan',
            self.scan_callback,
            10
        )
        self.cartesian_points = []
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        self.cartesian_points = self.calculate_cartesian_points(angle_min, angle_increment, ranges)
        self.plot_cartesian_points()

    def calculate_cartesian_points(self, angle_min, angle_increment, ranges):
        cartesian_points = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if not math.isinf(r):  # ignoring infinite values
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                cartesian_points.append((x, y))
        return cartesian_points

    def plot_cartesian_points(self):
        plt.clf()  # Clear the previous plot
        if self.cartesian_points:
            x, y = zip(*self.cartesian_points)
            plt.scatter(x, y)
            plt.xlabel('X-axis')
            plt.ylabel('Y-axis')
            plt.title('Cartesian Points from /cartopole/scan')
            plt.pause(0.01)  # Briefly pause to update the plot

    def timer_callback(self):
        self.plot_cartesian_points()

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber = ScanSubscriber()
    rclpy.spin(scan_subscriber)
    scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

