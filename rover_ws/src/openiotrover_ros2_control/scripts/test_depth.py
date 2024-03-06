import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoCameraSubscriber:
    def __init__(self):
        rclpy.init()

        self.node = rclpy.create_node('stereo_camera_subscriber')

        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the stereo camera depth image raw topic
        self.depth_image_sub = self.node.create_subscription(
            Image,
            '/cartopole/kinect_camera/depth/image_raw',
            self.image_callback,
            10  # Queue size
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            print(f"Error converting image: {e}")
            return

        # Normalize depth values for visualization
        normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

        # Convert to 8-bit unsigned integer
        depth_display = np.uint8(normalized_depth)

        # Display the depth image
        cv2.imshow('Stereo Camera - Depth Image', depth_display)
        cv2.waitKey(1)  # You may need to adjust this depending on your system

def main():
    stereo_camera_subscriber = StereoCameraSubscriber()

    try:
        rclpy.spin(stereo_camera_subscriber.node)
    except KeyboardInterrupt:
        pass
    finally:
        stereo_camera_subscriber.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
