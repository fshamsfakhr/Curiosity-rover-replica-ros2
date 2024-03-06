import rclpy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class StereoCameraSubscriber:
    def __init__(self):
        rclpy.init()

        self.node = rclpy.create_node('stereo_camera_subscriber')

        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the stereo camera left image raw topic
        self.left_image_sub = self.node.create_subscription(
            Image,
            '/cartopole/kinect_camera/image_raw',
            self.image_callback,
            10  # Queue size
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image
        cv2.imshow('Stereo Camera - Left Image', cv_image)
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
