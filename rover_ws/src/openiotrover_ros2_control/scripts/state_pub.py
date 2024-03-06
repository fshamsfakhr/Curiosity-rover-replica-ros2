#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import EntityState
from std_msgs.msg import Header

class EntityStatePublisher(Node):
    def __init__(self):
        super().__init__('entity_state_publisher')
        self.client = self.create_client(GetEntityState, '/cartpole/get_entity_state')

        # Wait for the service to be available before calling it
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = GetEntityState.Request()
        self.request.name = "dummy_rplidar_base"
        self.request.reference_frame = "world"

        # Create publishers for the entity state
        self.pose_publisher = self.create_publisher(Pose, 'rover_pose', 10)
        self.twist_publisher = self.create_publisher(Twist, 'rover_twist', 10)

    def call_service_and_publish(self):
        self.future = self.client.call_async(self.request)
        while not self.future.done():
            rclpy.spin_once(self)

        try:
            response = self.future.result()
            self.publish_response(response)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_response(self, response):
        # Publish pose
        pose_msg = Pose()
        pose_msg.position = response.state.pose.position
        pose_msg.orientation = response.state.pose.orientation
        self.pose_publisher.publish(pose_msg)
        # self.get_logger().info('Published rover pose on the topic.')

        # Publish twist
        twist_msg = Twist()
        twist_msg.linear = response.state.twist.linear
        twist_msg.angular = response.state.twist.angular
        self.twist_publisher.publish(twist_msg)
        # self.get_logger().info('Published rover twist on the topic.')

def main(args=None):
    rclpy.init(args=args)
    entity_state_publisher = EntityStatePublisher()

    try:
        while rclpy.ok():
            entity_state_publisher.call_service_and_publish()
    except KeyboardInterrupt:
        pass

    entity_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
