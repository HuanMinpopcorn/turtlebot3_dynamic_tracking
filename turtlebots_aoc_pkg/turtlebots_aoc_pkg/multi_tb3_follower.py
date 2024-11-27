import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

class DynamicTracking(Node):

    def __init__(self):
        super().__init__('dynamic_tracking_node')
        self.robots = ["tb1", "tb2", "tb3", "tb4"]
        self.offset_distance = 0.5  # Distance to maintain behind the preceding robot

        # Store positions and orientations of all robots
        self.positions = {robot: None for robot in self.robots}
        self.orientations = {robot: None for robot in self.robots}

        # Create subscribers for all robots' odometry
        self.subscriber_list = []  # Changed from self.subscriptions
        for robot in self.robots:
            sub = self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
            self.subscriber_list.append(sub)

        # Create publishers for all followers' goal positions
        self.follower_publishers = {  # Changed from self.publishers
            robot: self.create_publisher(PoseStamped, f'/{robot}/goal_pose', 10)
            for robot in self.robots[1:]  # Exclude the lead robot (tb1)
        }

    def odom_callback(self, msg: Odometry, robot: str):
        """Callback to update the position and orientation of a robot."""
        self.positions[robot] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.orientations[robot] = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.update_goals()

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (in radians)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_goals(self):
        """Update goal positions for all follower robots."""
        for i in range(1, len(self.robots)):  # Iterate over followers
            leader = self.robots[i - 1]
            follower = self.robots[i]

            # Ensure leader's position and orientation are available
            if self.positions[leader] is None or self.orientations[leader] is None:
                continue

            # Calculate the target position for the follower
            goal_x = self.positions[leader][0] - self.offset_distance * math.cos(self.orientations[leader])
            goal_y = self.positions[leader][1] - self.offset_distance * math.sin(self.orientations[leader])

            # Publish the new goal for the follower
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.orientation.w = 1.0  # No rotation

            self.follower_publishers[follower].publish(goal_msg)
            self.get_logger().info(f'Published new goal for {follower}: ({goal_x}, {goal_y})')

def main(args=None):
    rclpy.init(args=args)
    dynamic_tracking_node = DynamicTracking()
    rclpy.spin(dynamic_tracking_node)
    dynamic_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
