import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

class DynamicTracking(Node):

    def __init__(self):
        super().__init__('dynamic_tracking_node')
        self.robot_names = ["tb1", "tb2"]
        self.lead_robot = self.robot_names[0]  # tb1
        self.follower_robot = self.robot_names[1]  # tb2
        # self.lead_robot = "tb1"
        # self.follower_robot = ["tb2", "tb3", "tb4"]


        # Subscriber for lead robot (tb1) odometry
        self.lead_subscription = self.create_subscription(
            Odometry,
            f'/{self.lead_robot}/odom',
            self.lead_callback,
            10
        )

        # Publisher for follower robot (tb2) goal position
        self.follower_publisher = self.create_publisher(
            PoseStamped,
            f'/{self.follower_robot}/goal_pose',
            10
        )

        # Variable to store the lead robot's position and orientation
        self.lead_position = None
        self.lead_orientation = None
        # Variable to store the follower robot's position
        self.follower_position = None

        # Subscriber for follower robot (tb2) odometry
        self.follower_subscription = self.create_subscription(
            Odometry,
            f'/{self.follower_robot}/odom',
            self.follower_callback,
            10
        )

    def lead_callback(self, msg: Odometry):
        """Callback to get the lead robot's position and orientation."""
        self.lead_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        # Extract yaw from orientation quaternion
        self.lead_orientation = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.update_goal()

    def follower_callback(self, msg: Odometry):
        """Callback to get the follower robot's position."""
        self.follower_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.update_goal()

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (in radians)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_goal(self):
        """Update the goal position for the follower robot."""
        if self.lead_position is None or self.lead_orientation is None:
            # Wait for lead robot's position and orientation to be available
            return

        # Calculate the target position 0.5 meters behind the lead robot
        offset_distance = 0.5
        goal_x = self.lead_position[0] - offset_distance * math.cos(self.lead_orientation)
        goal_y = self.lead_position[1] - offset_distance * math.sin(self.lead_orientation)

        # Publish the new goal for the follower robot
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0  # No rotation
        self.follower_publisher.publish(goal_msg)

        self.get_logger().info(f'Published new goal for {self.follower_robot}: '
                               f'({goal_msg.pose.position.x}, {goal_msg.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    dynamic_tracking_node = DynamicTracking()
    rclpy.spin(dynamic_tracking_node)
    dynamic_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
