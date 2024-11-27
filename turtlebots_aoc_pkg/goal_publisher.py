import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np


# publish the goal position to the nav2 
class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.publish_goal)
        
    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.5
        goal.pose.position.y = -0.5
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # Update the goal position to move forward at 0.1 m/s
        goal.pose.position.x += 0.02 * self.timer_period
        print(self.timer)

        self.publisher_.publish(goal)
        self.timer_period += 1
        self.get_logger().info('Publishing goal position: [%f, %f]' % (goal.pose.position.x, goal.pose.position.y))


def main(args=None):
    rclpy.init(args=args)
    goal_publisher_node = GoalPublisher()
    rclpy.spin(goal_publisher_node)
    goal_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
