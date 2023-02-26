#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MoveTurtle(Node):

    def __init__(self, goal_x, goal_y):
        super().__init__('move_turtle')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turtle_pose = Pose()
        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 1)
        self.goal_x = goal_x
        self.goal_y = goal_y

    def pose_callback(self, data):
        self.turtle_pose.x = round(data.x, 2)
        self.turtle_pose.y = round(data.y, 2)
        self.turtle_pose.theta = round(data.theta, 2)

    def timer_callback(self):
        distance_to_goal = round(((self.goal_x - self.turtle_pose.x) ** 2 + (self.goal_y - self.turtle_pose.y) ** 2) ** 0.5, 2)
        angle_to_goal = round(math.atan2(self.goal_y - self.turtle_pose.y, self.goal_x - self.turtle_pose.x), 2)
        max_linear_velocity = 0.5

        velocity = Twist()
        velocity.linear.x = min(max_linear_velocity, distance_to_goal)
        velocity.angular.z = 4 * (angle_to_goal - self.turtle_pose.theta)
        self.velocity_publisher.publish(velocity)

        if distance_to_goal < 0.1:
            self.velocity_publisher.publish(Twist())
            self.timer.destroy()
            self.pose_subscription.destroy()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle(5.544445, 5.544445)
    rclpy.spin(move_turtle)
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
