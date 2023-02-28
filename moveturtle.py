#!/usr/bin/env python
import time
import math
from queue import Queue

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):

    def __init__(self, goal_x, goal_y, coordinates):
        super().__init__('move_turtle')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turtle_pose = Pose()
        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 1)
        self.goal_x = goal_x
        self.goal_y = goal_y
        
        self.cli = self.create_client(SetParameters, '/turtlesim/set_parameters')
        self.cli2 = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0) and not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service')
        self.req = SetParameters.Request()
        self.req2 = SetPen.Request()
        
        # find the first set of coordinates
        self.cord_queue = coordinates
        self.calculate_coordinates(self.cord_queue)
        
        
    def pose_callback(self, data):
        self.turtle_pose.x = round(data.x, 2)
        self.turtle_pose.y = round(data.y, 2)
        self.turtle_pose.theta = round(data.theta, 2)
        
    def set_param(self, a, b):
        self.req.parameters = [Parameter(name = a, value = ParameterValue(integer_value = b, type=ParameterType.PARAMETER_INTEGER))]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    # coordinates passed in as a string in the format "x y" relative to tamu logo image
    # need to convert coordianated in original image to the current one and set it as goal
    def calculate_coordinates(self, coordinates):
        ORIGINAL_IMAGE_SIZE = 540.0
        CURRNT_IMAGE_SIZE = 11.0
        
        DRAW_LINE_ON = 0
        DRAW_LINE_OFF = 1
        print("a")
        cord = coordinates.get()
        original_x = cord.split()[0]
        original_y = cord.split()[1]
        print("b")
        # if the new coordiantes are "-1 -1" means need to go to next cord in list with no path drawing
        # else have it on
        if (original_x == '-1') or (original_y == '-1'):
            #self.set_pen(DRAW_LINE_OFF)
            
            # grab next values
            cord = coordinates.get()
            original_x = cord.split()[0]
            original_y = cord.split()[1]
            
        #else:
            #self.set_pen(DRAW_LINE_ON)
            
        print("c")
        print("oiginal coordinates: ", original_x, original_y)
        new_x = ((CURRNT_IMAGE_SIZE/ORIGINAL_IMAGE_SIZE) * float(original_x))
        new_y = 11- ((CURRNT_IMAGE_SIZE/ORIGINAL_IMAGE_SIZE) * float(original_y))
        
        
        print("grabbed new coordinates: ", new_x, new_y)
        self.goal_x = new_x
        self.goal_y = new_y
        
        
    
    def set_pen(self, off):
        self.req2.r = 255
        self.req2.g = 255
        self.req2.b = 255
        self.req2.off = off
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    # Function to continuosly calculate the distance to the goal as well as the angle the turtle must face to reach the goal. 

    def timer_callback(self):
        distance_to_goal = round(((self.goal_x - self.turtle_pose.x) ** 2 + (self.goal_y - self.turtle_pose.y) ** 2) ** 0.5, 2)
        angle_to_goal = round(math.atan2(self.goal_y - self.turtle_pose.y, self.goal_x - self.turtle_pose.x), 2)
        max_linear_velocity = 0.5
        
        velocity = Twist()
        
        # Make sure the turtle is properly aligned before moving forward

        if abs(angle_to_goal - self.turtle_pose.theta) > 0.1:
            velocity.linear.x = 0.0
        else:
            velocity.linear.x = min(max_linear_velocity, distance_to_goal)

        velocity.angular.z = 4 * (angle_to_goal - self.turtle_pose.theta)
        self.velocity_publisher.publish(velocity)

        # # rotate before driving forwards
        # if((angle_to_goal - self.turtle_pose.theta) > 0.1):
        #     velocity.angular.z = 1 * (angle_to_goal - self.turtle_pose.theta)
        #     velocity.linear.x = 0.0
        #     self.velocity_publisher.publish(velocity)
        
        # # move forwards to goal
        # else:
        #     velocity.linear.x = (1.0 * distance_to_goal)
        #     velocity.angular.z = 0.5 * (angle_to_goal - self.turtle_pose.theta)
        #     self.velocity_publisher.publish(velocity)

        print(distance_to_goal, angle_to_goal, self.turtle_pose.theta)
        
        # set new goal position once arrived
        if distance_to_goal < 0.05:
            # time.sleep(2)
            print("here")
            self.calculate_coordinates(self.cord_queue)
            print("stuck")
            # self.velocity_publisher.publish(Twist())
            # self.timer.destroy()
            # self.pose_subscription.destroy()
            # self.destroy_node()
            # rclpy.shutdown()

def main(args=None):
    cord_queue = Queue()
    file = open('coordinates.txt')
    lines = file.readlines()
    for line in lines:
        cords = line.splitlines()[0]
        cord_queue.put(cords)
    
    rclpy.init(args=args)
    move_turtle = MoveTurtle(5.5, 5.5, cord_queue)
    rclpy.spin(move_turtle)
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
