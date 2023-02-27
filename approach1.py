import math
import time
import threading
import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x = 0.0
y = 0.0
theta = 0.0

class TamuTurtle(Node):

    def __init__(self):
        super().__init__('tamu_turtle')
        self.cli = self.create_client(SetParameters, '/turtlesim/set_parameters')
        self.cli2 = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0) and not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service')
        self.req = SetParameters.Request()
        self.req2 = SetPen.Request()
        
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.turtle_pose = Pose()
        
        
    def set_param(self, a, b):
        self.req.parameters = [Parameter(name = a, value = ParameterValue(integer_value = b, type=ParameterType.PARAMETER_INTEGER))]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def set_pen(self, off):
        self.req2.r = 255
        self.req2.g = 255
        self.req2.b = 255
        self.req2.off = off
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    # Feedback from post to get the current position and angle of Turtle
    def pose_callback(self, data):
        #print(data)
        self.turtle_pose.x = round(data.x, 2)
        self.turtle_pose.y = round(data.y, 2)
        self.turtle_pose.theta = round(data.theta, 2)
    
    def rotate_turtle(self, goal_x, goal_y):
        #Turn turle to next location using a feedback loop of current angle
        rotation = Twist()
        while (True):
            angle_to_goal = round(math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x), 2)
            rotation.angular.z = (angle_to_goal - self.turtle_pose.theta)
            self.velocity_publisher.publish(rotation)
            #print(rotation)
            
            if (angle_to_goal < 0.01):
                time.sleep(2)
                break;

        
    def move_turtle(self, goal_x, goal_y):
        
        MIN_LINEAR_VELOCITY = 0.5
        
        global theta
        self.rate = self.create_rate(10)
        # Move turtle to desired location using feedback loop, has angle element incase of earlier errors
        velocity = Twist()
        while (True):
            distance_to_goal = round(((goal_x - self.turtle_pose.x) ** 2 + (goal_y - self.turtle_pose.y) ** 2) ** 0.5, 2)
            angle_to_goal = round(math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x), 2)
            
            
            velocity.linear.x = MIN_LINEAR_VELOCITY * distance_to_goal
            velocity.linear.y = 0.0
            #print(angle_to_goal, "|", self.turtle_pose.theta, "|", theta)
            velocity.angular.z = 4.0 * (angle_to_goal - self.turtle_pose.theta)#angle_to_goal - self.turtle_pose.theta)
            self.velocity_publisher.publish(velocity)
            self.rate.sleep()
            
        
            if distance_to_goal < 0.01:
                break;
            

def main(args=None):
    rclpy.init(args=args)

    tamu_turtle = TamuTurtle()
    executor2 = rclpy.Executor.add_node(tamu_turtle)
    
    rclpy.Executor._execute_subscription
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tamu_turtle)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    response = tamu_turtle.set_param('background_b', 0)
    tamu_turtle.set_param('background_g', 0)
    tamu_turtle.set_param('background_r', 80)
    tamu_turtle.set_pen(1)
    
    tamu_turtle.rotate_turtle(1.0, 1.0)
    print(tamu_turtle.turtle_pose)
    tamu_turtle.move_turtle(1.0, 1.0)
    # tamu_turtle.rotate_turtle(4.0, 5.0)
    # tamu_turtle.rotate_turtle(1.0, 3.0)

    
    tamu_turtle.set_pen(0)

    tamu_turtle.destroy_node()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
