import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from turtlesim.srv import SetPen


class TamuTurtle(Node):

    def __init__(self):
        super().__init__('tamu_turtle')
        self.cli = self.create_client(SetParameters, '/turtlesim/set_parameters')
        self.cli2 = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0) and not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service')
        self.req = SetParameters.Request()
        self.req2 = SetPen.Request()

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


def main(args=None):
    rclpy.init(args=args)

    tamu_turtle = TamuTurtle()
    response = tamu_turtle.set_param('background_b', 0)
    tamu_turtle.set_param('background_g', 0)
    tamu_turtle.set_param('background_r', 80)
    tamu_turtle.set_pen(1)
    tamu_turtle.set_pen(0)

    tamu_turtle.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
