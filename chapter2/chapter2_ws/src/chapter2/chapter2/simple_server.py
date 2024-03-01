import rclpy
from rclpy.node import Node
from chapter2_interfaces.srv import CalculateCylinderVolume
import math


class SimpleServer(Node):

    def __init__(self):
        super().__init__('simple_server')
        self.srv = self.create_service(CalculateCylinderVolume, 'calculate_cylinder_volume', self.calculate_cylinder_volume_callback)

    def calculate_cylinder_volume_callback(self, request, response):
        response.v = math.pi * request.r**2 * request.h
        self.get_logger().info('Request\nr: %f h: %f' % (request.r, request.h))

        return response

def main(args=None):
    rclpy.init(args=args)

    simple_server = SimpleServer()

    rclpy.spin(simple_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
