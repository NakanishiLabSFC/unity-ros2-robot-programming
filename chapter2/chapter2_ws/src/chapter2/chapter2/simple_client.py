from chapter2_interfaces.srv import CalculateCylinderVolume
import sys
import rclpy
from rclpy.node import Node


class SimpleClient(Node):

    def __init__(self):
        super().__init__('simple_client')
        self.client = self.create_client(CalculateCylinderVolume, 'calculate_cylinder_volume')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available now')
        self.request = CalculateCylinderVolume.Request()

    def send_request(self):
        self.request.r = float(sys.argv[1])
        self.request.h = float(sys.argv[2])
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    simple_client = SimpleClient()
    response = simple_client.send_request()
    simple_client.get_logger().info('Result: V = %f' % (response.v))

    simple_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
