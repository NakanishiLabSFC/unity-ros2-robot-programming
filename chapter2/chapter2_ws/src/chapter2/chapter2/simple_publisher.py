import rclpy
from rclpy.node import Node
from datetime import datetime as dt
from std_msgs.msg import String


class SimplePublisher(Node):
    
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'simple_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        dt_now = dt.now()
        dt_str = dt_now.strftime('%H:%M:%S')
        msg.data = 'Published: ' + dt_str
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
