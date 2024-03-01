import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ScanQosConverter(Node):
    def __init__(self):
        super().__init__('scan_qos_converter')
        qos_profile_pub = QoSProfile(depth=10)
        qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile_sub = QoSProfile(depth=10)
        qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.listener_callback, 
            qos_profile_sub)

        self.publisher = self.create_publisher(
            LaserScan, 
            '/scan_qos_converted', 
            qos_profile_pub)

    def listener_callback(self, msg):
        # そのまま新しいトピックにパブリッシュする
        self.publisher.publish(msg)
        self.get_logger().info('Forwarding scan data to /scan_qos_converted')

def main(args=None):
    rclpy.init(args=args)
    scan_qos_converter = ScanQosConverter()
    rclpy.spin(scan_qos_converter)
    scan_qos_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()