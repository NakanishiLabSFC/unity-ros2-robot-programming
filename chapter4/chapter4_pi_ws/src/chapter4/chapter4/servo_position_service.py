import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import ServoPositionService
import pigpio

SERVO_PIN = 18
SERVO_MIN = 500
SERVO_MAX = 2500

pi = pigpio.pi()

class PositionServiceNode(Node):

    def __init__(self):
        super().__init__('servo_position_service_node')
        self.srv = self.create_service(ServoPositionService, 'servo_position_srv', self.servo_position_callback)

    def servo_position_callback(self, request, response):
        print(request)
        target_position = request.position
        p_width = int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * target_position / 180.0)
        try:
            pi.set_servo_pulsewidth(SERVO_PIN, p_width)
            response.result = "success"
        except:
            response.result = "fail"
        return response


def main(args=None):
    rclpy.init(args=args)
    servo_position_service = PositionServiceNode()
    rclpy.spin(servo_position_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
