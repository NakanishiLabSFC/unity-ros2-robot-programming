import rclpy
from rclpy.action import ActionClient
import rclpy.action
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class Nav2UnityPipeline(Node):
    def __init__(self):
        super().__init__('nav2_unity_pipeline')
        self.get_logger().info('Waiting for Goal Pose...')
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.server_status =  self.nav2_client.wait_for_server(10000)
        self.task_status = False
        self.feedback = None
        self.result_future = None
        self.send_goal_future = None
        self.goal_to_distance = 0.0
        self.send_goal_future = None
        self.goal_handle = None
        self.i = 0

        if(self.server_status):
            self.timer = self.create_timer(0.5, self.timer_update)
            self.subscription_goal = self.create_subscription(PoseStamped, 'goal_pose_unity', self.callback_pose, 10)
            self.subscription_goal
            self.subscription_cancel = self.create_subscription(Bool, 'cancel_status', self.callback_cancel, 10)
            self.subscription_cancel

    def update_feedback(self, msg):
        self.goal_to_distance = msg.feedback.distance_remaining
        if(self.i % 60 == 0):
            self.get_logger().info("Distance remaining... %f" % self.goal_to_distance)
        self.i += 1
        
    def send_goal(self, msg):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = msg.header
        goal_msg.pose.pose = msg.pose
        self.send_goal_future = self.nav2_client.send_goal_async(goal_msg, self.update_feedback)
        self.get_logger().info("Sending Goal")

    def timer_update(self):
        if(self.send_goal_future != None):
            self.task_status = self.send_goal_future.done()
            if(self.task_status):
                self.goal_handle = self.send_goal_future.result()
                self.result_future = self.goal_handle.get_result_async()

    def callback_pose(self, msg):
        self.get_logger().info("Received New Goal")
        self.send_goal(msg)

    def callback_cancel(self, msg):
        self.get_logger().info("Received Cancel")
        if(msg.data and self.task_status):
            self.goal_handle.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2UnityPipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
