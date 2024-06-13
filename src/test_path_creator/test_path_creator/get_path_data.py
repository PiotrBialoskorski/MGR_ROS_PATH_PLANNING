import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from my_interfaces.msg import MapOccData
from my_interfaces.msg import EssentialData

class getPose(Node):
    def __init__(self):
        super().__init__('GetPose')
        self.flaggoal = 0
        self.flaginit = 0
        self.flagdata = 0
        self.publisher_ = self.create_publisher(EssentialData, '/EssData',10)
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.subscription_init = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_callback,
            10,
        )
        self.subscription_data = self.create_subscription(
            MapOccData,
            '/test',
            self.data_callback,
            10
        )
        self.subscription_data
        self.subscription_goal
        self.subscription_init

    def goal_callback(self, msg : PoseStamped):
        self.pose_goal = msg.pose.position
        self.get_logger().info(f'Goal position received: {self.pose_goal}')
        self.flaggoal = 1
        self.send()

    def initial_callback(self, msg : PoseWithCovarianceStamped):
        self.pose_initial = msg.pose.pose.position
        self.get_logger().info(f'Initial position received: {self.pose_initial}')
        self.flaginit = 1
        self.send()
    
    def data_callback(self, map_data : MapOccData):
        self.data = map_data
        self.get_logger().info(f'Resolution and occupancy grid received')
        self.flagdata = 1
        self.send()

    def send(self):
        self.msg = EssentialData()
        if self.flaggoal == 1:
            self.msg.goal = self.pose_goal
        if self.flaginit == 1:
            self.msg.initial = self.pose_initial
        if self.flagdata == 1:
            self.msg.data = self.data.data
            self.msg.resolution = self.data.resolution
        if ((self.flaggoal == 1) and (self.flaginit == 1) and (self.flagdata == 1)):
            self.publisher_.publish(self.msg)
            self.get_logger().info(f'publishing essential data')
            self.flaginit = 0
            self.flaggoal = 0
            self.flagdata = 0


def main(args=None):
    rclpy.init(args=args)

    get_goal_pose = getPose()

    rclpy.spin(get_goal_pose)
    get_goal_pose.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

