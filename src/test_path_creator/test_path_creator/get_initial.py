import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from my_interfaces.msg import EssentialData

class initPose(Node):

    def __init__(self):
        super().__init__('Get_Initial')
        self.publisher_ = self.create_publisher(EssentialData, '/EssData', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_callback,
            10
        )
        self.subscription

    def initial_callback(self, msg : PoseWithCovarianceStamped):
        self.pose = msg.pose.pose.position
        self.get_logger().info(f'Initial position received: {self.pose}')
        self.send_initial()
    
    def send_initial(self):
        self.get_logger().info(f'Sending initial position')
        self.initial_pose = EssentialData()
        self.initial_pose.initial = self.pose
        self.publisher_.publish(self.initial_pose)

    def ret_initial(self):
        return self.initial_pose.initial
    
def main(args=None):
    rclpy.init(args=args)

    initpose = initPose()

    rclpy.spin_once(initpose)
    print(initpose.ret_initial())
    initpose.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()