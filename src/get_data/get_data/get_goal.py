import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from my_interfaces.msg import EssentialData

class getGoal(Node):
    def __init__(self):
        super().__init__('Get_Goal')
        self.publisher_ = self.create_publisher(EssentialData, '/EssData',10)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.get_callback,
            10
        )
        self.subscription

    def get_callback(self, msg : PoseStamped):
        self.pose = msg.pose.position
        self.get_logger().info(f'Goal position received: {self.pose}')
        self.send_goal()

    def send_goal(self):
        self.get_logger().info(f'Sending goal position')
        self.goal_pose = EssentialData()
        self.goal_pose.goal = self.pose
        # self.goal_pose.goal.y = self.pose.y
        # self.goal_pose.goal.z = self.pose.z
        self.publisher_.publish(self.goal_pose)



def main(args=None):
    rclpy.init(args=args)

    get_goal_pose = getGoal()

    rclpy.spin(get_goal_pose)
    get_goal_pose.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

