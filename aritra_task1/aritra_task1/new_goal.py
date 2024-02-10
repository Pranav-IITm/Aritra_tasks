import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import math
import random


class NEW_GOAL(Node):
    def __init__(self):
        super().__init__('new_goal')
        self.publisher_1=self.create_publisher(Pose,'new_coordinates',10)
        self.subscriber_1=self.create_subscription(String,'reached_target',self.next_goal,10)
        
    def next_goal(self,msg):
         self.get_logger().info('Current Goal (%f, %f) reached successfully', msg.x, msg.y)
         data=Pose()
         data.x=random.uniform(2,10)
         data.y=random.uniform(2,10)
         self.get_logger().info('Next Goal is ',data.x,data.y)
         self.publisher_1.publish(data)

def main(args=None):
    rclpy.init(args=args)
    new_goal= NEW_GOAL()
    rclpy.spin(new_goal)

if __name__ == '__main__':
    main()
