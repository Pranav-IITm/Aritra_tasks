import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import math
import random


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.publisher_2 = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber_2 = self.create_subscription(Pose, 'turtle1/pose', self.get_current_pose, 10)
        self.subscriber_3 = self.create_subscription(Pose, 'new_coordinates', self.get_new_goal, 10)
        self.publisher_3 = self.create_publisher(String, 'reached_target', 10)
        self.subsciber_4 = self.create_subscription(Pose, 'turtle_location', self.avoid_points, 10)
        self.reached = False
        self.angle=None
        self.current_pose = None
        
    def get_current_pose(self, msg):
            self.current_pose = Pose()
            self.angle= self.current_pose.theta
            self.current_pose = msg
            self.get_logger().info('Current Pose is ', self.current_pose.x, self.current_pose.y)

    def go_to_goal(self):
            msg = Twist()
            while not self.reached :
                x = self.new_goal.x - self.current_pose.x
                y = self.new_goal.y - self.current_pose.y
                angle_to_goal = math.atan2(y, x)
                if abs(angle_to_goal - self.current_pose.theta)!= 0.0:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.5
                else:
                     msg.linear.x = 0.1
                     msg.angular.z = 0.0
                     self.publisher_2.publish(msg)
            if self.current_pose.x == self.new_goal.x and self.current_pose.y == self.new_goal.y:
                self.reached = True
                self.publisher_3.publish('reached')
                self.get_logger().info('Reached Goal')
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_2.publish(msg)
                self.get_logger().info('Reached Goal')
                self.publisher_3.publish('reached')
                self.reached = True

        
            

    def get_new_goal(self, msg):
            self.new_goal = Pose()
            self.new_goal = msg
            self.get_logger().info('New Goal is x=%f, y=%f', self.new_goal.x, self.new_goal.y)
            self.reached = False
            self.points_to_avoid= None
            self.point_to_avoid=None
            self.avoid_points(self.point_to_avoid)

    def avoid_points(self, msg):
             self.point_to_avoid= msg
             self.points_to_avoid.append(self.point_to_avoid)
             for i in self.points_to_avoid:
                 if self.current_pose.theta==i and ((i.x-self.current_pose.x)**2+(i.y-self.current_pose.y)**2< 0.1) :
                    self.get_logger().info('Avoiding the point where there is a possibility of collision')
                    msg=Twist()
                    time.sleep(1)
                    while abs(self.current_pose.theta-i)==1.57: 
                        msg.linear.x=0.0
                        msg.angular.z=1.57 
                        self.publisher_2.publish(msg)
                    time.sleep(1)
                    while self.current_pose.theta!=i:
                        msg.linear.x=0.1
                        msg.angular.z=-1 
                        self.publisher_2.publish(msg)
                    time.sleep(1)
                    self.get_logger().info('Avoided the point of collision')
                    while self.angle !=i:
                        msg.linear.x=0.0
                        msg.angular.z=1
                        self.publisher_2.publish(msg)
                    time.sleep(1)
                    self.get_logger().info('Resuming the path to the goal')
             self.go_to_goal()
                         
def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)
    rclpy.shutdown()
if __name__ == '__main__':
    main()