import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 
from std_msgs.msg import String
import math
import numpy as np

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.publisher_2 = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber_1 = self.create_subscription(Pose, 'turtle_location', self.avoid_points, 10)
        self.subscriber_2 = self.create_subscription(Pose, 'turtle1/pose', self.get_current_pose, 10)
        self.subscriber_3 = self.create_subscription(Pose, 'new_coordinates', self.get_new_goal, 10)
        self.publisher_3 = self.create_publisher(String, 'reached_target', 10)
        self.current_pose = None
        self.goal = None
        self.grid = None
        self.grid_size = 12  # Size of the grid
        self.obstacle_value = 1  # Value representing obstacles in the grid
        self.point_to_avoid = None
        self.points_to_avoid = []
        self.score=0

    def get_current_pose(self, msg):
        self.current_pose = msg

    def get_new_goal(self, msg):
        self.goal = msg
        self.generate_grid()
        self.astar()

    def generate_grid(self):
        self.grid = np.zeros((self.grid_size, self.grid_size))
        self.avoid_points(self.points_to_avoid)
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for point in self.points_to_avoid:
                    if (int(point.x)<=i and int(point.x)>i-1 and int(point.y)<=j and int(point.y)>j-1):
                        self.grid[i][j] = self.obstacle_value
        # Mark obstacles in the grid based on TurtleSim's obstacles
        # Example: if a cell has an obstacle, set the corresponding grid cell to the obstacle value
        # You can modify this based on the actual TurtleSim environment
        # For simplicity, let's assume there are no obstacles initially
        # If there were obstacles, you'd need to update the grid accordingly


    def astar(self):
        speed=Twist
        start_point=self.current_pose
        end_point=self.goal
        m=math.floor(start_point.x)
        n=math.floor(start_point.y)
        distance_1=(m-start_point.x)**2+(n-start_point.y)**2
        distance_2=(m+1-start_point.x)**2+(n-start_point.y)**2
        distance_3=(m-start_point.x)**2+(n+1-start_point.y)**2
        distance_4=(m+1-start_point.x)**2+(n+1-start_point.y)**2
        if self.grid[m][n]==self.obstacle_value:
             distance_1=1000
        if self.grid[m+1][n]==self.obstacle_value:
             distance_2=1000
        if self.grid[m][n+1]==self.obstacle_value:
                distance_3=1000
        if self.grid[m+1][n+1]==self.obstacle_value:
                distance_4=1000
        a=min(distance_1,distance_2,distance_3,distance_4)
        if a==distance_1:
            angle_to_node=math.atan2(n-start_point.y,m-start_point.x)
            if self.current_pose.theta!=angle_to_node:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=m and self.current_pose.y!=n:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if a==distance_2:
            angle_to_node=math.atan2(n-start_point.y,m+1-start_point.x)
            if self.current_pose.theta!=angle_to_node:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=m+1 and self.current_pose.y!=n:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if a==distance_3:
            angle_to_node=math.atan2(n+1-start_point.y,m-start_point.x)
            if self.current_pose.theta!=angle_to_node:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=m and self.current_pose.y!=n+1:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if a==distance_4:
            angle_to_node=math.atan2(n+1-start_point.y,m+1-start_point.x)
            if self.current_pose.theta!=angle_to_node:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=m+1 and self.current_pose.y!=n+1:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        
        if self.current_pose.theta!=0.0:
            speed.angular.z=1.0
            self.publisher_2.publish(speed)

        
        o=math.floor(end_point.x)
        p=math.floor(end_point.y)
        heuristic_estimate=np.zeros(12,12)
        g_score=np.zeros(12,12)
        f_score=np.zeros(12,12)
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                heuristic_estimate[i][j]=abs(i-o)+abs(j-p)
                g_score[i][j]=abs(i-m)+abs(j-n)
                f_score[i][j]=g_score[i][j]+heuristic_estimate[i][j]
        i=m
        j=n      
        while (i!=o and i!=1)and (j!=p and j!=1):
            while self.grid[i-1][j]==self.obstacle_value:
                f_score[i-1][j]=10000
            while self.grid[i+1][j]==self.obstacle_value:
                f_score[i+1][j]=10000
            while self.grid[i][j-1]==self.obstacle_value:
                f_score[i][j-1]=10000
            while self.grid[i][j+1]==self.obstacle_value:
                f_score[i][j+1]=10000
            self.c=min(f_score[i-1][j],f_score[i+1][j],f_score[i][j-1],f_score[i][j+1])
            if self.c==f_score[i-1][j]:
                while self.current_pose.theta!=-1.57:
                    speed.angular.z=1.0
                while i!=i-1:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                i=i-1
            if self.c==f_score[i+1][j]:
                while self.current_pose.theta!=1.57:
                    speed.angular.z=1.0
                while i!=i+1:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                i=i+1
            if self.c==f_score[i][j-1]:
                while self.current_pose.theta!=3.14:
                    speed.angular.z=-1.0
                while j!=j-1:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                j=j-1
            if self.c==f_score[i][j+1]:
                while self.current_pose.theta!=0.0:
                    speed.angular.z=1.0
                while j!=j+1:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                j=j+1
        while i==1 or j==1:
            while self.grid[2][j]==self.obstacle_value:
                f_score[2][j]=10000
            while self.grid[i][2]==self.obstacle_value:
                f_score[i][2]=10000
            self.d=min(f_score[2][j],f_score[i][2])
            if self.d==f_score[2][j]:
                while self.current_pose.theta!=1.57:
                    speed.angular.z=1.0
                while i!=2:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                i=2
            if self.d==f_score[i][2]:
                while self.current_pose.theta!=0.0:
                    speed.angular.z=1.0
                while j!=2:
                    speed.linear.x=1.0
                    self.publisher_2.publish(speed)
                j=2
        distance_5= (end_point.x-o)**2+(end_point.y-p)**2
        distance_6= (end_point.x-o+1)**2+(end_point.y-p)**2
        distance_7= (end_point.x-o)**2+(end_point.y-p+1)**2
        distance_8= (end_point.x-o+1)**2+(end_point.y-p+1)**2
        if self.grid[o][p]==self.obstacle_value:
             distance_5=1000
        if self.grid[o+1][p]==self.obstacle_value:
                distance_6=1000
        if self.grid[o][p+1]==self.obstacle_value:
                distance_7=1000
        if self.grid[o+1][p+1]==self.obstacle_value:
                distance_8=1000
        b=min(distance_5,distance_6,distance_7,distance_8)
        if b==distance_5:
            angle_to_finalnode=math.atan2(end_point.y-p,end_point.x-o)
            if self.current_pose.theta!=angle_to_finalnode:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=o and self.current_pose.y!=p:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if b==distance_6:
            angle_to_finalnode=math.atan2(end_point.y-p,end_point.x-o+1)
            if self.current_pose.theta!=angle_to_finalnode:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=o+1 and self.current_pose.y!=p:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if b==distance_7:
            angle_to_finalnode=math.atan2(end_point.y-p+1,end_point.x-o)
            if self.current_pose.theta!=angle_to_finalnode:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=o and self.current_pose.y!=p+1:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        if b==distance_8:
            angle_to_finalnode=math.atan2(end_point.y-p+1,end_point.x-o+1)
            if self.current_pose.theta!=angle_to_finalnode:
                speed.angular.z=1.0
                self.publisher_2.publish(speed)
            if self.current_pose.x!=o+1 and self.current_pose.y!=p+1:
                speed.linear.x=1.0
                self.publisher_2.publish(speed)
        self.publisher_3.publish(String(data='reached'))
        self.score=self.score+1       
        # Implement A* algorithm here to find the optimal path from current_pose to the goal
        # This involves defining a heuristic function, calculating costs, expanding nodes, etc.
        # Return the path as a list of positions [(x1, y1), (x2, y2), ..., (xn, yn)] if found
        # Return None if no path is found
        self.get_logger().info("Current Score in this game is")
        self.get_logger().info(self.score)
    def avoid_points(self, msg:Pose):
        self.point_to_avoid= msg
        self.points_to_avoid.append(self.point_to_avoid)
        # Implement obstacle avoidance logic here using received coordinates of spawned turtles
        


def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
