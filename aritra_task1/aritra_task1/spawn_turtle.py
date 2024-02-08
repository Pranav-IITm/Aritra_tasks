import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import time
import math
from std_msgs.msg import String
from turtlesim.msg import Pose
class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__('spawn_turtle')
        self.points_to_avoid= None
        self.publisher_4=self.create_publisher(Pose,'turtle_location',10)
        self.subscriber_4=self.create_subscription(String,'reached_target',self.spawn_turtle,10)


        

    def spawn_turtle(self,msg):
        # Create a client for the Spawn service
        self.spawn_client = self.create_client(Spawn, 'spawn')

        # Wait for the service to become available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        # Prepare the request message
        request = Spawn.Request()

        # Generate random positions for x and y
        request.x = random.uniform(0, 11)  # X position between 0 and 11
        request.y = random.uniform(0, 11)  # Y position between 0 and 11
        request.theta = random.uniform(0, 6.283)  # Random orientation angle between 0 and 2*pi
        request.name = 'turtle_' + str(int(time.time()))  # Unique name for the turtle
        
        self.points_to_avoid=Pose()
        self.points_to_avoid.x=request.x
        self.points_to_avoid.y=request.y
        self.points_to_avoid.theta=request.theta
        self.publisher_4.publish(self.points_to_avoid)



        # Call the service
        future = self.spawn_client.call_async(request)

        # Wait for the service response
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Turtle spawned successfully')
                else:
                    self.get_logger().error('Failed to spawn turtle')
                break

def main(args=None):
    rclpy.init(args=args)
    spawn_turtle_node = SpawnTurtleNode()
    
    rclpy.spin(spawn_turtle_node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

