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
        self.publisher_4 = self.create_publisher(Pose, 'turtle_location', 10)
        self.subscriber_4 = self.create_subscription(String, 'reached_target', self.spawn_turtle, 10)
        self.spawn_client = self.create_client(Spawn, 'spawn')

    def spawn_turtle(self, msg):
        # Generate random positions for x and y
        request = Spawn.Request()
        request.x = random.uniform(2, 10)  # X position between 2 and 10
        request.y = random.uniform(2, 10)  # Y position between 2 and 10
        request.theta = random.uniform(0, 2 * math.pi)  # Random orientation angle between 0 and 2*pi
        request.name = 'turtle_' + str(int(time.time()))  # Unique name for the turtle

        # Publish the spawned turtle location
        pose_msg = Pose()
        pose_msg.x = request.x
        pose_msg.y = request.y
        pose_msg.theta = request.theta
        self.publisher_4.publish(pose_msg)

        # Call the service
        future = self.spawn_client.call_async(request)

        # Wait for the service response
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                    if response is not None:
                        self.get_logger().info('Turtle spawned successfully')
                    else:
                        self.get_logger().error('Failed to spawn turtle')
                    break
                except Exception as e:
                    self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    spawn_turtle_node = SpawnTurtleNode()
    rclpy.spin(spawn_turtle_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
