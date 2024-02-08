from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     spawn_turtle = Node(
            package='aritra_task1',
            namespace='spawn_turtle',
            executable='spawn_turtle',
            name='spawn_turtle'
         )
     go_to_goal=Node(
            package='aritra_task1',
            namespace='go_to_goal',
            executable='go_to_goal',
            name='go_to_goal'
        )
     new_goal=Node(
            package='aritra_task1',
            namespace='new_goal',
            executable='new_goal',
            name='new_goal'
        )
     
     return LaunchDescription([spawn_turtle,go_to_goal,new_goal])
    