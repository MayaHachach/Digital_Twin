import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    communicator_node = Node(
        package='communication',
        executable='communicator',
        name='communicator',
        output='screen'
    )
    
    request_history_server = Node(
        package='communication',
        executable='request_history_server',
        name='request_history_server',
        output='screen'
    )
    
    husky_odom_alignment = Node(
        package='communication',
        executable='odom_alignment',
        name='odom_alignment',
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    ld.add_action(communicator_node)
    
    ld.add_action(request_history_server)
    
    #! comment this if you're not using the husky.
    ld.add_action(husky_odom_alignment)

    return ld