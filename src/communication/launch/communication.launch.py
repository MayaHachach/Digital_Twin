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
    
    request_STOD_server = Node(
        package='communication',
        executable='request_STOD_server',
        name='request_STOD_server',
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
    
    #! run this node separately if you think you're gonna be requesting the STOD more than once from omniverse.
    ld.add_action(request_STOD_server)
    
    #! comment this if you're not using the husky.
    ld.add_action(husky_odom_alignment)

    return ld