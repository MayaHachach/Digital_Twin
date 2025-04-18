import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('communication')
    
    # Define the config file path
    config_file = os.path.join(pkg_share_dir, 'config', 'robots.yaml')

    communicator_node = Node(
        package='communication',
        executable='communicator_node',
        name='communicator_node',
        output='screen',
        parameters=[{
            'config_path': config_file
        }]
    )
    
    request_history_server = Node(
        package='communication',
        executable='request_history_server',
        name='request_history_server',
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    ld.add_action(communicator_node)
    
    ld.add_action(request_history_server)

    return ld