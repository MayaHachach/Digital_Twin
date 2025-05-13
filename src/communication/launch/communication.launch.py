import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('communication')
    
    # Define the config file path
    # domain_bridge_config = os.path.join(
    #     '/home/maya/workspaces/Digital_Twin/src/communication/config',
    #     'domain_bridge_config.yaml'
    # )

    communicator_node = Node(
        package='communication',
        executable='communicator_node',
        name='communicator_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share_dir, 'config', 'robots.yaml')
        }]
    )
    
    request_history_server = Node(
        package='communication',
        executable='request_history_server',
        name='request_history_server',
        output='screen'
    )

    # domain_bridge_node = Node(
    #     package='domain_bridge',
    #     executable='domain_bridge',
    #     name='domain_bridge',
    #     output='screen',
    #     arguments=[domain_bridge_config]
    # )

    # Create and return launch description
    ld = LaunchDescription()

    ld.add_action(communicator_node)
    ld.add_action(request_history_server)
    # ld.add_action(domain_bridge_node)

    return ld
