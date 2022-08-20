import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('bfc_motion_bridge'),
        'config',
        'head-movement.yaml'
        )
        
    node=Node(
        package = 'bfc_motion_bridge',
        name = 'head_movement',
        executable = 'head_movement',
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld