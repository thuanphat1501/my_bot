import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name='my_bot'
    param_config = os.path.join(get_package_share_directory(package_name), 'config', 'param.yaml')

    return LaunchDescription([

        
        Node(
             package='depthimage_to_laserscan',
             executable='depthimage_to_laserscan_node',
             name='depthimage_to_laserscan',
             remappings=[('depth', '/camera/depth/image_rect_raw'),
                         ('depth_camera_info', '/camera/depth/camera_info'),
                         ('scan', '/scan')],
             parameters=[param_config])
    ])
