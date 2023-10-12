import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    param_config = os.path.join(get_package_share_directory(package_name), 'config', 'param.yaml')
    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config','ekf.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # Start robot localization using an Extended Kalman Filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_params_file])
    # tf2_ros= Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         output='screen',
    #         arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    #         ),
    # tf2_ros1 = Node(
    #     ## Configure the TF of the robot 
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    #     ),
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    depthimage_to_laserscan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth','/depth/image_rect_raw'),
                        ('depth_camera_info', '/depth/camera_info'),
                        ('scan', '/scan')],
            parameters=[param_config]
        )
    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        # robot_localization,
        # tf2_ros,
        # tf2_ros1,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # depthimage_to_laserscan
    ])
