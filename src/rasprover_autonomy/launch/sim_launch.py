import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    launch_args = []
    launch_args.append(DeclareLaunchArgument(
        'world', default_value='empty',
        description='Gazebo world to use (default empty world)'))
    launch_args.append(DeclareLaunchArgument(
        'px4_sitl_udp_port', default_value='14540',
        description='PX4 SITL MAVLink UDP port for ROS (default 14540)'))

    # Include PX4 SITL launch (this will start PX4 SITL and Gazebo)
    px4_launch_path = os.path.join(os.getenv('PX4_DIR', '/px4'), 'ros2_ws/src/px4_ros_com/launch/sitl.launch.py')
    # Assuming px4_ros_com package and a SITL launch file is available.
    include_px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(px4_launch_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    # The above is an example. Alternatively, use an ExecuteProcess to start the PX4 SITL binary:
    # ExecuteProcess(cmd=['px4', f'udp://:14540'], cwd=os.getenv('PX4_DIR','.'), output='screen')

    # MAVROS node connecting to SITL
    mavros_params_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'mavros_px4.yaml')
    mavros_node = Node(
        package='mavros', executable='mavros_node', output='screen',
        parameters=[mavros_params_file,
                    {'connection_url': 'udp://:14540@127.0.0.1:14557',
                     'gcs_url': 'udp://@127.0.0.1:14550'}]
    )
    # Here we set fcu_url for SITL: it opens UDP on local port 14540 and connects to PX4 SITL on 14557 (PX4 SITL default).
    # Also opens GCS link on 127.0.0.1:14550 for QGC (if running on same machine).

    # Robot localization and pure pursuit nodes (same as rover_launch)
    navsat_params = os.path.join(os.path.dirname(__file__), '..', 'config', 'navsat_transform.yaml')
    ekf_params = os.path.join(os.path.dirname(__file__), '..', 'config', 'ekf_localization.yaml')
    pp_waypoints = os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')
    navsat_node = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
        output='screen', parameters=[navsat_params],
        remappings=[('imu/data', '/mavros/imu/data'),
                    ('gps/fix', '/mavros/global_position/raw/fix'),
                    ('odometry/filtered', '/odometry/filtered')]
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_localization_node', name='ekf_filter',
        output='screen', parameters=[ekf_params],
        remappings=[('odometry/gps', '/odometry/gps'),
                    ('odometry/filtered', '/odometry/filtered'),
                    ('/mavros/local_position/odom', '/mavros/local_position/odom')]
    )
    pp_node = Node(
        package='rasprover_autonomy', executable='pure_pursuit_node', name='pure_pursuit',
        output='screen', parameters=[pp_waypoints]
    )

    return LaunchDescription(launch_args + [include_px4, mavros_node, navsat_node, ekf_node, pp_node])
