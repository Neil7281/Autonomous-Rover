import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments for device names/addresses
    launch_args = []
    launch_args.append(DeclareLaunchArgument(
        'fcu_url', default_value='serial:///dev/ttyACM0:57600',
        description='Connection URL for Pixhawk (e.g. serial:///dev/ttyACM0:57600)'
    ))
    launch_args.append(DeclareLaunchArgument(
        'gcs_url', default_value='udp://@0.0.0.0:14550',
        description='GCS UDP URL for QGroundControl telemetry'
    ))

    # MAVROS node
    mavros_params_file = os.path.join(get_package_share_directory('rasprover_autonomy'), 'config', 'mavros_px4.yaml')
    mavros_node = Node(
        package='mavros', executable='mavros_node', output='screen',
        namespace='',
        parameters=[mavros_params_file,
                    {'connection_url': LaunchConfiguration('fcu_url'),
                     'gcs_url': LaunchConfiguration('gcs_url')}]
    )
    # The above loads the YAML and overrides fcu_url/gcs_url from launch arguments for convenience.

    # NavSat Transform node (GPS to odom conversion)
    navsat_params = os.path.join(get_package_share_directory('rasprover_autonomy'), 'config', 'navsat_transform.yaml')
    navsat_node = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
        output='screen',
        parameters=[navsat_params],
        remappings=[('imu/data', '/mavros/imu/data'),           # IMU input from Pixhawk
                    ('gps/fix', '/mavros/global_position/raw/fix'),  # GPS fix input from Pixhawk
                    ('odometry/filtered', '/odometry/filtered')]     # EKF output (if used for yaw, but we set use_odometry_yaw false)
    )

    # EKF Localization node
    ekf_params = os.path.join(get_package_share_directory('rasprover_autonomy'), 'config', 'ekf_localization.yaml')
    ekf_node = Node(
        package='robot_localization', executable='ekf_localization_node', name='ekf_filter',
        output='screen',
        parameters=[ekf_params],
        remappings=[('odometry/filtered', '/odometry/filtered'),
                    ('odometry/gps', '/odometry/gps')]  # Ensure it listens to navsat output
    )

    # Pure Pursuit control node
    pp_node = Node(
        package='rasprover_autonomy', executable='pure_pursuit_node', name='pure_pursuit',
        output='screen',
        parameters=[{'waypoints': LaunchConfiguration('waypoints', default='')}],
        # The waypoints param can be passed via command line or config file; for simplicity can be left empty and loaded from config file below:
        # parameters=[os.path.join(get_package_share_directory('rasprover_autonomy'), 'config', 'waypoints.yaml')]
    )
    # Alternatively, uncomment to load from YAML:
    # waypoints_file = os.path.join(get_package_share_directory('rasprover_autonomy'), 'config', 'waypoints.yaml')
    # pp_node = Node(..., parameters=[waypoints_file])

    # (Optional) A safety to automatically set mode to OFFBOARD after a delay could be added:
    # e.g., using ExecuteProcess to call ros2 service after 5 seconds, but here we assume manual or QGC usage.

    return LaunchDescription(launch_args + [mavros_node, navsat_node, ekf_node, pp_node])
