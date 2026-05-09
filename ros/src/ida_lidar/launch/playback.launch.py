from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Arguments ---
    # Teammates pass this on the command line:
    #   ros2 launch ida_lidar playback.launch.py bag_path:=/path/to/your/bag
    bag_path = LaunchConfiguration('bag_path')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            description='Absolute path to the ROS 2 bag directory to play back'
        ),

        # --- GPS/IMU Odometry node ---
        # Converts GPS + IMU data into the odom → base_link TF transform.
        # This replaces the static_transform_publisher workaround and accounts
        # for actual boat movement/drift during a run.
        Node(
            package='ida_lidar',
            executable='gps_imu_odom',
            name='gps_imu_odom',
            output='screen',
        ),

        # --- Perception node ---
        # Subscribes to /gazebo_lidar/points (replayed from the bag),
        # runs the full PCL pipeline, publishes /buoy_markers.
        Node(
            package='ida_lidar',
            executable='buoy_lidar',
            name='buoy_lidar',
            output='screen',
        ),

        # --- Bag playback ---
        # --loop keeps replaying so teammates can observe the full sequence
        # repeatedly without restarting the launch file.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--loop'],
            output='screen',
        ),

        # --- RViz2 ---
        # Launched with our pre-configured layout so teammates don't need
        # to manually add topics or set the fixed frame.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('ida_lidar'), 'rviz', 'buoy_detection.rviz'
            ])],
            output='screen',
        ),

    ])
