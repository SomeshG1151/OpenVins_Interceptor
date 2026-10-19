from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('accel_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_scale', default_value='1.0'),
        DeclareLaunchArgument('use_filter', default_value='false'),
        DeclareLaunchArgument('filter_alpha', default_value='0.80'),
        DeclareLaunchArgument('filter_alpha_accel', default_value='0.80'),
        DeclareLaunchArgument('filter_alpha_gyro', default_value='0.80'),
        DeclareLaunchArgument('filter_order', default_value='1'),
        DeclareLaunchArgument('use_median_filter', default_value='false'),
        DeclareLaunchArgument('median_window', default_value='3'),

        Node(
            package='esp32_imu_driver',
            executable='esp32_imu_driver',
            name='esp32_imu_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'accel_scale': LaunchConfiguration('accel_scale'),
                'gyro_scale': LaunchConfiguration('gyro_scale'),
                'use_filter': LaunchConfiguration('use_filter'),
                'filter_alpha': LaunchConfiguration('filter_alpha'),
                'filter_alpha_accel': LaunchConfiguration('filter_alpha_accel'),
                'filter_alpha_gyro': LaunchConfiguration('filter_alpha_gyro'),
                'filter_order': LaunchConfiguration('filter_order'),
                'use_median_filter': LaunchConfiguration('use_median_filter'),
                'median_window': LaunchConfiguration('median_window'),
            }],
        ),
    ])