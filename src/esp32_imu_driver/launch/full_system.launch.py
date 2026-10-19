from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Config for your custom sensor (create this in step 7)
    config_arg = DeclareLaunchArgument(
        'config', default_value='my_laptop_esp32',
        description='Config folder name under ov_msckf/config'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/v4l/by-id/usb-CCB_Camera_CCB_Camera-video-index0',
        description='V4L2 camera device path (defaults to external CCB fixed-focus camera)'
    )
    
    # Cleanup stale v4l2_camera processes holding camera devices
    cleanup_camera = ExecuteProcess(
        cmd=['bash', '-c', 'fuser -k /dev/video2 /dev/video0 2>/dev/null; sleep 0.5'],
        output='screen',
        name='cleanup_video'
    )
    
    # Camera node (defaults to external /dev/video2)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 360],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'cam0',
        }],
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('camera_info', '/cam0/camera_info'),
        ]
    )

    # ESP32 IMU driver
    imu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('esp32_imu_driver'),
                'launch/esp32_imu.launch.py'
            ])
        ]),
        launch_arguments={
            'port': '/dev/ttyACM0',
            'baudrate': '115200',
            'frame_id': 'imu_link',
            'accel_scale': '1.0',
            'gyro_scale': '1.0',
        }.items()
    )

    # OpenVINS node
    ov_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='run_subscribe_msckf',
        namespace='ov_msckf',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'verbosity': 'INFO',
            'config_path': PathJoinSubstitution([
                get_package_share_directory('ov_msckf'),
                'config', LaunchConfiguration('config'), 'estimator_config.yaml'
            ]),
            'use_stereo': False,
            'max_cameras': 1,
            'save_total_state': True,
            'filepath_est': '/home/bhadwabhindi/workspace/catkin_ws_ov/logs/latest_trajectory.txt',
            'filepath_std': '/home/bhadwabhindi/workspace/catkin_ws_ov/logs/latest_uncertainty.txt',
        }],
        remappings=[
            ('/cam0/image_raw', '/cam0/image_raw'),
            ('/imu/data_raw', '/imu/data_raw'),
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            get_package_share_directory('ov_msckf'), 'launch', 'display_ros2.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        video_device_arg,
        cleanup_camera,
        camera_node,
        imu_node,
        ov_node,
        rviz_node,
    ])