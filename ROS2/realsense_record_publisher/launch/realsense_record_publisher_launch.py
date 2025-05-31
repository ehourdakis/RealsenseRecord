import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('dataset_directory', default_value='/home/vslam/data/col5/', description='Datafiles path'),
        DeclareLaunchArgument('rgb_index_file', default_value='rgb_aligned.txt', description='RGB index file'),
        DeclareLaunchArgument('rgb_calibration_filename', default_value='rgb.intrinsics', description='Calibration file'),
        DeclareLaunchArgument('rgb_distortion_coefficients_filename', default_value='rgb.distortion', description='Distortion coefficients file'),
        DeclareLaunchArgument('depth_index_file', default_value='depth_aligned.txt', description='Depth index file'),
        DeclareLaunchArgument('rgb_info_topic_name', default_value='/camera/color/camera_info', description='RGB info topic'),
        DeclareLaunchArgument('rgb_image_topic_name', default_value='/camera/color/image_raw', description='RGB image topic'),
        DeclareLaunchArgument('depth_info_topic_name', default_value='/camera/depth/camera_info', description='Depth info topic'),
        DeclareLaunchArgument('depth_image_topic_name', default_value='/camera/depth/image_rect_raw', description='Depth image topic'),

        # Node
        Node(
            package='realsense_record_publisher',
            executable='realsense_record_publisher',
            name='realsense_record_publisher',
            parameters=[{
                'dataset_directory': LaunchConfiguration('dataset_directory'),
                'rgb_index_file': LaunchConfiguration('rgb_index_file'),
                'rgb_calibration_filename': LaunchConfiguration('rgb_calibration_filename'),
                'rgb_distortion_coefficients_filename': LaunchConfiguration('rgb_distortion_coefficients_filename'),
                'depth_index_file': LaunchConfiguration('depth_index_file'),
                'rgb_info_topic_name': LaunchConfiguration('rgb_info_topic_name'),
                'rgb_image_topic_name': LaunchConfiguration('rgb_image_topic_name'),
                'depth_info_topic_name': LaunchConfiguration('depth_info_topic_name'),
                'depth_image_topic_name': LaunchConfiguration('depth_image_topic_name'),
            }],
            output='screen'
        ),

        # Optionally log output
        LogInfo(
            condition=None,
            msg="Launch file loaded and nodes started."
        ),
    ])
