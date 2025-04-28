# realsense_record_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_record_publisher',
            executable='realsense_record_publisher',
            name='realsense_record_publisher',
            output='screen',
            parameters=[{
                'dataset_directory': '/overlay_ws/data/col6/',
                #'dataset_directory': '/home/manos/Downloads/felice dev/try2',
                'rgb_index_file': 'rgb_aligned.txt',
                'rgb_calibration_filename': 'rgb.intrinsics',
                'rgb_distortion_coefficients_filename': 'rgb.distortion',
                'depth_index_file': 'depth_aligned.txt',
                'rgb_info_topic_name': '/camera/color/camera_info',
                'rgb_image_topic_name': '/camera/color/image_raw',
                'depth_info_topic_name': '/camera/aligned_depth_to_color/camera_info',
                'depth_image_topic_name': '/camera/aligned_depth_to_color/image_raw'
            }]
        )
    ])
