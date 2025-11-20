#!/usr/bin/env python3
"""
Launch file for Image Conversion Node with usb_cam.
Launches both usb_cam and image_conversion nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for image conversion system."""
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/image_raw',
        description='Topic name for input camera images'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_image_topic',
        default_value='/image_converted',
        description='Topic name for output converted images'
    )
    
    mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='true',
        description='Initial conversion mode (true=grayscale, false=color)'
    )
    
    # Get launch configurations
    input_topic = LaunchConfiguration('input_image_topic')
    output_topic = LaunchConfiguration('output_image_topic')
    initial_mode = LaunchConfiguration('initial_mode')
    
    # USBCam Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            {'camera_name': 'default_cam'},
            {'camera_info_url': 'package://usb_cam/config/camera_info.yaml'},
            {'framerate': 30.0},
            {'frame_id': 'camera_frame'},
        ]
    )
    
    # Image Conversion Node
    image_conversion_node = Node(
        package='image_conversion',
        executable='image_conversion_node',
        name='image_conversion',
        output='screen',
        parameters=[
            {'input_topic': input_topic},
            {'output_topic': output_topic},
            {'grayscale_mode': initial_mode},
        ],
        remappings=[
            ('/image_raw', input_topic),
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        mode_arg,
        usb_cam_node,
        image_conversion_node,
    ])
