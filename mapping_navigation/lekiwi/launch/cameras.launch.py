#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_bottom_camera_arg = DeclareLaunchArgument(
        'start_bottom_camera',
        default_value='true',
        description='Start the bottom camera feed'
    )

    start_wrist_camera_arg = DeclareLaunchArgument(
        'start_wrist_camera',
        default_value='false',
        description='Start the wrist camera feed (if available)'
    )

    start_bottom_camera = LaunchConfiguration('start_bottom_camera')
    start_wrist_camera = LaunchConfiguration('start_wrist_camera')

    # Bottom camera - adjust /dev/video0 to match your camera device
    bottom_camera_node = Node(
        condition=IfCondition(start_bottom_camera),
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='bottom_camera',
        parameters=[{
            'video_device': '/dev/video0',  # primary camera device
            'framerate': 15.0,             # reduced for stability
            'io_method': 'mmap',
            'pixel_format': 'yuyv2rgb',
            'image_width': 640,
            'image_height': 480,
            'camera_name': 'bottom_camera',
            'camera_info_url': '',
            'brightness': 50,
            'auto_white_balance': True,
            'autoexposure': True,
            'autofocus': False,
        }],
        remappings=[
            ('image_raw', '/bottom_camera/image_raw'),
            ('camera_info', '/bottom_camera/camera_info'),
        ],
        output='screen',
    )

    # Wrist camera - optional second camera
    wrist_camera_node = Node(
        condition=IfCondition(start_wrist_camera),
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='wrist_camera',
        parameters=[{
            'video_device': '/dev/video2',  # adjust to match your second camera
            'framerate': 15.0,
            'io_method': 'mmap',
            'pixel_format': 'yuyv2rgb',
            'image_width': 640,
            'image_height': 480,
            'camera_name': 'wrist_camera',
            'camera_info_url': '',
            'brightness': 50,
            'auto_white_balance': True,
            'autoexposure': True,
            'autofocus': False,
        }],
        remappings=[
            ('image_raw', '/wrist_camera/image_raw'),
            ('camera_info', '/wrist_camera/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        start_bottom_camera_arg,
        start_wrist_camera_arg,
        bottom_camera_node,
        wrist_camera_node,
    ]) 