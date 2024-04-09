
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir
import serial



def generate_launch_description():


    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    realsense_params_path = os.path.join(pkg_share, 'params', 'realsense_params.yaml')

    default_model_path = os.path.join(pkg_share, 'models/r2d2.urdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    


    camera1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',

        parameters=[ {'serial_no': '213322072999',
        'base_frame_id': 'base_link',
        'enable_acel' : True,
        'initial_reset': True,
        'enable_gyro': True,
        'enable_color': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_sync': True,
        'publish_odom_tf': True,
        'publish_tf': True,
        'unite_imu_method':2,
        'use_sim_time': False,
        'pointcloud.allow_no_texture_points': False,
                    'pointcloud.enable': True,
                    'pointcloud.filter_magnitude': 1,
                    'pointcloud.frames_queue_size': 16,
                    'pointcloud.ordered_pc': False,
                    'pointcloud.pointcloud_qos': 'DEFAULT',
                    'pointcloud.stream_filter': 2,              
                    'pointcloud.stream_format_filter': 0,
                    'pointcloud.stream_index_filter': 0}]


    )


    ld = LaunchDescription()
    ld.add_action(camera1_node)

    return ld