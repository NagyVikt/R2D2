
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