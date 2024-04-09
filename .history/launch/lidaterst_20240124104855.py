
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




def generate_launch_description():


    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    default_model_path = os.path.join(pkg_share, 'models/r2d2.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_portleft = LaunchConfiguration('serial_port', default='/dev/leftlidar')
    serial_portright = LaunchConfiguration('serial_port', default='/dev/rightlidar')

    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace', default='')  # Assuming empty default namespace


    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_idleft = LaunchConfiguration('frame_id', default='lidarleft')
    frame_idright = LaunchConfiguration('frame_id', default='lidarright')

    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode_left = LaunchConfiguration('scan_mode', default='Boost')
    scan_mode_right = LaunchConfiguration('scan_mode', default='Standard')


    topic_nameright = LaunchConfiguration('topic_name', default ='scan_right')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]



    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
      name='gui',
      default_value='True',
      description='Flag to enable joint_state_publisher_gui')

 # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path, 
      description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=default_rviz_config_path,
      description='Full path to the RVIZ config file to use')




    declare_channel_type =  DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar')

    declare_serial_baudrate = DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar')

    declare_frame_id_right = DeclareLaunchArgument(
            'frame_id_right',
            default_value=frame_idright,
            description='Specifying frame_id of lidar')

    declare_frame_id_left = DeclareLaunchArgument(
            'frame_id_left',
            default_value=frame_idleft,
            description='Specifying frame_id of lidar')

    declare_inverted =  DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data')

    declare_angle_compensate =  DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data')
    declare_scan_mode_left = DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode_left,
            description='Specifying scan mode of lidar')
    declare_scan_mode_right = DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode_right,
            description='Specifying scan mode of lidar')    

    declare_use_namespace_cmd = DeclareLaunchArgument(
      name='use_namespace',
      default_value='False',
      description='Whether to apply a namespace to the navigation stack')

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')

        # For left lidar
    declare_serial_port_left = DeclareLaunchArgument(
        'serial_port_left',
        default_value=serial_portleft,
        description='Specifying usb port to connected left lidar')

    declare_topic_name_left = DeclareLaunchArgument(
        'topic_name_left',
        default_value='scan_left',
        description='Specifying topic name of left lidar scan data')

    # For right lidar
    declare_serial_port_right = DeclareLaunchArgument(
        'serial_port_right',
        default_value=serial_portright,
        description='Specifying usb port to connected right lidar')

    declare_topic_name_right = DeclareLaunchArgument(
        'topic_name_right',
        default_value='scan_right',
        description='Specifying topic name of right lidar scan data')

    start_rviz_cmd = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file])

    start_visual_odometry_right = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry_right',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan_right_filtered',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 14.0}],
            )

    start_rplidar_ros_right = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_right',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_portright,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_idright,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'topic_name': topic_nameright,
                         'scan_mode':scan_mode_right}],
            output='screen')

    start_lidarright_filter =  Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                {"scan_topic": "scan_right"},
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "angular_filter_right.yaml"
                ])],
        )
    start_joint_state_publisher_cmd = Node(

      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher')


    start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=namespace,
      parameters=[{'robot_description': Command(['xacro ', model])}],
      remappings=remappings,
      arguments=[default_model_path])


    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'autostart': autostart}.items())


    ld = LaunchDescription()



    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)   

    ld.add_action(declare_channel_type)
    ld.add_action(declare_serial_port_left)
    ld.add_action(declare_serial_port_right)
    ld.add_action(declare_serial_baudrate)
    ld.add_action(declare_frame_id_left)
    ld.add_action(declare_frame_id_right)
    ld.add_action(declare_inverted)
    ld.add_action(declare_angle_compensate)
    ld.add_action(declare_scan_mode_left)
    ld.add_action(declare_scan_mode_right)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_model_path_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(declare_topic_name_left)
    ld.add_action(declare_topic_name_right)
    ld.add_action(start_rplidar_ros_right)
    ld.add_action(start_lidarright_filter)
    ld.add_action(start_visual_odometry_right)  

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    return ld