
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

def reset_esp32(port):
    ser = serial.Serial(port, 115200)
    ser.setDTR(False)  # Drop DTR
    ser.flushInput()
    ser.setDTR(True)  # Toggle DTR to reset the ESP32
    ser.close()

def on_exit_event(context):
    print("Node exited. Restarting the ESP32 devices.")
    reset_esp32("/dev/ESP32")


def generate_launch_description():
    #Catrographer
    map_saver_params_file = 'map_saver.yaml'
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                    cartographer_ros_prefix, 'configuration_files'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='rs_cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    apriltag_ros_prefix = get_package_share_directory('apriltag_ros')
    apriltag_ros_param_file = LaunchConfiguration('apriltag_ros_param_file', default=os.path.join(
                                                     apriltag_ros_prefix, 'cfg', 'tags_36h11.yaml'))
    

    # Paths
    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
   
    default_model_path = os.path.join(pkg_share, 'models/r2d2.urdf')
    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsenselidar.rviz')

    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace', default='')  # Assuming empty default namespace


    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_portleft = LaunchConfiguration('serial_port', default='/dev/leftlidar')
    serial_portright = LaunchConfiguration('serial_port', default='/dev/rightlidar')

    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_idleft = LaunchConfiguration('frame_id', default='lidarleft')
    frame_idright = LaunchConfiguration('frame_id', default='lidarright')

    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode_left = LaunchConfiguration('scan_mode', default='Boost')
    scan_mode_right = LaunchConfiguration('scan_mode', default='Standard')

    topic_nameleft = LaunchConfiguration('topic_name', default ='scan_left')
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
  
   
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
      name='use_robot_state_pub',
      default_value='True',
      description='Whether to start the robot state publisher')

    declare_resolution_cmd = DeclareLaunchArgument(
    'resolution',
    default_value='0.05',
    description='Resolution of a grid cell in the published occupancy grid')

    declare_use_namespace_cmd = DeclareLaunchArgument(
      name='use_namespace',
      default_value='False',
      description='Whether to apply a namespace to the navigation stack')

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')

    declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
      'cartographer_config_dir',
      default_value=cartographer_config_dir,
      description='Full path to config file to load')
    
    declare_publish_period_sec_cmd = DeclareLaunchArgument(
      'publish_period_sec',
      default_value='1.0',
      description='OccupancyGrid publishing period')
    
    declare_configuration_basename_cmd = DeclareLaunchArgument(
      'configuration_basename',
      default_value=configuration_basename,
      description='Name of lua file for cartographer')   


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



    start_rplidar_ros_left = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_left',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_portleft,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_idleft,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'topic_name': topic_nameleft,
                         'scan_mode':scan_mode_left}],
            output='screen')
    

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

    start_lidarleft_filter =  Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                {"scan_topic": "scan_left"},
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "angular_filter_left.yaml"
                ])],
        )
    
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



    start_micro_ros = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ESP32'],
            output='screen',
            name='micro_ros_agent_usb0',
        )
    

    
    start_hoverboard_bridge = LifecycleNode(
            package='hoverboard_bridge',
            name='hoverboard_bridge',
            namespace='',  # Set to an empty string if no specific namespace is needed
            executable='bridge_node',
            output='screen'
        )
    
    start_visual_odometry_right = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry_right',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 14.0}],
            )
    
    start_visual_odometry_left = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry_left',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan_left_filtered',
                    'odom_topic' : '/odom_left',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odomleft',
                    'init_pose_from_topic' : '',
                    'freq' : 15.0}],
            )
    

    start_visual_odometry = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 14.0}],
            )

    lifecycle_node =  LifecycleNode(package='demo_lifecycle', executable='lifecycle_talker',
                    name='lc_talker', namespace='', output='screen')
      
      
    demo_lifecycle_listener =  Node(package='demo_lifecycle', executable='listener', output='screen')
    demo_service_client =  Node(package='demo_lifecycle', executable='service_client', output='screen')


 




          # Laser Scan Merger Node
    laser_scan_filter_node = Node(
        package= 'laser_scan_filter_pkg',
        executable='laser_scan_filter_node',
        name='laser_scan_filter_node',
        output='screen',
        # remappings=[
        #     ('/scan_1', '/scan_1'),  # Ensuring it subscribes to the right topics
        #     ('/scan_2', '/scan_2'),
        #     ('/merged_scan', '/merged_scan')  # This is where the merged scan will be published
        # ]
    )


           # Laser Scan Merger Node
    laser_scan_merge_node = Node(
        package= 'laser_scan_merger_cam',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        # remappings=[
        #     ('/scan_1', '/scan_1'),  # Ensuring it subscribes to the right topics
        #     ('/scan_2', '/scan_2'),
        #     ('/merged_scan', '/merged_scan')  # This is where the merged scan will be published
        # ]
    )





    start_lidar_tf_publisher = Node(
        package='lidar_tf_publisher',
        executable='lidar_tf_publisher',
        name='lidar_tf_publisher',
        output='screen'
    )


    # Launch description
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_model_path_cmd)


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

    ld.add_action(declare_topic_name_left)
    ld.add_action(declare_topic_name_right)


    #ld.add_action(start_rplidar_ros_left)
    ld.add_action(start_rplidar_ros_right)
    #ld.add_action(start_lidarleft_filter)
    ld.add_action(start_lidarright_filter)

    ld.add_action(start_lidar_tf_publisher)
    ld.add_action(start_visual_odometry_right)

    return ld

