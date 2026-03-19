import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('powerline_inspection')
    
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths
    sdf_path = os.path.join(
        pkg_dir, 
        'models', 
        'CTU_CRAS_NORLAB_X500_SENSOR_CONFIG_1', 
        'model_rviz.sdf'
    )
    
    config_file_path = os.path.join(pkg_dir, 'config', 'ros_gz_bridge.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'rviz_config.rviz')
    
    # Read SDF file for robot description
    with open(sdf_path, 'r') as f:
        robot_desc = f.read()
    
    # ROS-GZ Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': config_file_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./air_pressure.publisher.reliability': 'best_effort',


            # laser scan configuration 
            'qos_overrides./front_laser/scan.reliability': 'best_effort',
            'qos_overrides./front_laser/scan.durability': 'volatile',
            'qos_overrides./front_laser/scan.history': 'keep_last',
            'qos_overrides./front_laser/scan.depth': '5',
            

            # point cloud configuration
            'qos_overrides./front_laser/point_cloud.reliability': 'best_effort',
            'qos_overrides./front_laser/point_cloud.durability': 'volatile',
            'qos_overrides./front_laser/scan.history': 'keep_last',
            'qos_overrides./front_laser/point_cloud.depth': '5',

            'qos_overrides./rs_down/point_cloud.reliability': 'best_effort',
            'qos_overrides./rs_down/point_cloud.durability': 'volatile',
            'qos_overrides./rs_down/point_cloud.history': 'keep_last',
            'qos_overrides./rs_down/point_cloud.depth': '5',

            'qos_overrides./rs_up/point_cloud.reliability': 'best_effort',
            'qos_overrides./rs_up/point_cloud.durability': 'volatile',
            'qos_overrides./rs_up/point_cloud.history': 'keep_last',
            'qos_overrides./rs_up/point_cloud.depth': '5',


            # image configuration 
            'qos_overrides./camera_front/image.reliability': 'best_effort',
            'qos_overrides./camera_front/image.durability': 'volatile',
            'qos_overrides./camera_front/image.history': 'keep_last',
            'qos_overrides./camera_front/image.depth': '5',

            'qos_overrides./rs_down/depth_image.reliability': 'best_effort',
            'qos_overrides./rs_down/depth_image.durability': 'volatile',
            'qos_overrides./rs_down/depth_image.history': 'keep_last',
            'qos_overrides./rs_down/depth_image.depth': '5',

            'qos_overrides./rs_down/image.reliability': 'best_effort',
            'qos_overrides./rs_down/image.durability': 'volatile',
            'qos_overrides./rs_down/image.history': 'keep_last',
            'qos_overrides./rs_down/image.depth': '5',

            'qos_overrides./rs_up/depth_image.reliability': 'best_effort',
            'qos_overrides./rs_up/depth_image.durability': 'volatile',
            'qos_overrides./rs_up/depth_image.history': 'keep_last',
            'qos_overrides./rs_up/depth_image.depth': '5',

            'qos_overrides./rs_up/image.reliability': 'best_effort',
            'qos_overrides./rs_up/image.durability': 'volatile',
            'qos_overrides./rs_up/image.history': 'keep_last',
            'qos_overrides./rs_up/image.depth': '5',

            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ],
        output='screen'
    )

    # Add this to the 'nodes' list in your launch file
    drone_tf_node = Node(
        package='powerline_inspection',
        executable='drone_tf_broadcaster',
        name='drone_tf_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'base_link_height': 0.23}],
        output='screen'
    )
        
    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    ]
    
    # Nodes to launch
    nodes = [
        bridge,
        robot_state_publisher,
        drone_tf_node,
        rviz2,
    ]
    
    return LaunchDescription(launch_args + nodes)