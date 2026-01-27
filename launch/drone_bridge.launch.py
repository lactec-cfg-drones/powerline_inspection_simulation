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