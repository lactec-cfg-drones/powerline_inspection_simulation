import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'powerline_inspection_simulation'

# Constuction to pyenv px4 environment
px4_python_path = os.path.join(
os.path.expanduser('~'), 
'.pyenv', 
'versions', 
'line_detector', 
'bin', 
'python3'
)

# Function to collect all model files including meshes
def get_model_data_files():
    data_files = []
    
    # Base model directory
    model_dir = 'models'
    
    # Walk through all directories in models
    for root, dirs, files in os.walk(model_dir):
        for file in files:
            # Include all relevant model files
            if file.endswith(('.sdf', '.config', '.dae', '.stl', '.obj', '.png', '.jpg', '.mtl')):
                src_file = os.path.join(root, file)
                # Destination is in share/package_name/models/...
                dest_dir = os.path.join('share', package_name, root)
                data_files.append((dest_dir, [src_file]))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + get_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leticia',
    maintainer_email='leticiarp2000@hotmail.com',
    description='Autonomous inspection drone simulation using ROS 2 Jazzy, Gazebo Harmonic, and PX4. Features custom catenary wire worlds, X500 sensor setup, and complete launch configuration for ROS 2/Gazebo bridging and visualization.',
    license='Apache License',

    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_tf_broadcaster = powerline_inspection.odometry_bridge_node:main',
            'waypoint_cmd_node = powerline_inspection.waypoint_cmd_node:main',
            'line_detector_node = powerline_inspection.line_detector_node:main', 
        ],
    },
    options={
        'build_scripts': {
            'executable': px4_python_path
        }
    }
)