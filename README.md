# Power Line Inspection Drone Simulation 

**Environment:** ROS 2 Jazzy | Gazebo Harmonic | PX4 Autopilot (v1.15+) | Ubuntu 24.04 

**Drone model:** : [CTU_CRAS_NORLAB_X500_SENSOR_CONFIG_1](https://app.gazebosim.org/OpenRobotics/fuel/models/CTU_CRAS_NORLAB_X500_SENSOR_CONFIG_1)

## 1. Project Overview
This project simulates an autonomous quadrotor drone performing power line inspection. It utilizes a highly accurate sensor configuration (LiDAR + Cameras) mounted on an X500 frame.
- **PX4 Autopilot:** For flight dynamics and control. 
- **MAVLink / QGroundControl:** The lightweight communication protocol used for telemetry, command and control, and linking the drone to Ground Control Stations. 
- **Gazebo Harmonic:** For physics environment. 
- **ROS 2 Jazzy:** For perception, path planning, and data visualization. 

## 2. Installation and setup


### Step 1: Install Gazebo Harmonic 

Check the installation instructions in the link: [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/install_ubuntu/)

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### Step 2: Install PX4 

Download PX4 source code (preferably to the home folder): 
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Create a pyenv:
```bash
pyenv virtualenv 3.12.3 px4
```

Activate the pyenv:
```bash
pyenv activate px4
```

Run the installation script:
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Restart the computer
```bash
sudo reboot
```

Navigate into the PX4 directory and build the software
```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

If everything went fine, you should see a gazebo window with the x500 drone model and a terminal for the PX4 controller. To proceed with the installation, you can close it with `CTRL C`. 

### Step 3: Install QGroundControl 
Enable serial-port access. Add your user to the dialout group so you can talk to USB devices without root:
```bash
sudo usermod -aG dialout "$(id -un)"
```

Install the dependencies: 
```bash 
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

Download the QGroundControl image: 
```bash
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
```

Make the AppImage executable 
```bash
chmod +x QGroundControl-x86_64.AppImage
```

Run QGroundControl. Either double-click the AppImage in your file manager or launch it from a terminal:
```bash
./QGroundControl-x86_64.AppImage
```

### Step 4: Install ROS 2 

Follow the installation instructions in the link: [ROS 2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)

Install colcon: 
```bash 
sudo apt install python3-colcon-common-extensions
```

Make sure that the pyenv is activated: 
```bash
pyenv activate px4
```

Install the dependencies: 
```bash 
pip3 install empy==3.3.4 catkin_pkg  kconfiglib transforms3d pyros-genmsg setuptools
```

Installation and setup of the Micro XRCE-DDS Agent & Client 
```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Create a ROS 2 workspace for work on the project
```bash 
mkdir -p ~/drone_sim_ws/src
cd ~/drone_sim_ws/src
```

Clone the example repository, px4_msgs, and the simulation repository
```bash
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/LeticiaRP/powerline_inspection
```

Compile the workspace: 
```bash 
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --executor sequential
source install/setup.bash 
```

## How to run the simulation 

### Terminal 1 
Gazebo environment 
```bash 
# Export Model Paths
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/drone_sim_ws/src/powerline_inspection_simulation/models:~/drone_sim_ws/src/powerline_inspection_simulation/worlds

# Launch Gazebo
gz sim -r inspection_valley.world
```

Tip: To save computing resources, you can launch Gazebo in headless (server) mode:
```bash 
gz sim -r -s inspection_valley.world
```


### Terminal 2

```bash
cd ~/PX4-Autopilot

# Export Configuration
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD=inspection_valley
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL_POSE='-48.24,72.7,0.23,0,0,0' 
export PX4_SIM_MODEL=CTU_CRAS_NORLAB_X500_SENSOR_CONFIG_1

# Run PX4
./build/px4_sitl_default/bin/px4
```
### Terminal 3
Launch the QGroundControl 
```bash
# Assuming that the AppImage is in Download directory
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

### Terminal 4 
ROS 2 bridge and visualization 
```bash 
# Source the environment 
pyenv activate px4 
source /opt/ros/jazzy/setup.bash 
source ~/drone_sim_ws/install/setup.bash

# Run the bridge 
ros2 launch powerline_inspection_simulation drone_bridge.launch.py
```
