# SensorFusion

# Launching Project
roslaunch px4 mavros_posix_sitl.launch
roslaunch nodes.launch

# Downloading PX4
git clone https://github.com/PX4/Firmware.git
bash ./Firmware/Tools/setup/ubuntu.sh
cd Firmware
make px4_sitl_default gazebo

Add below to the bachrc file:
# Sourcing ROS and workspace
source /opt/ros/melodic/setup.bash
source /SensorFusion/autonomous_ws/develsetup.bash
# Sourcing and setup PX4 and Gazebo
source /home/$USER/src/Firmware/Tools/setup_gazebo.bash /home/$USER/src/Firmware /home/$USER/src/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware/Tools/sitl_gazebo
