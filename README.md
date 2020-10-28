# SensorFusion

# Sourcing and setup PX4 and Gazebo
source /home/$USER/src/Firmware/Tools/setup_gazebo.bash /home/$USER/src/Firmware /home/$USER/src/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware/Tools/sitl_gazebo
