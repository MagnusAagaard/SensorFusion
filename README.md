# SensorFusion

# Launching Project
roslaunch px4 mavros_posix_sitl.launch  
roslaunch sensor_fusion nodes.launch  

# Downloading PX4 and setup this repository
Clone PX4 to a folder on your computer, NOT THIS GITHUB REPOSITORY!  
git clone https://github.com/PX4/Firmware.git  
Clone this repository to another location on your computer. Where you want this repository to be  
git clone https://github.com/MagnusAagaard/SensorFusion.git  
cd ~path/to/this/repository  
bash ./ubuntu_sim_ros_melodic.sh  
cd ~path/to/cloned/px4/repository/Firmware  
make px4_sitl_default gazebo  
  
Add below to the bachrc file: Note that the paths should be changed accordingly  
source /home/$USER/src/Firmware/Tools/setup_gazebo.bash /home/$USER/src/Firmware /home/$USER/src/Firmware/build/px4_sitl_default  
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware  
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/src/Firmware/Tools/sitl_gazebo 

# Sourcing ROS and workspace
Before launching launch files you should source your workspace:  
source ./SensorFusion/autonomous_ws/devel/setup.bash  
