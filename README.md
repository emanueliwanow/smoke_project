# smoke_project
Repository for implementing MPC control to stabilize a MAV.


## Installing the PX4 Simulation 

```
sudo apt update
sudo apt upgrade
sudo apt install git
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git --recursive
cd Firmware
bash ./Tools/setup/ubuntu.sh

## reboot computer
https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh

## close the terminal and open it again
cd src/Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo

source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo

## Make sure to add the above inside the .bashrc file if you want to run it everytime from the terminal. The $pwd should be replaced with the path to Firmware folder.

roslaunch px4 multi_uav_mavros_sitl.launch
```