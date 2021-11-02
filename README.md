# CoppeliaSim & Fetch

## Description

This branch provides the solution to setup the simulation environment with Fetch robot and arm and mobile base Ros controllers which have the same 
format with the controllers of real Fetch robot, so it will be easy for users to switch between real world and the simulation. 
## Install guide

System requirement:

1. Ubuntu 18.04
2. ROS melodic
3. CoppeliaSim ([Download](https://coppeliarobotics.com/downloads))
4. CMake 3.16 version ([Download](https://cmake.org/download/))([CMake install tutorial](https://cmake.org/install/))(We use 3.20.1)
5. ros-melodic-navigation package(sudo apt-get install ros-melodic-navigation)
6. ros-melodic-joint-trajectory-controller package(sudo apt-get install ros-melodic-joint-trajectory-controller)
7. ros-melodic-diff-drive-controller package(sudo apt-get install ros-melodic-diff-drive-controller)

### Install Steps

1. change the Coppeliasim directory name to "CoppeliaSim"
2. Add following codes into .bashrc
```
export COPPELIASIM_ROOT_DIR=$HOME/CoppeliaSim
```
3. mkdir catkin_ws
4. clone the repository into catkin_ws
5. change that directory's name to "src"
6. run following command in catkin_ws for compiling the code
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```
7. Run following commands to copy plugin to CoppeliaSim
```
cp  $HOME/catkin_ws/devel/lib/libsimExtRosControl.so  $HOME/CoppeliaSim/
cp  $HOME/catkin_ws/devel/lib/libsimExtRosServices.so  $HOME/CoppeliaSim/
cp  $HOME/catkin_ws/devel/lib/libsimExtROS.so  $HOME/CoppeliaSim/
```
8. run following command in catkin_ws
```
source devel/setup.bash
```

### Launch Steps
1. roslaunch fetch_coppeliasim simulation.Launch
2. roslaunch fetch_coppeliasim fetch_control.launch