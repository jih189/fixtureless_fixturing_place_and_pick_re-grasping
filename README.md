# CoppeliaSim & Fetch

## Description

This branch provides the solution to setup the simulation environment with Fetch robot and arm and mobile base Ros controllers which have the same 
format with the controllers of real Fetch robot, so it will be easy for users to switch between real world and the simulation. 
## Install guide

System requirement:

1. Ubuntu 18.04
2. python 3.8
3. ROS melodic
4. CoppeliaSim ([Download](https://coppeliarobotics.com/downloads))
5. CMake 3.16 version ([Download](https://cmake.org/download/))([CMake install tutorial](https://cmake.org/install/))(We use 3.20.1)
6. ros-melodic-navigation package(sudo apt-get install ros-melodic-navigation)
7. ros-melodic-joint-trajectory-controller package(sudo apt-get install ros-melodic-joint-trajectory-controller)
8. ros-melodic-diff-drive-controller package(sudo apt-get install ros-melodic-diff-drive-controller)

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
## Set initial joint values on Fetch
You are allowed to change the initial joint value of the arm. Following is how you can do it:
1. Stop the simulation
2. Select the joint you want to change its value in Scene hierachy
3. Open its "Scene Object Properties"
4. set the position value in the "Configuration" section of "Joint" tag.
## Scene Building
There are three ways to import the scene:
1. Import by .ttt(File->Open Scene...). This will load the scene directly, but we do not have enough .ttt format rooms currently.
2. Import by loading mesh files(File->Import->Mesh...), so you can select a mesh file(.obj, .dae, .stl). However, such kind of object have no physics property and will flow into the air. Therefore, 
you need to add enough physics properties by yourself(select the object, and click "Scene Object Properties" item on the left side of the window, then click "show dynamic properties dialog" button at
the button of the popped up window, and click the "Body is respondable" and "Body is dynamic" checkbox).

3. Import by "URDF import"(Plugins->URDF import...). If you want to import a urdf file, you need to put the file under the "objects_desciption" directory, and recompile and source the workspace.
Then you can launch Coppeliasim and use the URDF import. Remeber uncheck "Convex decompose non-convex collision..." and check "Alternate local respondable masks". Them you can import the urdf file. 

If you want to use the urdf from "partnet", you have to do some modifications. For example, you download a table urdf file named "32761". After you unzip it, change its name
to "table" if you can, and move it under the "objects_desciption". Open the "mobility.urdf" file, and change all "filename="" to "filename="package://objects_desciption/[object name]/", so
the URDF import can recognize it later.

Warming:

The object's origin point will be modifed after import.

https://forum.coppeliarobotics.com/viewtopic.php?t=885
