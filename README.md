# Forest replant simulation

## Dependencies
Install ros package submodule
```
git submodule update --init --recursive
```
Install python module
```
pip install kdtree # or pip3 install
```
Install Ros Package dwa local planner
```
sudo apt-get install ros-noetic-dwa-local-planner
```

## Build 
```
cd ros_code/catkin_ws
catkin build or catkin_make
source devel/setup.bash or devel/setup.zsh
```

## Run
```
export TURTLEBOT3_MODEL=burger
roslaunch planning_final_project demo.launch
roscd planning_final_project/scripts
./main
```
