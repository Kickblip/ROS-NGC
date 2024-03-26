Ubuntu 20.04

ROS 2 Galactic Installation
update bashrc with aliases

install interbotix_ros_toolboxes package
run from `colcon_ws/src`
```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_toolboxes.git
```
```bash
sudo apt-get install libyaml-cpp-dev
```

install the interbotix_ros_core package
```bash
mkdir -p interbotix_ros2_ws/src
cd interbotix_ros2_ws/src
git clone https://github.com/Interbotix/interbotix_ros_core.git -b galactic
cd interbotix_ros_core
git submodule init interbotix_ros_xseries/dynamixel_workbench_toolbox/
git submodule update
rm interbotix_ros_xseries/COLCON_IGNORE
cd ../..
rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO
colcon build
```

Install the missing dependencies

```bash
sudo apt install ros-galactic-dynamixel-sdk
sudo apt install ros-galactic-gazebo-ros-pkgs
sudo apt install ros-galactic-tf-transformations
sudo apt install ros-galactic-ros2-control
sudo apt install ros-galactic-ros2-controllers
```

add the drivers package to the /src directory
```bash
git clone https://github.com/Interbotix/interbotix_xs_driver.git
```

add the interbotix_ros_rover package /src directory
```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_rovers.git
```

