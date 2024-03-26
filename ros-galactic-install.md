# Installing the Interbotix ROS Package Suite
This guide covers the steps necessary to install and build the `interbotix_ros_rovers` package to start developing on the LoCoBot. This guide is made for ROS 2 Galactic running on Ubuntu 20.04 (Focal)

## Installing ROS 2 Galactic
These steps are copied from the [official Galactic installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

1. Verify locale settings (mainly for minimal environments like a Docker container)
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. Ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3. Add ROS 2 GPG key with apt
```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
4. Add the repository to your sources list
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
5. Update your apt repository caches after setting up the repositories.
```bash
sudo apt update
```
6. ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```bash
sudo apt upgrade
```
7. Install ROS 2 packages
```bash
sudo apt install ros-galactic-desktop
```
8. Open your `.bashrc` file with a text editor.
```bash
cd
nano ~/.bashrc
```
9. Add the following line to the bottom of the file.
```bash
alias galactic='source /opt/ros/galactic/setup.bash'
```
10. Restart your `.bashrc` to reflect these changes.
```bash
source ~/.bashrc
```
Now you can source your ROS environment in a new terminal by simply typing
```bash
galactic
```
## Installing `interbotix_ros_rovers` Package Dependencies

# TODO: SETUP COLCON WORKSPACE

1. Install `interbotix_ros_toolboxes` package
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

