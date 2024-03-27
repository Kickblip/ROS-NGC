# Installing the Interbotix ROS Package Suite
This guide covers the steps necessary to install and build the `interbotix_ros_rovers` package to start developing on the LoCoBot. This guide is made for ROS 2 Galactic running on Ubuntu 20.04 (Focal).

## Installing ROS 2 Galactic
These steps are copied from the [official Galactic installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

1. Verify locale settings (mainly for minimal environments like a Docker container).
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. Ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3. Add ROS 2 GPG key with apt.
```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
4. Add the repository to your sources list.
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
7. Install ROS 2 packages.
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
Now you can source your ROS environment in a new terminal by simply typing.
```bash
galactic
```
11. Install and initialize Rosdep
```bash
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```
12. Now we will install Colcon (the ROS 2 build tool).
```bash
sudo apt install python3-colcon-common-extensions
```
13. And finally, we will initialize our Colcon workspace.
```bash
mkdir -p ~/colcon_ws/src
```

## Installing `interbotix_ros_rovers` Package Dependencies
The four main Interbotix packages are dependent on eachother in the following way:
```
interbotix_ros_rovers
  └── interbotix_ros_toolboxes
  └── interbotix_ros_core
      └── interbotix_ros_xs_driver
```
**NOTE:** This tree is just a way to show dependencies, all these packages will go in `~/colcon_ws/src`.

1. Install `interbotix_ros_toolboxes` package in your `~/colcon_ws/src` directory.
```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_toolboxes.git
```
2. Install the required C++ YAML parser.
```bash
sudo apt-get install libyaml-cpp-dev
```

3. Install the `interbotix_ros_core` package.
```bash
git clone https://github.com/Interbotix/interbotix_ros_core.git -b galactic
```
4. Initialize the `dynamixel_workbench_toolbox` package.
```bash
cd interbotix_ros_core
git submodule init interbotix_ros_xseries/dynamixel_workbench_toolbox/
git submodule update
rm interbotix_ros_xseries/COLCON_IGNORE
```
5. Install Missing dependencies (these were the ones I needed to install, it may be different for other systems).
```bash
sudo apt install ros-galactic-dynamixel-sdk ros-galactic-gazebo-ros-pkgs ros-galactic-tf-transformations ros-galactic-ros2-control ros-galactic-ros2-controllers
```
6. Add the `interbotix_xs_driver` drivers package to your `~/colcon_ws/src`.
```bash
git clone https://github.com/Interbotix/interbotix_xs_driver.git
```
7. Now build your workspace from `~/colcon_ws` (the root of your workspace) to make sure everything is working.
```bash
galactic
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro=galactic
colcon build
```
8. If the build was successful, it is time to install all the LoCoBot packages. Clone the following repo into your `~/colcon_ws/src` directory.
```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_rovers.git
```
9. And finally, try to build the workspace once more.
```bash
cd ..
colcon build
```
If you are getting errors regarding missing paths, try deleting the `install` and `build` directories from your workspace and rebuilding.
```bash
rm -rf install build
colcon build
```

