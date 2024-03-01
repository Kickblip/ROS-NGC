# Installation and Setup Guide
This guide lays out the steps to get ROS 1 and ROS 2 communicating on a fresh Ubuntu 18.04 (Bionic Beaver) install.

## Upgrading CMake 3.10 to 3.12
Ubuntu 18.04 will have CMake 3.10 installed by default but in order to build `ros1_bridge` later, we'll need to upgrade to 3.12


## 1. Installing ROS Melodic (ROS 1)
Instructions from the [ROS Wiki Melodic Installation Guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

### 1.1 Ubuntu Repository Access
Ensure your Ubuntu repository sources are configured to allow "restricted," "universe," and "multiverse." follow the [Ubuntu Guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for more info

### 1.2 Setup sources.list
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### 1.3 Setup Your Keys
```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### 1.4 Installation
Install the desktop (full) version of ROS Melodic
```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
```
### 1.5 Dependencies
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
### 1.6 Initialize Rosdep
```bash
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## 2. Installing ROS Eloquent (ROS 2)
Instructions from the [ROS Wiki Eloquent Installation Guide](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)

### 2.1 Initalize Sources
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
### 2.2 Installation
```bash
sudo apt update
```
```bash
sudo apt install ros-eloquent-desktop
```
### 2.3 Install argcomplete (Optional)
```bash
sudo apt install -y python3-pip
pip3 install -U argcomplete
```
### 2.4 Install Colcon (Build tool)
```bash
sudo apt install python3-colcon-common-extensions
```

## 3. Create workspaces
```bash
mkdir -p ~/catkin_ws/src
mkdir -p ~/colcon_ws/src
mkdir -p ~/bridge_ws/src
```

## 4. Creating Aliases (Optional, Recommended)
Open `.bashrc` with a text editor (nano) using
```bash
cd
nano .bashrc
```
And add the following lines to the end of the file
```bash
alias melodic='source /opt/ros/melodic/setup.bash'
alias eloquent='source /opt/ros/eloquent/setup.bash'
alias bridge='source ~/ros1_bridge/install/setup.bash'
```
Now you can source a ros workspace by simply typing
```bash
melodic
```

## 5. Building ros1_bridge
