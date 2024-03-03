# Installation and Setup Guide
This guide lays out the steps to get ROS 1 and ROS 2 communicating on a fresh Ubuntu 18.04 (Bionic Beaver) install.  This is **NOT** a guide on how to get the NGC package working on your machine.

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
Reload `.bashrc`
```bash
source ~/.bashrc
```
Now you can source a ROS workspace by simply typing
```bash
melodic
```

## 5. Bridging ROS 1 and ROS 2 Messages

### 5.1 Defining messages for ROS 1
To define what messages will be bridged from ROS 1 to ROS 2, we need to create a package in our ROS 1 and ROS 2 workspaces (~/catkin_ws and ~/colcon_ws)
**IMPORTANT**: Make sure you only have one workspace sourced at a time while doing this. For example, when creating the ROS 1 bridge package, do not have ROS eloquent sourced.

#### 5.1.1 Create ROS 1 bridge package
```bash
cd ~/catkin_ws/src
melodic
catkin_create_pkg custom_msg_ros1 rospy std_msgs
```

#### 5.1.2 Add message definitions
```bash
cd custom_msg_ros1
mkdir msg
touch CustomMessage.msg
```

#### 5.1.3 Placeholder value in message definition
Add the following line to your new CustomMessage.msg file as a placeholder.
```bash
float64 custom_value
```

#### 5.1.4 Update CMakeLists
Add the following line to your CMakeLists.txt file in the `custom_msg_ros1` package root (**WITHOUT** the '+').
```bash
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
+ message_generation
)
```
Then uncomment and edit these lines.
```bash
add_message_files(
  FILES
+ CustomMessage.msg
)
```
```bash
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

#### 5.1.5 Update Package.xml
Add the following dependency declarations to your Package.xml file.
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

#### 5.1.6 Build workspace
To make sure everything is working properly, run the following command from the **root** of your ROS 1 workspace `~/catkin_ws`.
```bash
catkin_make
```
Then test your message.
```bash
source devel/setup.bash
rosmsg info custom_msg_ros1/CustomMessage
```
If you see `float64 custom_value` then you are free to continue.

#### 5.1.7 Create talker node for testing
Create the file `talker.py` using the publisher node example from the [ROS 1 Tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29). Add this node under `/src` in `custom_msg_ros1`.
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from custom_msg_ros1.msg import CustomMessage

def talker():
    pub = rospy.Publisher('chatter', CustomMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = CustomMessage()
        msg.custom_value = 1.22
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Declare the new node in `CMakeLists.txt` so Catkin knows to build it
```bash
catkin_install_python(PROGRAMS
+ src/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 5.2 Defining messages for ROS 2
Now we will repeat 5.1 for ROS 2.

#### 5.2.1 Create ROS 2 package
Open a new terminal (**WITHOUT** a ROS workspace sourced) and navigate to your ROS 2 workspace `~/colcon_ws`. 
```bash
cd colcon_ws/src
eloquent
ros2 pkg create --build-type ament_cmake custom_msg_ros2 --dependencies rclcpp
```

#### 5.2.2 Add message definitions
```bash
cd custom_msg_ros1
mkdir msg
touch CustomMessage.msg
```

#### 5.2.3 Placeholder value in message definition
Add the following line to your new CustomMessage.msg file as a placeholder.
```bash
float64 custom_value
```

#### 5.2.4 Update CMakeLists
```bash
+ rosidl_generate_interfaces(${PROJECT_NAME}
+   "msgs/CustomMessage.msg"
+ )
```

#### 5.2.5 Update Package.xml
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

# TODO: ADD 5.2.6 TALKER AND LISTENER NODES

### 5.3 Building the `ros1_bridge` package
At this point, we should have two packages for ROS 1 and ROS 2 that have custom messages defined. In order for them to communicate across ROS versions, we will build the [ros1_bridge package](https://github.com/ros2/ros1_bridge) from source and start using it.

#### 5.3.1 Cloning repository
Run the following from your bridge workspace's source folder `~/bridge_ws/src`.
```bash
git clone https://github.com/ros2/ros1_bridge.git
```

#### 5.3.2 Define message mappings
For this guide, the goal is to get our `custom_msg_ros1` and `custom_msg_ros2` packages communicating. Since they have different names, we will need to define a mapping to tell `ros1_bridge` how to handle these messages. This mapping will be a YAML file that is put **IN THE ROOT OF YOUR ROS 2 WORKSPACE**

This guide only touches on simple package mappings, for more complex mappings refer to the [ros1_bridge package documentation](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)

**Create and edit** the following file `my_bridge_mapping.yaml` in the root of `colcon_ws`
```yaml
-
  ros1_package_name: 'custom_msg_ros1'
  ros2_package_name: 'custom_msg_ros2'
```

You will also have to update the `CMakeLists.txt` file in your `colcon_ws` to reflect these changes
```bash
+ install(
+   FILES my_bridge_mapping.yaml
+   DESTINATION share/${PROJECT_NAME})
```

And update `package.xml` in the same directory
```xml
<export>
  <build_type>ament_cmake</build_type>
+ <ros1_bridge mapping_rules="my_bridge_mapping.yaml" />
</export>
```

Now build your `colcon_ws` by running the following commands from the workspace root
```bash
eloquent
colcon build
```

#### 5.3.3 Build `ros1_bridge`
Open a new terminal with no workspaces sourced and run the following commands. **NOTE:** The final build may take a few minutes.
```bash
melodic
eloquent
source catkin_ws/devel/setup.bash
source colcon_ws/install/setup.bash
cd bridge_ws
colcon build --symlink-install --cmake-force-configure
```

#### 5.3.4 Run the bridge
```bash
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i custom
```






