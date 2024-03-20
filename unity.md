# Installation and Setup Guide
This guide lays out the steps to get Unity and ROS 2 Foxy integrated and communicating

## 1. Installing Unity
First, we will install Unity through the Unity Hub

### 1.1 Unity Hub
Download the Unity Hub setup file from the official Unity CDN

https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

Ensure the App Image file is executable
```bash
chmod +x path/to/UnityHub.AppImage
```

### 1.2 Unity Editor
Then open the App Image file (this may take a few minutes). Once opened, install the latest Unity editor version (at least 2020.X.X).

## 2. Setting up ROS 2 Foxy
To best run ROS 2 Foxy, we will use a Docker image from the Unity Robotics Hub GitHub repository

### 2.1 Installing Docker
```bash
sudo apt update
sudo apt install snapd
sudo snap install docker
```

### 2.2 Running Docker Containers
Next, you will clone the repository that contains the necessary files.
```bash
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```
Once cloned, navigate to the directory `tutorials/ros_unity_integration` and run the following command.
```bash
docker build -t foxy -f ros2_docker/Dockerfile .
docker run -it --rm -p 10000:10000 foxy /bin/bash
```
This should build a docker image and start it. We will also open a new terminal tab `shift+ctrl+T` and open a second terminal of the same container.
```bash
docker ps
```
This will show you the currently running Docker containers. Initialize a second terminal with 
```bash
docker exec -it <container name> bash
```
You should now have two terminal tabs each running the same Docker container

### 2.3 Running the server endpoint
In the first terminal tab, run the following command. Because we are using Docker, you can safely replace `<your IP address>` with `0.0.0.0`
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address>
```

## Setting up Unity
1. Launch Unity and create a new 3D project.
2. Open the Package Manager window under the "Window" tab at the top of the editor
3. Click the + button at the top left corner. Select "add package from git URL" and enter "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector" to install the (ROS-TCP-Connector)[https://github.com/Unity-Technologies/ROS-TCP-Connector] package.

![add_package_2](https://github.com/Kickblip/ROS-NGC/assets/54160850/4e220f5b-f7df-4e88-a074-c31b8c3b533a)

4. Open ROS Settings under the new "Robotics" tab at the top of the editor and change the protocol to ROS 2.

![ros2_protocol](https://github.com/Kickblip/ROS-NGC/assets/54160850/a83d3850-b9a6-407c-b819-227e4be04b93)

5. In the Unity menu bar (at the top), go to `Robotics` -> `Generate ROS Messages`.... In the Message Browser window, click the Browse button at the top right to set the ROS message path to `tutorials/ros_unity_integration/ros_packages/unity_robotics_demo_msgs` in this repo.

6. In the message browser, expand the unity_robotics_demo_msgs subfolder and click "Build 2 msgs" and "Build 2 srvs" to generate C# scripts from the ROS .msg and .srv files.

![generate_messages_3](https://github.com/Kickblip/ROS-NGC/assets/54160850/be7f8357-87c3-467f-b48e-e9794fc73551)

## Creating the Unity publisher
Create a simple Unity scene which publishes a GameObject's position and rotation to a ROS topic.

In your Project tab in Unity, create a new C# script and name it `RosPublisherExample`. Paste the following code into the new script file. 

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

/// <summary>
///
/// </summary>
public class RosPublisherExample : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            cube.transform.rotation = Random.rotation;

            PosRotMsg cubePos = new PosRotMsg(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, cubePos);

            timeElapsed = 0;
        }
    }
}
```
Add a plane and a cube to your Unity scene. You can create simple geometric shapes in Unity by going to the Hierarchy window, clicking the + button, and navigating to the shape you want to create.

![create_cube](https://github.com/Kickblip/ROS-NGC/assets/54160850/ffedba52-14e0-440b-b1a9-829225ac0e1f)

Click the cube in your scene and use the vertical arrow to move it up until it is hovering above the plane

Create another **empty** GameObject, name it `RosPublisher` and attach the `RosPublisherExample` script by dragging the C# file onto the object.

Drag the cube GameObject onto the Cube parameter in the empty GameObject's inspection window.

Press play in the Editor. You should see the connection lights at the top left corner of the Game window turn blue, and something like `[INFO] [1622242057.562860400] [TCPServer]: Connection from 172.17.0.1` appear in the terminal running your server_endpoint.

## Echo the publisher
To make sure messages are being published from Unity correctly, run the following commands in the second tab of you Docker container (the one that is not running your server)
```bash
source install/setup.bash
ros2 topic echo pos_rot
```


