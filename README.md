# ROS (Robot Operating System) Basic Workshop Tutorial

## Introduction

Welcome to the basic ROS workshop! This tutorial is designed to introduce you to the fundamental concepts of ROS, along with practical hands-on exercises using the Turtlesim package. By the end of this workshop, you will have a basic understanding of ROS architecture, its commands, and how to control a simulated turtle in Turtlesim.

## Prerequisites

- Basic knowledge of Linux commands
- Ubuntu (preferably 18.04 or 20.04)
- ROS Noetic installed on your machine

## Workshop Outline

1. Introduction to ROS
2. ROS Concepts
    - ROS Master
    - ROS Nodes
    - ROS Topics
    - ROS Services
    - ROS Actions
3. Basic ROS Commands
4. Working with Turtlesim
5. Conclusion

---

## 1. Introduction to ROS

### What is ROS?

ROS (Robot Operating System) is an open-source, flexible framework for writing robot software. It provides tools, libraries, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

---

## 2. ROS Concepts

### ROS Master

The ROS Master provides naming and registration services to the rest of the ROS system. It enables nodes to locate each other and manage communications.

### ROS Nodes

Nodes are the basic processes that perform computation in ROS. Each node is designed to execute a specific task, and nodes can communicate with each other to build a complete robotic system.

### ROS Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, and other nodes can subscribe to that topic to receive those messages. This communication method is asynchronous.

### ROS Services

Services in ROS provide a synchronous communication mechanism between nodes. A service consists of a request and a response. A node can call a service to request a specific action, and the service will respond once the action is complete.

### ROS Actions

Actions in ROS are designed for tasks that take an extended period and can be preempted. They provide a way to send goals to a server, receive feedback during execution, and get a result when the task is completed. Actions are particularly useful for long-running or continuous tasks.

---

## 3. Basic ROS Commands

### Setting Up ROS Environment

```bash
source /opt/ros/noetic/setup.bash
```

### Creating a ROS Workspace

1. Create a directory for your workspace:
    ```bash
    mkdir -p ~/ros_workspace/src
    ```

2. Initialize the workspace:
    ```bash
    cd ~/ros_workspace/src
    catkin_init_workspace
    ```

3. Build the workspace:
    ```bash
    cd ~/ros_workspace
    catkin_make
    ```

4. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

### Basic ROS Commands

- **Starting ROS Master**: `roscore`
- **Creating a New Package**: `catkin_create_pkg <package_name> std_msgs rospy roscpp`
- **Listing ROS Nodes**: `rosnode list`
- **Displaying Node Information**: `rosnode info /node_name`
- **Listing ROS Topics**: `rostopic list`
- **Displaying Topic Information**: `rostopic info /topic_name`
- **Publishing to a Topic**: `rostopic pub /topic_name std_msgs/String "data: 'Hello, ROS!'"`
- **Subscribing to a Topic**: `rostopic echo /topic_name`
- **Running a ROS Node**: `rosrun <package_name> <node_name>`

---

## 4. Working with Turtlesim

### Installing Turtlesim

```bash
sudo apt-get install ros-noetic-turtlesim
```

### Launching Turtlesim

1. Open a terminal and start the ROS Master:
    ```bash
    roscore
    ```

2. In a new terminal, launch Turtlesim:
    ```bash
    rosrun turtlesim turtlesim_node
    ```

3. In another terminal, open the Turtlesim Teleop Key:
    ```bash
    rosrun turtlesim turtle_teleop_key
    ```

### Demonstrating ROS Concepts with Turtlesim

#### ROS Nodes

- **List nodes**:
    ```bash
    rosnode list
    ```
  This will show you the nodes that are currently running, including `/rosout` and `/turtlesim`.

- **Node information**:
    ```bash
    rosnode info /turtlesim
    ```
  This command provides information about the `/turtlesim` node, such as the topics it publishes and subscribes to.

#### ROS Topics

- **List topics**:
    ```bash
    rostopic list
    ```
  This will display all the active topics, including `/turtle1/cmd_vel` and `/turtle1/pose`.

- **Topic information**:
    ```bash
    rostopic info /turtle1/pose
    ```
  This command shows details about the `/turtle1/pose` topic, including the type of messages and the nodes publishing and subscribing to it.

- **Echo topic**:
    ```bash
    rostopic echo /turtle1/pose
    ```
  This command prints messages being published to the `/turtle1/pose` topic, showing the turtle's position in real-time.

#### ROS Services

- **List services**:
    ```bash
    rosservice list
    ```
  This lists all available services, such as `/clear` and `/spawn`.

- **Service information**:
    ```bash
    rosservice info /spawn
    ```
  This provides information about the `/spawn` service.

- **Call service**:
    ```bash
    rosservice call /spawn 2 2 0.2 "turtle2"
    ```
  This spawns a new turtle at the specified coordinates with the name "turtle2".

#### ROS Actions

- **List actions**: There are no default actions with Turtlesim, but you can explore actions with other ROS packages that support actions, such as the navigation stack.
- **Action information**: To understand actions, you typically need a package that defines action servers and clients. Actions involve more complex interactions than services, suitable for tasks that provide feedback and can be preempted.

### Writing a Simple ROS Node for Turtlesim

1. **Create a new package**: Follow the instructions to create a package and set up your workspace.

2. **Create a Python script**: Write a script to move the turtle in a pattern using the `geometry_msgs/Twist` message type.

3. **Run the script**: Execute the script using `rosrun` and observe the turtle's movements in the Turtlesim window.

---

## Conclusion

Congratulations! You have completed the basic ROS workshop. You now have a foundational understanding of ROS concepts, commands, and how to work with the Turtlesim package. Additionally, you have learned about ROS services, actions, and the role of the ROS Master in managing communications. Keep exploring ROS to build more complex and exciting robotic applications.

Happy coding!



# TurtleBot3 Simulation Workshop Tutorial

## Introduction

Welcome to the TurtleBot3 simulation workshop! This tutorial will guide you through setting up and using TurtleBot3 in a simulated environment using RViz and Gazebo. You will learn how to perform basic movements, SLAM (Simultaneous Localization and Mapping), save maps, and navigate autonomously.

## Prerequisites

- Basic knowledge of Linux commands
- Ubuntu (preferably 18.04 or 20.04)
- ROS Noetic installed on your machine
- TurtleBot3 packages installed

## Workshop Outline

1. Introduction to TurtleBot3
2. Basic Setup
3. RViz Simulation
4. Gazebo Simulation
    - Empty World
    - Predefined World
    - Teleoperation
5. SLAM
6. Map Saving
7. Autonomous Navigation

---

## 1. Introduction to TurtleBot3

### What is TurtleBot3?

TurtleBot3 is a low-cost, personal robot kit with open-source software. It is designed to support education, research, and product prototyping.

---

## 2. Basic Setup

### Installing TurtleBot3 Packages

1. Install TurtleBot3 packages:
    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
    ```

2. Set the TurtleBot3 model environment variable:
    ```bash
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
    ```

---

## 3. RViz Simulation

### Launching TurtleBot3 in RViz

1. Open a terminal and start the ROS Master:
    ```bash
    roscore
    ```

2. In a new terminal, launch the TurtleBot3 simulation in RViz:
    ```bash
    roslaunch turtlebot3_fake turtlebot3_fake.launch
    ```

3. RViz will start, showing the TurtleBot3 model. You can use RViz to visualize different sensor data and robot states.

---

## 4. Gazebo Simulation

### Launching TurtleBot3 in an Empty World

1. Open a terminal and launch Gazebo with an empty world:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

### Launching TurtleBot3 in a Predefined World

1. Open a terminal and launch Gazebo with a predefined world:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

### Teleoperation in Gazebo

1. Open a terminal and run the teleop node to control TurtleBot3:
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
2. Use the keyboard to move the robot around the simulation.

---

## 5. SLAM (Simultaneous Localization and Mapping)

### Running SLAM with TurtleBot3

1. Open a terminal and launch the TurtleBot3 in a Gazebo world:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

2. In a new terminal, start the SLAM node:
    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    ```

3. In another terminal, run the teleop node to control the robot and explore the environment:
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```

4. Move the robot around to create a map in RViz.

---

## 6. Map Saving

### Saving the Map

1. Once you are satisfied with the map created, save it using the following command in a new terminal:
    ```bash
    rosrun map_server map_saver -f ~/map
    ```

2. This will save the map as `map.pgm` and `map.yaml` in your home directory.

---

## 7. Autonomous Navigation

### Running Autonomous Navigation

1. Launch the TurtleBot3 in a Gazebo world:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

2. Launch the navigation stack with your saved map:
    ```bash
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
    ```

3. Use RViz to set an initial pose estimate:
    - Click on the "2D Pose Estimate" button and click on the map to set the initial pose.

4. Use RViz to set a goal for autonomous navigation:
    - Click on the "2D Nav Goal" button and click on the map to set the target destination.

---

## Conclusion

Congratulations! You have completed the TurtleBot3 simulation workshop. You now have a foundational understanding of simulating TurtleBot3 using RViz and Gazebo, performing SLAM, saving maps, and navigating autonomously. Keep exploring and building upon these skills to create more advanced robotic applications.

Happy coding!
