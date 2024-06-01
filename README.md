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
