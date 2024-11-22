# Turtlesim "Catch Them All" Project

This project was built to understand the basics of **ROS2 (Robot Operating System)** while building a fully functioning application. Using the **Turtlesim package**, I simulate a robot (a turtle) that navigates the screen, "catches" other turtles, and interacts with custom nodes and messages. The primary goal is to learn ROS2 concepts such as nodes, topics, services, and custom interfaces through a hands-on project.

## **Why This Project?**

The **Turtlesim "Catch Them All" Project** was created as a learning exercise to:
- Understand the fundamentals of ROS2, including node communication and message handling.
- Gain practical experience in creating and managing custom nodes, services, and messages.
- Explore how to design and scale a small robot application.
- Learn to use simulation tools like Turtlesim for robot visualization.

## **What I Learned**

1. **ROS2 Basics**:
   - How nodes communicate using topics, services, and parameters.
   - How to design a scalable ROS application by separating functionalities into multiple nodes.

2. **Custom Interfaces**:
   - Creating custom message types (e.g., `Turtle.msg`, `TurtleArray.msg`) for specific data exchange.
   - Implementing custom service types (e.g., `CatchTurtle.srv`) for targeted interactions between nodes.

3. **Node Design**:
   - **Turtle Controller Node**: Implements a control loop to navigate the master turtle to target other turtles on the screen.
   - **Turtle Spawner Node**: Manages spawning, tracking, and removal of turtles using services.

4. **ROS2 Simulation Tools**:
   - Utilizing the Turtlesim package to visualize robot behavior.
   - Integrating simulation time and parameters to improve application behavior.

5. **Launch Files and Parameters** (NOT CONFIGURED YET):
   - Automating node launches with preconfigured parameters to simplify execution.

## **How to Run the Project**

### **Dependencies**
1. ROS2 installed on your system.
2. Turtlesim package (already included in ROS2).
3. The `turtlesim_catch_them_all` and `my_robot_interfaces` ROS2 packages.

### **Steps to Run**
1. **Build the Workspace**:
   Navigate to your workspace root (`ros2_ws`) and build it:
   ```bash
   colcon build
   source install/setup.bash```
2. **Launch the application** (NOT CURRENTLY IMPLEMENTED):
   Use the launch file to start all nodes with configured paramters
   ```ros2 launch my_robot_bringup catch_them_all.launch.py
   
