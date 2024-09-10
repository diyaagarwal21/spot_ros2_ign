# quadruped_ros2

Adaption of champ quadruped robot to ROS2 and Ignition Gazebo. Previously, the Boston Dynamics Spot robot was only simulated using ROS2 with Gazebo Classic, which goes end-of-life in January 2025. This package, modified from quadruped_ros2, simulates the Spot robot in Ignition Gazebo, a newer version of Gazebo. This combination of simulating the Spot robot with ROS2 and Ignition Gazebo on an open-source platform has not been done yet.

## Challenge
- Challenge Name: NASA Space ROS Sim Summer Sprint Challenge 
- Challenge Participant: Diya Agarwal
- Team Lead Freelancer User Name: diyaagar
- Submission Title: Simulation of the Boston Dynamics Spot robot in ROS2 and Ignition Gazebo

## Package Information
- `champ`: includes configurations for the quadruped's kinematics/movement
- `champ_base`: includes configurations for the quadruped's specific state and controllers
- `champ_bringup`: includes the controller yaml files and launch files
- `champ_description`: contains meshes model configs for three robots (inluding Spot)
- `champ_gazebo`: contains world/environment configs
- `champ_msgs`: contains info about robot messages

## How to Install (Docker info)

1. Navigate to your workspace (my_ws)

2. Build the Docker Image
```
chmod +x build.sh
./build.sh
```

3. Run the container
```
chmod +x run.sh
./run.sh
```

## How to run
1. Source your ROS2 distribution. 
```
source /opt/ros/${ROS_DISTRO}/setup.bash
```

2. Source your workspace.
```
source ~/my_ws/install/setup.bash
```

3. Run the launch file.
```
ros2 launch champ_bringup champ_bringup.launch.py
```
With this, Ignition Gazebo and RViz will both launch.

## How to change worlds/environments
- In the launch file (champ_bringup.launch.py), there are multiple world configurations to change between. When launching the Ignition Gazebo world, simply change the name of file path.

How to add a new environment:
- In champ_gazebo/worlds, add a .sdf file along with the necessary meshes with the correct path directory.
- In the launch file, add a file path to your .sdf and launch it with Ignition Gazebo.

## Examples
For different environments, see last section.

1. Spot Robot in ground plane (simple surface, just a plane)
2. Spot Robot in rocky lunar surface (has varying heights)

## Details of Configurations
To see the topics to configure the controllers, run the following commands:

This shows the links/joints of the robot.
`
ros2 run tf2_tools view_frames
```

This shows the nodes/topics to send information to make the robot walk.
```
ros2 run rqt_graph rqt_graph
```
(Typically use the Twist command with /vox_nav/cmd_vel topic configure controllers)

## Contributions
Modified from https://github.com/jediofgever/quadruped_ros2.

Contributor is Diya Agarwal, an undergraduate student at Caltech. This research was conducted at Florida International University's Applied Research Center, under Dr. Dwayne McDaniel.