#!/usr/bin/env python3

import os
import signal
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # Define the launch arguments
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    # Define the commands to be run
    gazebo_launch_cmd = "ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py"
    nav2_launch_cmd = f"ros2 launch nav2_bringup bringup_launch.py map:='/home/hector/robot1/install/turtlebot3_manipulation_navigation2/share/turtlebot3_manipulation_navigation2/map/turtlebot3_world.yaml' use_sim_time:=False"
    rviz2_cmd = f"ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"

    # Run the commands
    gazebo_process = subprocess.Popen(gazebo_launch_cmd, shell=True)
    nav2_process = subprocess.Popen(nav2_launch_cmd, shell=True)
    rviz_process = subprocess.Popen(rviz2_cmd, shell=True)

    # Wait for the user to terminate the processes (Ctrl+C)
    try:
        # Use signal to handle interruption (Ctrl+C)
        gazebo_process.wait()
        nav2_process.wait()
        rviz_process.wait()
    except KeyboardInterrupt:
        # When Ctrl+C is pressed, terminate all processes
        print("Terminating processes...")
        gazebo_process.terminate()
        nav2_process.terminate()
        rviz_process.terminate()

        # Ensure processes are completely killed
        gazebo_process.wait()
        nav2_process.wait()
        rviz_process.wait()

    return LaunchDescription([
        LogInfo(
            condition=None,
            msg="Launching TurtleBot3 simulation with Gazebo, Navigation2, and RViz2..."
        ),
    ])
