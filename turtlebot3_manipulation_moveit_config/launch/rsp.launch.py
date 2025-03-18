import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Get the path to the kinematics.yaml file
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_manipulation_moveit_config"), "config", "kinematics.yaml"]
    )
    
    robot_description_kinematics = {
        "robot_description_kinematics.arm.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",  # Example solver (KDL)
        "robot_description_kinematics.arm.kinematics_solver_search_resolution": 0.005,  # Example resolution
        "robot_description_kinematics.arm.kinematics_solver_timeout": 0.05,  # Example timeout
        "robot_description_kinematics.arm.position_only_ik": True,
        "robot_description_kinematics.gripper.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",  # Gripper solver
        "robot_description_kinematics.gripper.kinematics_solver_search_resolution": 0.005,  # Gripper search resolution
        "robot_description_kinematics.gripper.kinematics_solver_timeout": 0.05  # Gripper timeout
        }
# Return the LaunchDescription
    return LaunchDescription([
        Node(
            package='turtlebot3_manipulation_moveit_config',
            executable='motion',
            #name='motion_u',
            output='screen',
            parameters=[
                robot_description_kinematics,
                {'robot_description_kinematics': robot_description_kinematics},
                {'use_sim_time': True}
            ]
        ),
    ])
