#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner", controller],
                shell=True,
                output="screen",
                parameters=[{'turtlebot3_manipulation_moveit_config': os.path.join(get_package_share_directory('turtlebot3_manipulation_moveit_config'), 'config', 'joint_limits.yaml')}]

            )
        )

    for controller_action in load_controllers:
        ld.add_action(controller_action)

    return ld
