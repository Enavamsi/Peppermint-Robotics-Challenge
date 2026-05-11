# Copyright 2025 Your Organisation
#
# Licensed under the Apache License, Version 2.0

"""
simulation.launch.py
====================

Launches:

  • Gazebo Classic
  • Custom green sphere world
  • TurtleBot3 Burger
  • robot_state_publisher
  • color_nav visual servoing node

Usage
-----

export TURTLEBOT3_MODEL=burger

ros2 launch color_nav simulation.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # Package Directories
    # =========================================================

    pkg_color_nav = get_package_share_directory(
        "color_nav"
    )

    pkg_tb3_gazebo = get_package_share_directory(
        "turtlebot3_gazebo"
    )

    pkg_gazebo_ros = get_package_share_directory(
        "gazebo_ros"
    )

    # =========================================================
    # World File
    # =========================================================

    world_file = os.path.join(
        pkg_color_nav,
        "worlds",
        "green_sphere.world"
    )

    # =========================================================
    # Launch Arguments
    # =========================================================

    sim_args = [

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "x_pose",
            default_value="0.0"
        ),

        DeclareLaunchArgument(
            "y_pose",
            default_value="0.0"
        ),
    ]

    ctrl_args = [

        DeclareLaunchArgument(
            "kp",
            default_value="0.0025"
        ),

        DeclareLaunchArgument(
            "alignment_threshold",
            default_value="40"
        ),

        DeclareLaunchArgument(
            "search_speed",
            default_value="0.25"
        ),

        DeclareLaunchArgument(
            "forward_speed",
            default_value="0.05"
        ),

        DeclareLaunchArgument(
            "safe_distance",
            default_value="2.5"
        ),

        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true"
        ),
    ]

    # =========================================================
    # TurtleBot3 Environment Variable
    # =========================================================

    tb3_model_env = SetEnvironmentVariable(

        name="TURTLEBOT3_MODEL",

        value=EnvironmentVariable(
            "TURTLEBOT3_MODEL",
            default_value="burger"
        ),
    )

    # =========================================================
    # Gazebo Launch
    # =========================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gazebo.launch.py"
            )
        ),

        launch_arguments={

            "world": world_file,

            "verbose": "true",

        }.items(),
    )

    # =========================================================
    # Robot State Publisher
    # =========================================================

    robot_state_publisher = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                pkg_tb3_gazebo,
                "launch",
                "robot_state_publisher.launch.py"
            )
        )
    )

    # =========================================================
    # Spawn TurtleBot3
    # =========================================================

    spawn_tb3 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                pkg_tb3_gazebo,
                "launch",
                "spawn_turtlebot3.launch.py"
            )
        ),

        launch_arguments={

            "x_pose": LaunchConfiguration(
                "x_pose"
            ),

            "y_pose": LaunchConfiguration(
                "y_pose"
            ),

        }.items(),
    )

    # =========================================================
    # Visual Servoing Node
    # =========================================================

    visual_servoing_node = Node(

        package="color_nav",

        executable="visual_servoing",

        name="visual_servoing",

        output="screen",

        emulate_tty=True,

        parameters=[
            {
                "use_sim_time":
                    LaunchConfiguration(
                        "use_sim_time"
                    ),

                "kp":
                    LaunchConfiguration(
                        "kp"
                    ),

                "alignment_threshold":
                    LaunchConfiguration(
                        "alignment_threshold"
                    ),

                "search_speed":
                    LaunchConfiguration(
                        "search_speed"
                    ),

                "forward_speed":
                    LaunchConfiguration(
                        "forward_speed"
                    ),

                "safe_distance":
                    LaunchConfiguration(
                        "safe_distance"
                    ),

                "show_debug_window":
                    LaunchConfiguration(
                        "show_debug_window"
                    ),
            }
        ],
    )

    # =========================================================
    # Banner
    # =========================================================

    banner = LogInfo(

        msg=(

            "\n"

            "╔══════════════════════════════════════╗\n"

            "║      color_nav — Visual Servoing     ║\n"

            "║  Green sphere world  |  TurtleBot3   ║\n"

            "╚══════════════════════════════════════╝"
        )
    )

    # =========================================================
    # Launch Description
    # =========================================================

    return LaunchDescription(

        sim_args
        + ctrl_args
        + [

            banner,

            tb3_model_env,

            gazebo,

            robot_state_publisher,

            spawn_tb3,

            visual_servoing_node,
        ]
    )