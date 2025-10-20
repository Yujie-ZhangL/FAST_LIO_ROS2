#!/usr/bin/env python3
"""
Launch-Datei für den FAST-LIO Lokalisierungsalgorithmus.

Dieses Launch-File startet den Knoten `fastlio_mapping`, der LiDAR- und IMU-Daten
vom angegebenen Topic empfängt und daraus die Fahrzeugodometrie berechnet.

Parameter:
  - input_topic:   Eingehendes LiDAR-Topic (z.B. /velodyne_points_noisy)
  - imu_topic:     Eingehendes IMU-Topic (z.B. /imu/data)
  - config_file:   Pfad zur FAST-LIO Konfigurationsdatei (z.B. velodyne.yaml)
  - visualize:     Ob RViz gestartet werden soll
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    fastlio_pkg = get_package_share_directory("fast_lio")
    default_config = os.path.join(fastlio_pkg, "config", "velodyne.yaml")
    default_rviz = os.path.join(fastlio_pkg, "rviz", "fastlio.rviz")

    input_topic = LaunchConfiguration("input_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    config_file = LaunchConfiguration("config_file", default=default_config)
    visualize = LaunchConfiguration("visualize", default="true")

    declare_input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/velodyne_points_noisy",
        description="Eingangs-Topic für LiDAR-Punktwolken"
    )

    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="Eingangs-Topic für IMU-Daten"
    )

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Pfad zur FAST-LIO YAML-Konfigurationsdatei"
    )

    declare_visualize = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Starte RViz zur Visualisierung"
    )

    fastlio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laserMapping",
        output="screen",
        remappings=[
            ("velodyne_points", input_topic),
            ("imu/data", imu_topic),
        ],
        parameters=[
            config_file
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz],
        condition=IfCondition(visualize),
    )

    return LaunchDescription([
        declare_input_topic,
        declare_imu_topic,
        declare_config_file,
        declare_visualize,
        fastlio_node,
        rviz_node,
    ])
