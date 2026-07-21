#!/usr/bin/env python3
"""
costmap_check.launch.py
============================================================================
작업 0 검증 : 맵 없이 라이다 /scan 만으로 복도 벽이 local costmap 에
장애물로 잡히는지 확인하는 최소 구성.

띄우는 노드
  1) nav2_costmap_2d  (독립 costmap, 노드명 "costmap") ← local_costmap_check.yaml
  2) nav2_lifecycle_manager (costmap 을 autostart 로 configure+activate)
  3) (옵션) base_link → 라이다 프레임 static TF  : Isaac 이 이 TF 를 안 내보낼 때만
  4) (옵션) RViz2

사전조건 (Isaac 쪽에서 발행되어야 함)
  - /scan            (LaserScan)      : rplidar_ROS.usd 의 ROS2 publish 그래프
  - TF odom→base_link                 : Nova Carter odom 그래프
  - TF base_link→<scan frame_id>       : 로봇/라이다 TF (없으면 아래 static_tf 사용)
  - /clock                            : ROS2 bridge Clock (use_sim_time=true 대응)

실행 (colcon 빌드 불필요, 절대경로 launch)
  source /opt/ros/humble/setup.bash
  ros2 launch /home/rokey/cobot3_ws/src/integration/integration/nav2_check/costmap_check.launch.py

  # 라이다 TF 가 Isaac 에서 안 나오면 static TF 를 켜서 실행 :
  ros2 launch .../costmap_check.launch.py use_static_lidar_tf:=true \
       lidar_frame:=front_2d_lidar lidar_xyz:="0 0 0.3"

  # RViz 같이 :
  ros2 launch .../costmap_check.launch.py rviz:=true

성공 판정
  RViz 의 Map 디스플레이(topic /costmap/costmap)에서 복도 벽 라인이
  장애물(진한 셀)로 그려지면 통과. (또는 ros2 topic echo /costmap/costmap 로
  data 에 100/254 셀이 생기는지 확인)
============================================================================
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_HERE = Path(__file__).resolve().parent
PARAMS = str(_HERE / "local_costmap_check.yaml")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_static_tf = LaunchConfiguration("use_static_lidar_tf")
    lidar_frame = LaunchConfiguration("lidar_frame")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    use_rviz = LaunchConfiguration("rviz")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_static_lidar_tf", default_value="false",
                              description="보통 불필요(Carter 가 base_link→front_2d_lidar TF 발행). "
                                          "TF 가 안 나올 때만 true"),
        DeclareLaunchArgument("lidar_frame", default_value="front_2d_lidar",
                              description="/front_2d_lidar/scan header.frame_id 와 일치"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.3",
                              description="base_link 기준 라이다 높이 [m]"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    costmap_node = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        output="screen",
        parameters=[PARAMS, {"use_sim_time": use_sim_time}],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_check",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["costmap"],
        }],
    )

    # base_link → 라이다 프레임 static TF (Isaac 에서 안 나올 때만 사용)
    static_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_lidar",
        output="screen",
        condition=IfCondition(use_static_tf),
        arguments=[
            "--x", lidar_x, "--y", lidar_y, "--z", lidar_z,
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link", "--child-frame-id", lidar_frame,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_costmap_check",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [costmap_node, lifecycle, static_lidar_tf, rviz])
