# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# 멀티로봇(carter1=소독, carter2=폐기물) Nav2 런치 — modified_hospital 맵.
# multiple_robot_carter_navigation_hospital.launch.py 를 2로봇·새 맵·튜닝 params 로 적응.
#   robots=[carter1, carter2], map=maps/map/modified_hospital_map.yaml(공유),
#   params=params/modified_hospital/multi_robot_..._params_{1,2}.yaml (로봇별 initial_pose·네임스페이스 토픽)
# 실행:
#   ros2 launch carter_navigation multiple_robot_carter_navigation_modified_hospital.launch.py \
#     map:=<carter_navigation>/maps/map/modified_hospital_map.yaml

import json
import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    carter_nav2_bringup_dir = get_package_share_directory("carter_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    # [단일 RViz] 두 로봇을 한 창에 표시하는 통합 config (map 프레임 기준 표시:
    #  공유 Map + 로봇별 Amcl Swarm/Global Costmap/Path). RobotModel/LaserScan/Local costmap 은
    #  로봇별 tf 프레임(base_link/odom)이 겹쳐 한 RViz 에 못 합치므로 제외(ROS2 멀티로봇 공통 한계).
    rviz_config_dir = os.path.join(carter_nav2_bringup_dir, "rviz2", "carter_navigation_multi.rviz")

    # 로봇 2대 (carter1=소독, carter2=폐기물). 초기 pose 는 각 params_N 의 amcl.initial_pose 참조.
    robots = [{"name": "carter1"}, {"name": "carter2"}]

    # 공유 맵 (modified_hospital). maps/map/ 하위폴더에 있음.
    ENV_MAP_FILE = os.path.join("map", "modified_hospital_map.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml_file = LaunchConfiguration("map")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(carter_nav2_bringup_dir, "maps", ENV_MAP_FILE),
        description="Full path to map file to load",
    )

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "carter1_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "modified_hospital", "multi_robot_carter_navigation_params_1.yaml"
        ),
        description="Full path to the ROS2 parameters file for carter1 (소독)",
    )

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        "carter2_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "modified_hospital", "multi_robot_carter_navigation_params_2.yaml"
        ),
        description="Full path to the ROS2 parameters file for carter2 (폐기물)",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the stacks"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_dir, description="Full path to the RVIZ config file to use."
    )

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    # tf_relay(통합 RViz 용 전역 TF 공급) / initialpose 자동발행 토글 (14장, #4 일괄화)
    use_tf_relay = LaunchConfiguration("use_tf_relay")
    pub_initialpose = LaunchConfiguration("pub_initialpose")
    declare_use_tf_relay_cmd = DeclareLaunchArgument(
        "use_tf_relay", default_value="True",
        description="commander/tf_relay 를 로봇별로 띄워 /carterN/tf 를 전역 /tf 로 중계(통합 RViz 용).",
    )
    declare_pub_initialpose_cmd = DeclareLaunchArgument(
        "pub_initialpose", default_value="True",
        description="amcl 기동 후 각 로봇 initialpose 를 자동 발행(carter2 는 수동발행 필수라 자동화).",
    )

    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot["name"] + "_params_file")

        group = GroupAction(
            [
                # (per-robot RViz 제거 — 아래에서 통합 RViz 1개만 실행. 두 창 왔다갔다 방지)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(carter_nav2_bringup_dir, "launch", "carter_navigation_individual.launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": use_sim_time,
                        "params_file": params_file,
                        "default_bt_xml_filename": default_bt_xml_filename,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),
                # 3D 라이다(/carterN/front_3d_lidar/lidar_points) → 2D scan(/carterN/scan) 변환 (네임스페이스 내)
                Node(
                    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                    remappings=[('cloud_in', ['front_3d_lidar/lidar_points']),
                                ('scan', ['scan'])],
                    parameters=[{
                        'target_frame': 'front_3d_lidar',
                        'transform_tolerance': 0.01,
                        'min_height': -0.4,
                        'max_height': 1.5,
                        'angle_min': -1.5708,
                        'angle_max': 1.5708,
                        'angle_increment': 0.0087,
                        'scan_time': 0.3333,
                        'range_min': 0.05,
                        'range_max': 100.0,
                        'use_inf': True,
                        'inf_epsilon': 1.0,
                    }],
                    name='pointcloud_to_laserscan',
                    namespace=robot["name"],
                ),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot["name"]]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " map yaml: ", map_yaml_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " params yaml: ", params_file]),
            ]
        )
        nav_instances_cmds.append(group)

    # ── 통합 RViz 1개 (네임스페이스 없이 전역) : 두 로봇을 한 창에 표시 ──
    #   carter_navigation_multi.rviz 가 /carter1·/carter2 절대토픽을 직접 구독(Fixed Frame=map).
    single_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ── tf_relay ×2 (통합 RViz 용 전역 TF 중계, 14-3) ──
    #   /carterN/tf(_static) 를 프레임 접두(carterN/, map 은 공유) 붙여 전역 /tf 로 재발행.
    tf_relay_cmds = [
        Node(
            condition=IfCondition(use_tf_relay),
            package="commander", executable="tf_relay",
            name="tf_relay_" + robot["name"],
            parameters=[{"in_ns": robot["name"], "prefix": robot["name"]}],
            output="screen",
        )
        for robot in robots
    ]

    # ── pc_reframe ×2 (통합 RViz 용 포인트클라우드 프레임 접두 릴레이) ──
    #   Isaac 포인트클라우드 frame_id='front_3d_lidar'(무접두) → 'carterN/front_3d_lidar'(전역 /tf 매칭)
    #   로 바꿔 /carterN/front_3d_lidar/points_viz 로 재발행 → 통합 RViz PointCloud2 디스플레이가 표시.
    pc_reframe_cmds = [
        Node(
            condition=IfCondition(use_tf_relay),
            package="commander", executable="pc_reframe",
            name="pc_reframe_" + robot["name"],
            parameters=[{
                "prefix": robot["name"],
                "in_topic": "/" + robot["name"] + "/front_3d_lidar/lidar_points",
                "out_topic": "/" + robot["name"] + "/front_3d_lidar/points_viz",
            }],
            output="screen",
        )
        for robot in robots
    ]

    # ── initialpose 자동발행 (amcl 기동 대기 후, 14-6) ──
    #   좌표 = 각 params_N 의 amcl.initial_pose = Isaac 스폰(13번 CN_START_POSE)과 일치해야 함.
    #   단일 RViz 의 2D Pose Estimate 는 /carter1 만 찍혀 carter2 가 localize 못 하므로 명령으로 발행.
    #   ★1회만 발행(10s). 이전엔 12s·20s 두 번 발행했으나, 로봇이 이미 미션 주행 중일 때 두 번째
    #     발행이 amcl 을 홈으로 되돌려 위치가 튀는(오localize) 부작용이 있어 단발로 변경.
    #     (params set_initial_pose:true 가 amcl 활성화 시 자동초기화하므로 이 발행은 백업 성격.)
    # [2026-07-27 수정] 17_/19_ 의 "팔 베이스 도킹스테이션 중심 정렬 + 스폰방향 +Y 통일" 변경
    # (C1/C2_START_POSE y=+0.2317, yaw=90°) 이 여기 반영 안 돼 있어(y=0/yaw=0 그대로) 이 백업
    # 발행이 amcl.initial_pose 로 맞춰둔 값을 다시 틀어지게 만들던 문제 수정 — params_1/2.yaml
    # 의 initial_pose 와 반드시 동일해야 함.
    START_POSES = {"carter1": (18.5, 0.2317, 90.0), "carter2": (16.6629, 0.2287482072408726, 90.0)}

    def _initialpose_proc(ns, x, y, yaw_deg=0.0):
        # 중괄호 수동 이스케이프는 실수 나기 쉬움 → dict 를 json.dumps(=유효한 YAML) 로 안전 생성.
        yaw_rad = math.radians(yaw_deg)
        msg = json.dumps({
            "header": {"frame_id": "map"},
            "pose": {"pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"z": math.sin(yaw_rad / 2.0), "w": math.cos(yaw_rad / 2.0)},
            }},
        })
        return ExecuteProcess(
            condition=IfCondition(pub_initialpose),
            cmd=["ros2", "topic", "pub", "--once",
                 f"/{ns}/initialpose",
                 "geometry_msgs/msg/PoseWithCovarianceStamped", msg],
            output="screen",
        )

    initialpose_cmds = [
        TimerAction(
            period=10.0,
            actions=[_initialpose_proc(name, x, y, yaw) for name, (x, y, yaw) in START_POSES.items()],
        )
    ]

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_tf_relay_cmd)
    ld.add_action(declare_pub_initialpose_cmd)
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)
    ld.add_action(single_rviz_cmd)     # 통합 RViz (per-robot RViz 대체)
    for tf_relay_cmd in tf_relay_cmds:  # 전역 TF 중계 (통합 RViz 용)
        ld.add_action(tf_relay_cmd)
    for pc_reframe_cmd in pc_reframe_cmds:  # 포인트클라우드 프레임 접두 릴레이 (통합 RViz 용)
        ld.add_action(pc_reframe_cmd)
    for initialpose_cmd in initialpose_cmds:  # amcl 기동 후 initialpose 자동발행
        ld.add_action(initialpose_cmd)
    return ld
