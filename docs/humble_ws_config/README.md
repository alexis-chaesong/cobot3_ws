# humble_ws 설정 백업 (참고용 사본)

`~/IsaacSim-ros_workspaces/humble_ws`는 git remote가 이 팀 저장소가 아니라 NVIDIA 공식
`isaac-sim/IsaacSim-ros_workspaces`라 PR을 낼 수 없다. 아래 파일들은 그 워크스페이스에서
`19_dual_task_select_yolo_integrated.py` 실행에 필요한데 그쪽 git으로는 전혀 추적이 안 되던
파일들이라, 유실 방지용으로 이 레포에 참고 사본만 보관한다(루트 [README.md](../../README.md)
⑥번 항목 참고).

**이 폴더 자체를 실행에 쓰지 않는다** — 새 머신/새 클론에서 humble_ws를 채울 때, 아래처럼 원래
경로로 복사해 넣어야 한다(`src/`와 `install/share/` 둘 다):

```bash
HUMBLE=~/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation
HUMBLE_INSTALL=~/IsaacSim-ros_workspaces/humble_ws/install/carter_navigation/share/carter_navigation
BAK=~/cobot3_ws/docs/humble_ws_config

cp "$BAK/launch/multiple_robot_carter_navigation_modified_hospital.launch.py" "$HUMBLE/launch/"
cp "$BAK/params/modified_hospital/"*.yaml "$HUMBLE/params/modified_hospital/"
cp "$BAK/rviz2/carter_navigation_multi.rviz" "$HUMBLE/rviz2/"

# install/share 쪽도 동일하게 (또는 humble_ws 에서 colcon build --packages-select carter_navigation)
cp "$BAK/launch/multiple_robot_carter_navigation_modified_hospital.launch.py" "$HUMBLE_INSTALL/launch/"
cp "$BAK/params/modified_hospital/"*.yaml "$HUMBLE_INSTALL/params/modified_hospital/"
cp "$BAK/rviz2/carter_navigation_multi.rviz" "$HUMBLE_INSTALL/rviz2/"
```

맵 파일(`modified_hospital_2_map.yaml`/`.png`)은 여기 없다 — 그건 이미
`src/assets/map/`(레포 루트 기준)에 정식으로 git 추적되고 있어서 중복 백업하지 않았다.

⚠ **이 사본은 스냅샷이다.** humble_ws 쪽에서 params/launch/rviz를 다시 튜닝하면 이 폴더는
자동으로 안 따라가므로, 값이 안 맞으면 실제 실행 중인 머신의 파일을 기준으로 다시 복사해와야
한다.
