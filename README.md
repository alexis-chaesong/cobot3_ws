# cobot3_ws

병원 서빙 협동로봇(M0609) ROS2 워크스페이스. Isaac Sim 기반 시뮬레이션 자산(assets)과 실제 제어 로직(control_algorithms 등)을 함께 관리합니다.

## 폴더 구조

```
cobot3_ws/
├── .gitignore
├── .gitattributes                # USD/STL/DAE는 Git LFS로 관리
├── README.md
├── build/ install/ log/          # colcon 빌드 산출물, git 추적 안 함
└── src/
    ├── hospital_bot_msgs/        # 인터페이스 정의서 기반 커스텀 메시지
    ├── perception/                # 찬용 — YOLO 비전
    ├── control_algorithms/        # 채송 — 슬로싱 궤적, 커버리지, 임피던스 로직
    │   └── control_algorithms/
    │       ├── slosh_shaping/     # Input Shaping + Min Jerk
    │       ├── coverage_eval/     # 커버리지 정량화
    │       └── impedance_ctrl/    # 임피던스 로직 골격
    ├── grasp_control/             # 범준 — Cloth attach/grasp 로직
    ├── integration/               # 태성 — TF, IK, 시퀀싱, Nova Carter 연동
    ├── ClothAsset/                # Cloth 에셋 생성 스크립트
    ├── doosan-robot2/             # 외부 공식 레포, 로컬 관리 (git 추적 안 함)
    ├── onrobot_rg2/               # 외부 레포, 로컬 관리 (git 추적 안 함)
    └── assets/                    # Isaac Sim 공유 자산
        ├── robots/                 # M0609 공유 USD (카메라/그리퍼 포함)
        ├── scenes/                 # Cloth, 유체, 테이블 등 씬 USD
        └── meshes/                 # stl/dae 등 메시
```

## 참고

- `build/`, `install/`, `log/`는 colcon 빌드 산출물이므로 git에 올리지 않습니다.
- `src/doosan-robot2/`, `src/onrobot_rg2/`는 외부 레포를 로컬에서만 관리하며 버전 추적하지 않습니다 (`.gitignore` 참고).
- `*.usd`, `*.usda`, `*.usdc`, `*.stl`, `*.dae`는 Git LFS로 관리됩니다. 클론 후 `git lfs install`을 먼저 실행하세요.
- Isaac Sim 실험용 개인 스크립트(`isaacpjt/` 등)는 공유 대상이 아니며 git에서 제외되어 있습니다.
