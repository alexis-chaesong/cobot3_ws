# cobot3_ws

## cobot3_ws의 navigation 디렉토리를 엔비디아의 IsaacSim-ros_workspaces를 git clone 한 뒤, 
## /home/rokey/IsaacSim-ros_workspaces/humble_ws/src 경로의 navigation을 대체하여 넣어주시면 됩니다.
## 프로젝트 과정에서 map 경로가 이 navigation에 있어 안내해드립니다.

## cobot3_ws의 install에 commander 디렉토리가 필요하여 이는 삭제하지 않고 제출합니다. 

병원 서빙 협동로봇(M0609) ROS2 워크스페이스. Isaac Sim 기반 시뮬레이션 자산(assets)과 실제 제어 로직(control_algorithms 등)을 함께 관리합니다.

**저장소를 받는 방법(`git clone` / GitHub "Code → Download ZIP" 둘 다 동일하게 동작)**: 자산은 일반
git blob으로 저장되어 있어 별도 `git lfs pull` 없이 받은 그대로 바로 씁니다. 이후 셋업은 아래
"새 머신에서 받았을 때 (로컬 환경 셋업)" 절을 따르세요.

## 📌 폴더 구조

```
cobot3_ws/
├── .gitignore
├── .gitattributes
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
    ├── doosan-robot2/             # m0609 URDF/USD 변환 산출물 (git 추적, 받은 후 mesh 경로 재작성 필요)
    ├── onrobot_rg2/               # RG2 그리퍼 URDF/mesh (git 추적, 받은 후 mesh 경로 재작성 필요)
    └── assets/                    # Isaac Sim 공유 자산
        ├── robots/                 # M0609 공유 USD (카메라/그리퍼 포함)
        ├── scenes/                 # Cloth, 유체, 테이블 등 씬 USD
        └── meshes/                 # stl/dae 등 메시
```

## 📌 참고

- `build/`, `install/`, `log/`는 colcon 빌드 산출물이므로 git에 올리지 않습니다.
- `src/doosan-robot2/`, `src/onrobot_rg2/`는 m0609/그리퍼용 URDF+USD 변환 산출물이라 git에 포함되어 있습니다(공식 doosan-robot2 레포 전체가 아님). 단, 안에 baked된 mesh 절대경로 때문에 받은 후 `./fix_doosan_mesh_paths.sh`를 한 번 실행해야 합니다 — 아래 "새 머신에서 받았을 때" ③ 참고.
- `*.usd`, `*.usda`, `*.usdc`, `*.stl`, `*.dae`, `*.png`, `*.mdl`은 일반 git blob으로 커밋되어 있습니다 (Git LFS 아님 — GitHub의 "Download ZIP"이 LFS smudge를 실행하지 않아 zip으로 받으면 자산이 깨지는 문제가 있어 되돌림). `git lfs install` 같은 별도 설정 불필요.
- `isaacpjt/` (Isaac Sim 시뮬레이션 스크립트, `M0609/13_~19_...`)는 실제 실행되는 메인 코드라 git에 포함되어 있습니다.

## 📌 새 머신에서 받았을 때 (로컬 환경 셋업)

`git clone`이든 zip 다운로드든 받은 그대로 `colcon build`는 되지만, 그것만으로는 Isaac 시뮬레이션
스크립트(`isaacpjt/M0609/13_,15_,16_...`)가 바로 안 돌아갑니다. 아래를 순서대로 확인하세요.

**① 레포 밖에서 따로 설치해야 하는 것 (레포와 무관, 한 번만)**
- Isaac Sim 5.1 본체 — 공식 배포판을 홈 디렉토리 등 원하는 곳에 설치. `run_isaac*.sh`는
  `~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release`를 기본 경로로 가정하므로,
  다른 곳에 설치했다면 `run_isaac*.sh` 안의 `ISAAC=` 줄만 바꾸면 됩니다.
- ROS2 Humble 시스템 설치(`/opt/ros/humble`).
- carter_navigation 패키지가 들어있는 **별도 워크스페이스**
  (`~/IsaacSim-ros_workspaces/humble_ws`, NVIDIA Isaac Sim ROS2 워크스페이스) — Nav2 자율주행에
  필수. 다른 경로에 뒀다면 Isaac 스크립트 실행 전 `export CARTER_NAV_WS=/그경로/humble_ws`.
- (선택) 원격 노트북에서 Isaac Sim 화면만 보고 싶다면 NVIDIA 공식 사이트에서
  "Isaac Sim WebRTC Streaming Client" 설치 → Isaac 스크립트를 `LIVESTREAM=1`로 실행.

**② pip로 추가 설치해야 하는 것 (Isaac Sim 내장 Python 대상)**
```bash
isaac_python -m pip install -r src/perception/requirements.txt
```
`ultralytics`(YOLO)가 없어서 필요합니다. **numpy 버전을 절대 건드리지 마세요** —
그냥 `pip install ultralytics`만 하면 numpy가 2.x로 자동 승급되면서 Isaac 코어 패키지(numba 등)와
충돌합니다. 위 requirements.txt가 `numpy==1.26.0`으로 고정해줍니다.

**③ `src/doosan-robot2/`, `src/onrobot_rg2/` — 받은 후 mesh 경로 재작성 필수**
- 2026-07-29부터 이 두 폴더는 레포에 포함되어 있습니다(예전엔 "외부 공식 레포, 각자 복사해오기"로
  안내했으나, 실제로는 doosan-robot2 공식 레포 전체가 아니라 `urdf/`+`usd/`뿐인 이 프로젝트 전용
  m0609 변환 산출물이라 git 추적으로 바꿨습니다 — 더 이상 팀원에게 따로 구할 필요 없음).
- **주의**: 여러 URDF(`doosan-robot2/urdf/m0609_isaac_sim.urdf`,
  `doosan-robot2/urdf/m0609_with_nozzle.urdf`, `src/assets/robots/m0609_with_gripper.urdf`,
  `isaacpjt/M0609/rmpflow/m0609_isaac_sim.urdf`)의 `<mesh filename="...">`가 ROS `package://`가
  아니라 **절대경로**로 박혀 있어서, 마지막으로 이 경로를 재작성한 머신/계정 경로가 그대로 남아있으면
  다른 머신에서는 메시가 안 열립니다(Isaac URDF 임포터가 조용히 스킵). **받은 뒤 반드시 한 번**:
  ```bash
  ./fix_doosan_mesh_paths.sh
  ```
  를 실행해 절대경로를 이 머신의 실제 위치로 재작성하세요(계정/경로 무관하게 동작, 여러 번 실행해도
  안전).

**④ 실행 방식(코드 아님, `run_isaac*.sh`가 이미 처리)**
- Isaac Sim 내장 Python(3.11)에 rclpy를 물리려면 `isaacsim.ros2.bridge` extension의
  `LD_LIBRARY_PATH`가 잡혀 있어야 합니다(안 그러면 extension startup 실패 → `import rclpy`도
  연쇄로 실패). 시스템 ROS(3.10)를 직접 source하면 오히려 충돌해서 크래시 나니 하지 마세요.
  `run_isaac.sh`/`run_isaac_single.sh`/`run_isaac_dual.sh`가 이걸 대신 세팅해주니 이 스크립트로
  실행하세요. 옵션: `ISAAC_HEADLESS=1`(창 없이), `LIVESTREAM=1`(WebRTC 원격 스트리밍).
- GPU 전력 캡, VPN/Meshnet 같은 머신 인프라 설정은 레포 범위 밖이라 각자 환경에 맞게 알아서
  설정하면 됩니다.

**⑤ 소독 노즐 커스텀 USD 에셋 (해결됨)**
- `src/doosan-robot2/urdf/m0609_with_nozzle/m0609_with_nozzle.usd` (소독 노즐 커스텀 USD 에셋)도
  ③에서 설명한 doosan-robot2 산출물의 일부라 저장소를 받으면 바로 존재합니다. `13_`(carter1
  소독팔)·`15_`·`16_`의 툴체인저 로직에 필요.

**⑥ `19_dual_task_select_yolo_integrated.py` 실행을 위한 `humble_ws` 쪽 설정**

`humble_ws`(`~/IsaacSim-ros_workspaces/humble_ws`)는 `git remote`가 이 팀 저장소가 아니라
NVIDIA 공식 `isaac-sim/IsaacSim-ros_workspaces`를 가리키고 있어서, 거기에 PR을 낼 수 없습니다.
아래 파일들은 그 워크스페이스 자체 git으로는 추적이 안 되던(전부 untracked) 상태였는데, 유실
방지용 참고 사본을 [`docs/humble_ws_config/`](docs/humble_ws_config/)에 넣어뒀습니다. 새
머신이거나 `humble_ws`를 새로 클론했다면 이 폴더에서 원래 경로로 복사해 넣으면 됩니다
(복사 명령은 [`docs/humble_ws_config/README.md`](docs/humble_ws_config/README.md) 참고).
`carter_navigation` 패키지 특성상 **`src/`와 `install/share/`에 동일하게 있어야** `ros2 launch`가
즉시 반영합니다(하나만 바꾸면 안 됨).

- `carter_navigation/maps/map/modified_hospital_2_map.yaml` + `.png`
  → 출처 = 이 레포의 `src/assets/map/modified_hospital_2_map.yaml`/`.png`(이미 git 추적됨,
  그대로 복사, `docs/`에 중복 백업 안 함). `.yaml`의 `image:` 필드가 상대경로
  (`modified_hospital_2_map.png`)라 같은 폴더에 두 파일 다 있어야 합니다(18-3 트러블슈팅 —
  이거 하나 빠져서 맵 자체가 안 뜬 적 있음).
- `carter_navigation/launch/multiple_robot_carter_navigation_modified_hospital.launch.py`
  → 멀티로봇(carter1/carter2) Nav2 + 통합 RViz + tf_relay×2 + initialpose 자동발행을 한 번에
  띄우는 launch 파일. `run_nav.sh`가 이걸 호출합니다. 백업: `docs/humble_ws_config/launch/`.
- `carter_navigation/params/modified_hospital/multi_robot_carter_navigation_params_1.yaml`
  (carter1용) / `..._params_2.yaml`(carter2용)
  → AMCL/DWB/costmap 튜닝값 + `/carterN/` 토픽 접두 + `initial_pose`. carter1은
  `(18.5, 0.2317, yaw 90°)`, carter2는 `(16.6629, 0.2287, yaw 90°)`로 19_의 스폰좌표와
  일치해야 함(안 맞으면 AMCL이 처음부터 틀어짐 — 18-2 참고). 백업:
  `docs/humble_ws_config/params/modified_hospital/`.
- `carter_navigation/rviz2/carter_navigation_multi.rviz`
  → 통합 RViz 설정(맵 공유 + 로봇별 Amcl/Costmap/Path + TF는 `carter1/base_link`·
  `carter2/base_link` 2개만 표시하도록 필터링돼 있음, 2026-07-27 정리). 백업:
  `docs/humble_ws_config/rviz2/`.

⚠ `docs/humble_ws_config/`는 스냅샷입니다 — humble_ws 쪽에서 이 파일들을 다시 튜닝하면 이
폴더는 자동으로 안 따라가니, 값이 바뀌면 실행 중인 머신 기준으로 다시 복사해 커밋해야 합니다.

**환경변수(진단/ROS2 CLI 직접 사용 시 필수, 6-1·15-1 참고)** — 대화형 셸엔 `.bashrc`가 이미
export해주지만, 비대화형 스크립트나 CI 등에서는 직접 지정해야 합니다. 아래는 **이 머신의 현재
`.bashrc` 실측값**(예전 인수인계서 초안엔 `ROS_DOMAIN_ID=151`/`~/.ros/fastdds_whitelist.xml`로
적혀 있었으나, 현재는 `net_loadtest` 격리망용으로 바뀌어 있음 — 실제 적용 중인 값 기준으로 작성):
```bash
export ROS_DOMAIN_ID=150
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/net_loadtest/fastdds_whitelist.xml"
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash   # carter_navigation(Nav2)
source ~/cobot3_ws/install/setup.bash                            # commander(미션/tf_relay/pc_reframe)
```
`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`/`FASTRTPS_DEFAULT_PROFILES_FILE` 3개는 격리망(net_loadtest)
접속용이라 이 팀 네트워크에 한정된 값입니다 — 다른 네트워크 환경이면 각자 값에 맞게 바꾸세요.

**참고용 `.bashrc` alias** (이 프로젝트 작업 중 자주 쓰는 사람은 개인 `.bashrc`에 등록해두면 편함,
레포에는 포함 안 됨 — 아래 "실행 순서"의 각 명령과 1:1 대응):
```bash
alias ros_set='source /opt/ros/humble/setup.bash'
alias sh_set='source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash'
alias isaac_python="~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
alias isaac_ros='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib'
```
(`isaac_ros`는 Isaac Sim 내장 파이썬에 ROS2 브릿지를 물릴 때, `isaac_python` 실행 **직전에 같은
셸에서** 먼저 실행해야 함 — 안 그러면 `ModuleNotFoundError: No module named 'rclpy'`로 죽습니다.
직접 재현해서 확인된 실패 케이스입니다.)

**받은 후 최초 1회만 (새 컴퓨터로 옮겼을 때)**
```bash
cd ~/cobot3_ws
colcon build                    # ROS2 패키지(commander 등) install/ 생성
./fix_doosan_mesh_paths.sh      # 로봇팔 mesh 절대경로를 이 컴퓨터 기준으로 재작성
```

**실행 순서 (터미널 6개)** — `run_isaac.sh`는 현재 `13_multi_robot_integrated.py`를 실행하도록
돼 있어 `19_`용이 아닙니다. `19_`는 아래처럼 수동 실행:
1. **Isaac Sim (19_)**
   ```bash
   isaac_ros
   cd isaacpjt/M0609
   isaac_python 19_dual_task_select_yolo_integrated.py
   ```
   (alias 없이 직접: `ISAAC="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"; LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC/exts/isaacsim.ros2.bridge/humble/lib" "$ISAAC/python.sh" isaacpjt/M0609/19_dual_task_select_yolo_integrated.py`)
   → 창 뜨면 **Play ▶** (필수, `/clock` 시작)
2. **Nav2**: `./run_nav.sh` (Nav2 멀티 + 통합 RViz + tf_relay×2 + initialpose 자동, 위 humble_ws 파일들 전제)
3. **HMI v2 백엔드** (미션 노드보다 먼저 떠 있어야 함 — `run_missions_19_hmi.sh`가 이 백엔드를
   전제로 시작 상태를 관리):
   ```bash
   ros_set && sh_set && source ~/cobot3_ws/install/setup.bash
   cd hmi/backend_v2 && uvicorn main:app --host 0.0.0.0 --port 8001
   ```
   (`robot_bridge.py`가 rclpy를 지연 임포트하므로 uvicorn 띄우기 전에 ROS 환경이 이 셸에 잡혀
   있어야 함 — 안 그러면 첫 로봇 연결 시점에 조용히 실패)
4. **미션 노드**: `./run_missions_19_hmi.sh` (미션 노드 2개 — 순수 Nav2 goal 릴레이)
5. (선택) **YOLO 비전 뷰어**: `./run_vision_19.sh` (창을 X로 닫거나 강제종료하면 실제로 종료됨,
   2026-07-29 수정 — 예전엔 창만 닫히고 프로세스가 안 죽어 다음 프레임에 창이 재생성됐음)
6. **프론트엔드**: `cd hmi/frontend_v2 && npm run dev` → 브라우저 `http://localhost:5174`
   (`hmi/frontend_v2/.env`의 `VITE_API_BASE`/`VITE_WS_URL`이 특정 머신 IP로 박혀있으면 안 됨 —
   `localhost:8001` 기준이어야 다른 컴퓨터에서도 그대로 동작)

## 📌 안내 
### 컴퓨터 켜서 작업 시작할 때: 
무조건 git pull origin main을 먼저 해서 다른 팀원이 고친 최신 코드를 내 노트북으로 가져옵니다~!

### 📌 내 기능 구현이 끝났을 때:
git add .
git commit -m "Feat: Implement FastAPI SQLite connection"
git push origin main

## 📌 커밋 메시지 기본 구조
태그: 제목 (영어 대문자로 시작하지 않고, 동사 원형 사용 권장 혹은 명확한 한글 표현)
- 본문 (생략 가능, 왜 변경했는지 설명이 필요할 때 작성)

## 📌 자주 사용하는 태그(Tag) 종류
- `feat`: 새로운 기능 추가
- `fix`: 버그 수정
- `docs`: 문서 수정 (README.md, 주석 등)
- `style`: 코드 포맷팅, 세미콜론 누락 등 (코드 자체의 로직 변경이 없는 경우)
- `refactor`: 코드 리팩토링 (기능 추가나 버그 수정이 없는 구조 개선)
- `test`: 테스트 코드 추가 및 수정
- `chore`: 빌드 업무 수정, 패키지 매니저 설정, 프로젝트 설정 변경 (.gitignore 등)

**올바른 예시:**
- `feat: 상품 검색 필터링 기능 추가`
- `fix: 로그인 화면에서 비밀번호 입력 시 앱이 꺼지는 오류 수정`
- `docs: README 파일에 설치 가이드 업데이트`

## 📌 PR 템플릿 양식
### 💡 관련 이슈 / 작업 내용
- 어떤 기능을 구현했는지 간략하게 적어주세요.
- 예: 로그인 페이지 마크업 및 카카오 로그인 API 연동

### 🛠️ 주요 변경 사항
- 핵심적으로 변경된 파일이나 로직을 적어주세요.
- `src/components/Login.tsx`: UI 컴포넌트 구현
- `src/hooks/useAuth.ts`: 인증 상태 관리 커스텀 훅 추가

### 📷 스크린샷 (선택 사항)
*UI 변경이 있는 경우 캡처 사진이나 GIF를 첨부하면 리뷰어가 이해하기 훨씬 쉽습니다.*

### ❓ 리뷰어에게 바라는 점 / 특이 사항
- 충돌이 우려되는 부분이나, 로직 피드백을 받고 싶은 부분을 적어주세요.

## 📌 브랜치 이름 명명 규칙 (Branch Naming)
브랜치 이름만 보고도 어떤 작업이 진행 중인지 파악할 수 있도록 접두사를 사용합니다.

- **형식:** `접두사/기능-요약` (또는 `접두사/이름-기능`)
- **접두사 종류:**
    - `feature/` : 새로운 기능 개발 (예: `feature/signup-form`)
    - `fix/` : 버그 수정 (예: `fix/token-error`)
    - `docs/` : 문서 작업 (예: `docs/readme`)
    - `refactor/` : 전면적인 코드 구조 개선 (예: `refactor/api-layer`)
---
## 📌 Github 작업 프로세스 
### 1. 최신 코드 가져오기 (시작 전 필수)
- git checkout main
- git pull origin main

### 2. 새 브랜치 만들고 이동하기
**브랜치 생성과 이동을 동시에 하기**
- git checkout -b 기능이름-또는-이슈번호

### 3. 코드 수정 및 상태 확인
- git status

### 4. 스테이징 및 커밋 (Commit)
**변경된 모든 파일을 올릴 때**
- git add .

**특정 파일만 올릴 때**
- git add 파일경로/파일명.확장자

**커밋 메시지 작성하기**
- git commit -m "feat: 로그인 기능 구현"

### 5. 원격 저장소에 푸시 (Push)
- git push -u origin 내-브랜치-이름

### 6. Pull Request (PR) 생성하기
푸시가 성공적으로 완료되면, GitHub 웹사이트에 접속
- 해당 레포지토리(Repository) 페이지로 이동하면 상단에 "Compare & pull request"라는 초록색 버튼이 자동으로 떠 있는 것을 볼 수 있음.
- 버튼이 보이지 않는다면 Pull requests 탭으로 이동해 "New pull request"를 누른 뒤, 내가 만든 브랜치를 선택함.
- base 브랜치(코드가 합쳐질 곳, 예: main)와 compare 브랜치(내가 작업한 브랜치)가 올바르게 선택되었는지 확인함.
- 제목과 어떤 내용을 수정했는지 본문을 상세히 적은 후 "Create pull request"를 누르면 완성!
---
## 📌 기타
### 원격 저장소(origin)의 브랜치 삭제
- git push origin --delete 브랜치이름

### 원격에서 지워진 브랜치 목록을 로컬에도 반영하여 정리
- git fetch --prune
