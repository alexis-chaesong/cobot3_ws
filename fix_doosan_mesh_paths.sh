#!/usr/bin/env bash
# =====================================================================
# src/doosan-robot2/ 는 외부 레포라 git 추적 안 함(.gitignore) → 팀원마다 다른 경로/계정에서
# 복사해온 사본을 쓴다. 문제는 일부 URDF(특히 m0609_isaac_sim.urdf, m0609_with_nozzle.urdf)의
# <mesh filename="..."> 가 ROS package:// 가 아니라 "절대경로"로 박혀 있어서, 원본을 만든
# 머신/계정 경로(예: /home/jung/cobot3_ws/... 또는 다른 워크스페이스 이름)가 그대로 남아있으면
# 이 머신에서는 메시가 안 열린다(Isaac URDF 임포터가 조용히 스킵하거나 경고만 내고 무시함).
#
# 이 스크립트는 src/doosan-robot2/urdf/*.urdf 안의 "/home/.../doosan-robot2/..." 절대경로를
# 전부 "<이 레포의 실제 doosan-robot2 위치>/..." 로 재작성한다. 여러 번 실행해도 안전(idempotent).
#
# 사용법 : ./fix_doosan_mesh_paths.sh   (레포 루트에서, 또는 아무 데서나 실행 가능)
# =====================================================================
set -e
_THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOOSAN_DIR="$_THIS_DIR/src/doosan-robot2"

if [ ! -d "$DOOSAN_DIR" ]; then
    echo "[fix_doosan_mesh_paths] $DOOSAN_DIR 없음 — 먼저 doosan-robot2 를 src/ 밑에 복사해두세요." >&2
    exit 1
fi

python3 - "$DOOSAN_DIR" <<'PYEOF'
import re
import sys
from pathlib import Path

doosan_dir = Path(sys.argv[1]).resolve()

# .../doosan-robot2/(나머지 경로) 형태의 절대경로를 이 머신의 실제 doosan_dir 로 교체.
# 대소문자 그대로, 계정/워크스페이스 이름은 뭐든 상관없이 "doosan-robot2" 라는 디렉토리명
# 하나만 앵커로 삼는다.
pattern = re.compile(r'/home/[^"\s]*?/doosan-robot2/')
replacement = str(doosan_dir) + "/"

changed_files = 0
changed_refs = 0
for urdf in list(doosan_dir.rglob("*.urdf")) + list(doosan_dir.rglob("*.xacro")):
    text = urdf.read_text()
    new_text, n = pattern.subn(replacement, text)
    if n:
        urdf.write_text(new_text)
        changed_files += 1
        changed_refs += n
        print(f"[fix_doosan_mesh_paths] {urdf.relative_to(doosan_dir.parent)}: {n}개 경로 수정")

if changed_files:
    print(f"[fix_doosan_mesh_paths] 완료 — 파일 {changed_files}개, 경로 {changed_refs}개 → {doosan_dir}/ 기준으로 재작성")
else:
    print("[fix_doosan_mesh_paths] 고칠 게 없음 (이미 이 머신 경로거나 package:// 상대참조만 사용 중)")
PYEOF
