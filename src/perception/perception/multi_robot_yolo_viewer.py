#!/usr/bin/env python3
"""
multi_robot_yolo_viewer.py  ★14번(멀티로봇+사람) 전용 실시간 YOLO 뷰어 + 회피 신호원★
================================================================================
14_multi_robot_yolotest.py 와 짝을 이루는 ROS2 노드. 두 Nova Carter 의 전방 스테레오
카메라(hawk) 이미지를 구독해 YOLO 로 사람/쓰레기통을 검출하고 :
  1) cv2.imshow 로 "두 로봇 나란히" 실시간 창을 띄운다(사용자 확인용 GUI).
  2) 각 로봇 전방에 '사람이 위험 근접'인지 판정해 /carterN/person_alert(Bool) 를 발행한다.
     → 14 의 PersonGate 가 이 토픽을 받아 스크립트 구동 cmd_vel 구간(스윕/파지접근)을 정지·재개.

★왜 별도 프로세스(시스템 python3)인가★
  Isaac 번들 opencv 는 GUI(GTK/Qt) 가 없어 Isaac python.sh 안에서는 cv2.imshow 가 안 뜬다
  (인수인계서 확인). 반면 시스템 python3 의 opencv 는 Qt GUI 를 갖고 있어 창이 정상으로 뜬다.
  또 sim 루프가 GPU 병목(98%)이라 YOLO 를 Isaac 프로세스 밖에서 돌려 부하를 분리한다.

실행 (★시스템 python3, ROS 소싱된 터미널★ — Isaac python.sh 아님!) :
  cd /home/rokey/cobot3_ws
  python3 src/perception/perception/multi_robot_yolo_viewer.py

  # 옵션
  python3 .../multi_robot_yolo_viewer.py \
      --weights src/perception/models/trash_can_person_yolo11s.pt \
      --robots carter1 carter2 --conf 0.35 --rate 8 --near-frac 0.5

전제 : 14 (Isaac) Play + Nav2 멀티런치가 떠 있어 카메라 토픽이 발행 중이어야 한다.
  확인 : ros2 topic list | grep image_raw
         → /carter1/front_stereo_camera/left/image_raw · /carter2/... 가 보여야 함.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[2]
_DEFAULT_WEIGHTS = str(_WS_ROOT / "src" / "perception" / "models" / "trash_can_person_yolo11s.pt")
_FALLBACK_WEIGHTS = str(_WS_ROOT / "src" / "perception" / "models" / "yolo11s.pt")


def _parse_args():
    p = argparse.ArgumentParser(description="Multi-robot YOLO viewer + person_alert publisher")
    p.add_argument("--weights", default=_DEFAULT_WEIGHTS,
                   help=f"YOLO 가중치 (기본 {_DEFAULT_WEIGHTS}, 없으면 yolo11s.pt 로 폴백)")
    p.add_argument("--robots", nargs="+", default=["carter1", "carter2"],
                   help="네임스페이스 목록 (기본 carter1 carter2)")
    p.add_argument("--image-suffix", default="front_stereo_camera/left/image_raw",
                   help="네임스페이스 뒤 카메라 토픽 경로 (기본 front hawk left)")
    p.add_argument("--conf", type=float, default=0.35, help="YOLO confidence 임계 (기본 0.35)")
    p.add_argument("--rate", type=float, default=8.0, help="처리/표시 Hz (GPU 공유 → 너무 높이지 말 것)")
    p.add_argument("--device", default=None, help="YOLO device (예: cuda:0 / cpu). 기본 자동")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO 입력 크기")
    p.add_argument("--person-class", default="person",
                   help="사람으로 볼 클래스명 부분일치 (기본 'person')")
    p.add_argument("--near-frac", type=float, default=0.5,
                   help="사람 bbox 높이/이미지높이 가 이 값 이상 + 화면중앙이면 '위험근접'(alert) (기본 0.5)")
    p.add_argument("--center-band", type=float, default=0.6,
                   help="화면 가로 중앙 band 폭 비율(0~1). 이 band 안 중심이어야 위험판정 (기본 0.6)")
    p.add_argument("--no-window", action="store_true", help="창 없이 alert 만 발행(headless)")
    p.add_argument("--max-width", type=int, default=1600, help="합성 창 최대 가로 픽셀")
    return p.parse_args()


def resolve_weights(path: str) -> str:
    if Path(path).is_file():
        return path
    if Path(_FALLBACK_WEIGHTS).is_file():
        print(f"[WARN] weights 없음({path}) → 폴백 {_FALLBACK_WEIGHTS}", flush=True)
        return _FALLBACK_WEIGHTS
    raise FileNotFoundError(f"YOLO weights not found: {path}")


def decode_image(msg) -> np.ndarray | None:
    """sensor_msgs/Image → BGR np.uint8 (cv_bridge 없이 직접 디코드).
    Isaac 카메라는 보통 rgb8. bgr8/mono8 도 대응."""
    if msg is None or not msg.data:
        return None
    enc = (msg.encoding or "").lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    try:
        if enc in ("rgb8", "bgr8"):
            row = buf.reshape(h, step)[:, : w * 3]
            img = row.reshape(h, w, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
        if enc in ("rgba8", "bgra8"):
            row = buf.reshape(h, step)[:, : w * 4]
            img = row.reshape(h, w, 4)
            code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(img, code)
        if enc in ("mono8", "8uc1"):
            img = buf.reshape(h, step)[:, :w].reshape(h, w)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    except Exception as exc:
        print(f"[WARN] decode 실패 enc={enc} {h}x{w} step={step}: {exc}", flush=True)
        return None
    print(f"[WARN] 미지원 encoding='{enc}' → skip", flush=True)
    return None


def make_panel(bgr, title, n_det, alert, target_h=480):
    """검출 그려진 프레임에 상단 배너(로봇명·검출수·ALERT) 붙이고 높이 정규화."""
    if bgr is None:
        bgr = np.full((target_h, int(target_h * 4 / 3), 3), 40, np.uint8)
        cv2.putText(bgr, f"{title}: waiting for camera...", (20, target_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2, cv2.LINE_AA)
    h, w = bgr.shape[:2]
    if h != target_h:
        bgr = cv2.resize(bgr, (int(w * target_h / h), target_h))
    banner = np.zeros((44, bgr.shape[1], 3), np.uint8)
    color = (0, 0, 255) if alert else (0, 200, 0)
    tag = "PERSON NEAR - STOP" if alert else "clear"
    cv2.putText(banner, f"{title}  det={n_det}  [{tag}]", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    panel = np.vstack([banner, bgr])
    if alert:
        cv2.rectangle(panel, (0, 0), (panel.shape[1] - 1, panel.shape[0] - 1), (0, 0, 255), 4)
    return panel


def main() -> int:
    args = _parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
    from ultralytics import YOLO

    weights = resolve_weights(args.weights)
    print(f"[INFO] YOLO weights = {weights}", flush=True)
    model = YOLO(weights)
    names = model.names if isinstance(model.names, dict) else {i: n for i, n in enumerate(model.names)}
    print(f"[INFO] classes = {names}", flush=True)

    rclpy.init()
    node = rclpy.create_node("multi_robot_yolo_viewer")
    # Isaac 카메라는 sensor-data QoS(BEST_EFFORT). 안 맞추면 이미지가 안 들어온다.
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

    state = {}  # ns -> dict(frame, alert_pub)
    for ns in args.robots:
        topic = f"/{ns}/{args.image_suffix}"
        st = {"msg": None, "alert_pub": node.create_publisher(Bool, f"/{ns}/person_alert", 10)}
        state[ns] = st
        node.create_subscription(Image, topic, (lambda m, s=st: s.__setitem__("msg", m)), qos)
        print(f"[INFO] {ns}: sub {topic} → pub /{ns}/person_alert", flush=True)

    show = not args.no_window
    win = "Multi-Robot YOLO (carter1 | carter2)"
    if show:
        try:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        except Exception as exc:
            show = False
            print(f"[WARN] cv2 창 불가({exc}) → headless 로 alert 만 발행", flush=True)

    band = float(np.clip(args.center_band, 0.1, 1.0))
    period = 1.0 / max(1.0, args.rate)
    print(f"[INFO] 처리율 {args.rate}Hz, conf>={args.conf}, near_frac>={args.near_frac}, "
          f"center_band={band}, window={show}", flush=True)

    def is_person(cls_id):
        return args.person_class.lower() in str(names.get(int(cls_id), "")).lower()

    ret = 0
    try:
        while rclpy.ok():
            t0 = time.perf_counter()
            rclpy.spin_once(node, timeout_sec=period)

            panels = []
            for ns in args.robots:
                st = state[ns]
                bgr = decode_image(st["msg"])
                n_det = 0
                alert = False
                ann = bgr
                if bgr is not None:
                    kw = dict(source=bgr, conf=args.conf, imgsz=args.imgsz, verbose=False)
                    if args.device:
                        kw["device"] = args.device
                    res = model.predict(**kw)[0]
                    ann = res.plot()
                    if res.boxes is not None and len(res.boxes):
                        n_det = len(res.boxes)
                        H, W = bgr.shape[:2]
                        cx_lo, cx_hi = (0.5 - band / 2.0) * W, (0.5 + band / 2.0) * W
                        for b in res.boxes:
                            if not is_person(b.cls.item()):
                                continue
                            x1, y1, x2, y2 = b.xyxy[0].tolist()
                            frac = (y2 - y1) / max(1.0, H)
                            cx = 0.5 * (x1 + x2)
                            if frac >= args.near_frac and cx_lo <= cx <= cx_hi:
                                alert = True
                                # 위험 사람 박스 강조
                                cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                # alert 는 매 주기 발행(사람 없으면 False) → 게이트가 즉시 해제 가능
                st["alert_pub"].publish(Bool(data=bool(alert)))
                if show:
                    panels.append(make_panel(ann, ns, n_det, alert))

            if show and panels:
                # 나란히 합성(구분선)
                sep = np.full((panels[0].shape[0], 6, 3), 90, np.uint8)
                combo = panels[0]
                for pnl in panels[1:]:
                    combo = np.hstack([combo, sep, pnl])
                if combo.shape[1] > args.max_width:
                    s = args.max_width / combo.shape[1]
                    combo = cv2.resize(combo, (args.max_width, int(combo.shape[0] * s)))
                try:
                    cv2.imshow(win, combo)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        print("[INFO] quit key", flush=True)
                        break
                except Exception as exc:
                    show = False
                    print(f"[WARN] imshow 실패({exc}) → headless 전환", flush=True)

            # 처리율 유지(과도한 GPU 점유 방지)
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        ret = 1
    finally:
        # 종료 시 alert=False 로 남겨 로봇이 갇히지 않게
        try:
            for ns in args.robots:
                state[ns]["alert_pub"].publish(Bool(data=False))
        except Exception:
            pass
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return ret


if __name__ == "__main__":
    sys.exit(main())
