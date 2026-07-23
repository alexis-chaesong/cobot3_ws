#!/usr/bin/env python3
"""
tf_relay.py — 멀티로봇 단일 RViz용 TF 중계기
============================================================================
문제 : carter1/carter2 는 각자 namespace 된 /carterN/tf 로만 TF 를 발행하고, 프레임명은
  공유(base_link·odom·map ...)라, 전역 /tf 를 구독하는 "통합 RViz 1개"는 TF 를 전혀 못 받아
  ("No tf data") Fixed Frame(map) 조차 없어 아무것도 안 그려진다.

해결 : 이 노드가 /{ns}/tf(_static) 를 구독해, 프레임명에 접두(prefix/)를 붙여(단 shared_frames
  =map 은 그대로 공유) 전역 /tf(_static) 로 재발행한다. 그러면 전역 TF 트리가
     map → carter1/odom → carter1/base_link → ...  +  map → carter2/odom → ...
  로 하나가 되어, 통합 RViz(Fixed Frame=map)가 map 프레임을 갖고 map 기준 표시(Costmap·Path·
  Amcl·Map)를 그린다. (Nav2 자체는 여전히 자기 namespace 의 /carterN/tf 를 쓰므로 간섭 없음 —
  이 중계는 순수 시각화용 전역 /tf 공급.)

  ※ scan/RobotModel 처럼 "메시지의 frame_id(접두 없음)"에 의존하는 표시는 접두된 TF 와 안 맞아
    통합뷰에선 못 쓴다(그래서 통합 config 는 map 프레임 표시만 넣음). map 프레임 표시엔 충분.

실행(로봇별 1개씩) :
  ros2 run commander tf_relay --ros-args -p in_ns:=carter1 -p prefix:=carter1
  ros2 run commander tf_relay --ros-args -p in_ns:=carter2 -p prefix:=carter2
============================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")
        self.declare_parameter("in_ns", "carter1")      # 구독할 소스 namespace
        self.declare_parameter("prefix", "carter1")     # 프레임 접두 (보통 in_ns 와 동일)
        self.declare_parameter("shared_frames", ["map"])  # 접두 안 붙이고 공유할 프레임

        ns = str(self.get_parameter("in_ns").value).strip("/")
        self.prefix = str(self.get_parameter("prefix").value).strip("/")
        self.shared = set(self.get_parameter("shared_frames").value)

        # /tf : 일반(volatile), /tf_static : latched(transient_local)
        self.pub = self.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(depth=100,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_static = self.create_publisher(TFMessage, "/tf_static", static_qos)

        self.create_subscription(TFMessage, f"/{ns}/tf", self._cb, 10)
        self.create_subscription(TFMessage, f"/{ns}/tf_static", self._cb_static, static_qos)
        self.get_logger().info(
            f"relay /{ns}/tf(_static) → /tf(_static)  prefix='{self.prefix}/'  shared={sorted(self.shared)}")

    def _pref(self, frame):
        f = frame.lstrip("/")
        return f if f in self.shared else f"{self.prefix}/{f}"

    def _remap(self, msg):
        out = TFMessage()
        for t in msg.transforms:
            t.header.frame_id = self._pref(t.header.frame_id)
            t.child_frame_id = self._pref(t.child_frame_id)
            out.transforms.append(t)
        return out

    def _cb(self, msg):
        self.pub.publish(self._remap(msg))

    def _cb_static(self, msg):
        self.pub_static.publish(self._remap(msg))


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
