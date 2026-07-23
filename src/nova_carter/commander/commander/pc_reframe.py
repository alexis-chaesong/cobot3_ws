#!/usr/bin/env python3
"""
pc_reframe.py — 멀티로봇 통합 RViz용 PointCloud2 프레임 접두 릴레이
============================================================================
문제 : Isaac 은 포인트클라우드(/carterN/front_3d_lidar/lidar_points)의 header.frame_id 를
  접두 없이 'front_3d_lidar' 로 발행한다. 그런데 통합 RViz 가 쓰는 전역 /tf 에는 tf_relay 가
  접두를 붙인 'carterN/front_3d_lidar' 만 존재한다(map 은 공유). 그래서 통합 RViz(Fixed Frame=map)
  는 메시지 프레임 'front_3d_lidar' 를 TF 에서 못 찾아 포인트클라우드를 표시하지 못한다.

해결 : 이 노드가 입력 포인트클라우드를 받아 header.frame_id 에 접두(prefix/)를 붙여(=tf_relay 와
  동일 규칙, 전역 /tf 의 'carterN/front_3d_lidar' 와 매칭) 별도 토픽으로 재발행한다. 통합 RViz 에
  그 토픽을 보는 PointCloud2 디스플레이를 추가하면 두 로봇 포인트클라우드가 한 창에 표시된다.
  (Nav2/costmap 은 계속 원본 토픽을 쓰므로 간섭 없음 — 순수 시각화용.)

실행(로봇별 1개씩) :
  ros2 run commander pc_reframe --ros-args -p prefix:=carter1 \
      -p in_topic:=/carter1/front_3d_lidar/lidar_points \
      -p out_topic:=/carter1/front_3d_lidar/points_viz
============================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PcReframe(Node):
    def __init__(self):
        super().__init__("pc_reframe")
        self.declare_parameter("in_topic", "/carter1/front_3d_lidar/lidar_points")
        self.declare_parameter("out_topic", "/carter1/front_3d_lidar/points_viz")
        self.declare_parameter("prefix", "carter1")           # 프레임 접두 (tf_relay 와 동일)
        self.declare_parameter("shared_frames", ["map"])       # 접두 안 붙일 공유 프레임

        in_t = str(self.get_parameter("in_topic").value)
        out_t = str(self.get_parameter("out_topic").value)
        self.prefix = str(self.get_parameter("prefix").value).strip("/")
        self.shared = set(self.get_parameter("shared_frames").value)

        # 센서 QoS(best_effort) 로 구독/발행 — RViz PointCloud2 디스플레이도 Best Effort 로 설정할 것.
        self.pub = self.create_publisher(PointCloud2, out_t, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, in_t, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"reframe {in_t} → {out_t}  frame_id 접두 '{self.prefix}/' (shared={sorted(self.shared)})")

    def _cb(self, msg):
        f = msg.header.frame_id.lstrip("/")
        msg.header.frame_id = f if f in self.shared else f"{self.prefix}/{f}"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcReframe()
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
