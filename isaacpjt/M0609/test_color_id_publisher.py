"""
/color_id 테스트용 임시 퍼블리셔.

목적: 팀 B의 실제 색상 감지 노드가 아직 준비되지 않은 상태에서,
      ROS2 통신 경로(도메인ID, RMW, DDS 화이트리스트 등) 자체가 살아있는지와
      6_pick_place_color.py 의 ColorIdSubscriber가 실제로 값을 받는지를
      독립적으로 확인하기 위한 스크립트. 실제 색상 감지 로직은 없다.

사용법:
    (같은 PC / 같은 ROS_DOMAIN_ID 환경에서)
    python3 test_color_id_publisher.py

    → std_msgs/Int32 를 /color_id 토픽으로 1초에 한 번씩 1, 2, 1, 2 ... 번갈아 발행한다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ColorIdTestPublisher(Node):
    def __init__(self):
        super().__init__("color_id_test_publisher")
        self._pub = self.create_publisher(Int32, "/color_id", 10)
        self._next_value = 1  # 1=파랑, 2=초록
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self):
        msg = Int32()
        msg.data = self._next_value
        self._pub.publish(msg)
        self.get_logger().info(f"[TEST PUB] /color_id <- {self._next_value}")
        self._next_value = 2 if self._next_value == 1 else 1


def main():
    rclpy.init()
    node = ColorIdTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
