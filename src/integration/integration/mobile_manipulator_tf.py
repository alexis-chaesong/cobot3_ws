#!/usr/bin/env python3
"""
mobile_manipulator_tf.py

Nova Carter의 chassis_link와 m0609 로봇 팔의 base_link 사이의
Static TF를 발행하는 노드.

Isaac Sim에서 두 로봇이 Fixed Joint로 물리적으로 연결되어 있지만,
TF 트리 상으로는 연결이 끊겨 있기 때문에 이 노드로 연결해야 합니다.

실행 방법:
  source /opt/ros/humble/setup.bash
  python3 mobile_manipulator_tf.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class MobileManipulatorTF(Node):
    def __init__(self):
        super().__init__('mobile_manipulator_tf')

        self.broadcaster = StaticTransformBroadcaster(self)

        # chassis_link → arm base_link 연결 TF
        # localPos0 값과 동일하게 맞춰야 함 (root_joint 설정 기준)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'chassis_link'          # 부모: Carter 차체
        tf.child_frame_id = 'base_link'              # 자식: 로봇 팔 베이스

        # ※ 아래 값은 Isaac Sim의 root_joint localPos0 설정값과 일치시켜야 함
        # (현재 TF 데이터 기준: arm이 chassis_link로부터 x=-0.2317, z=0.577 위치)
        tf.transform.translation.x = -0.2317
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.577

        # 회전 없음 (팔이 카터와 같은 방향)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(tf)
        self.get_logger().info(
            f"[MobileManipulatorTF] Static TF published: "
            f"chassis_link → base_link "
            f"(x={tf.transform.translation.x}, "
            f"y={tf.transform.translation.y}, "
            f"z={tf.transform.translation.z})"
        )


def main():
    rclpy.init()
    node = MobileManipulatorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
