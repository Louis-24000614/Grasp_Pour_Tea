#!/usr/bin/env python3
"""
gripper_state_monitor.py - 订阅夹爪实时状态并打印

用法：
  python3 gripper_state_monitor.py
"""

import rclpy
from rclpy.node import Node
from omnipicker_interfaces.msg import OmniPickerState


STATE_MAP = {
    'arrived': '✅ 已到位',
    'moving': '🔄 运动中',
    'stalled': '📦 堵转/已抓取',
    'dropped': '⚠️ 物体掉落',
}


class GripperStateMonitor(Node):
    def __init__(self):
        super().__init__('gripper_state_monitor')
        self.create_subscription(
            OmniPickerState,
            '/omnipicker_state',
            self.state_cb,
            10
        )
        self.get_logger().info('👂 开始监听 /omnipicker_state ...')

    def state_cb(self, msg: OmniPickerState):
        state_desc = STATE_MAP.get(msg.picker_state, msg.picker_state)
        self.get_logger().info(
            f'{state_desc} | '
            f'位置:{msg.rt_pos:.3f} | '
            f'力矩:{msg.rt_force:.3f} | '
            f'故障码:{msg.picker_fault_code or "无"}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
