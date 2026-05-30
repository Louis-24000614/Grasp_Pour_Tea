#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import os
from omnipicker_interfaces.srv import OmniPickerControl
from omnipicker_interfaces.msg import OmniPickerState


class OmniPickerRS485Driver(Node):
    def __init__(self):
        super().__init__('omnipicker_rs485_driver')
        self.driver_tag = 'RS485_DRIVER_BUILD_2026_04_15_A'
        self.port_name = os.environ.get('OMNIPICKER_PORT', '/dev/ttyUSB0')
        self.baudrate = 115200
        try:
            self.serial_port = serial.Serial(port=self.port_name, baudrate=self.baudrate, timeout=0.05)
            self.get_logger().info(f'✅ RS485 串口已就绪: {self.port_name}')
            self.get_logger().info(f'🧩 驱动版本: {self.driver_tag}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口打开失败: {e}')
            raise SystemExit

        self.srv = self.create_service(OmniPickerControl, '/omnipicker_control', self.handle_control_request)
        self.state_pub = self.create_publisher(OmniPickerState, '/omnipicker_state', 10)
        self.action_timeout_sec = 8.0
        self.open_pos_threshold = 220  # 0~255 标尺下，认为“基本张开”
        self.open_pos_min_progress = 180

    def handle_control_request(self, request, response):
        self.get_logger().info(f'📥 下发指令 -> 目标位置:{request.pos}, 力度:{request.force}')

        # 浮点数转换为 0~255 的十六进制控制指令
        target_pos_hex = int(max(0.0, min(1.0, request.pos)) * 255)
        force_hex = int(max(0.0, min(1.0, request.force)) * 255)
        vel_hex = int(max(0.0, min(1.0, request.vel)) * 255)
        acc_hex = int(max(0.0, min(1.0, request.acc)) * 255)
        dec_hex = int(max(0.0, min(1.0, request.dec)) * 255)

        command_bytes = bytearray([
            0x41, 0x41, 0x01, 0x00,
            target_pos_hex, force_hex, vel_hex, acc_hex, dec_hex,
            0x00, 0x00, 0x00
        ])

        # 计算校验和
        command_bytes[11] = (~sum(command_bytes[2:11])) & 0xFF

        # 【核心逻辑】：判断意图。目标位置小于 128 (50%) 认为是闭合抓取，大于等于 128 是张开。
        is_closing = target_pos_hex < 128

        # 1. 首次发送指令，唤醒夹爪起步
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(command_bytes)
            self.serial_port.flush()
        except Exception as e:
            self.get_logger().error(f'❌ 发送异常: {e}')
            response.result, response.result_code = False, 'serial_error'
            return response

        # 给夹爪 0.1 秒反应时间起步，卸载可能残余的旧力矩和状态
        time.sleep(0.1)

        # 2. RS485 轮询机制：持续发包索要状态
        start_time = time.time()
        max_pos_fb = -1
        last_pos_fb = -1
        stable_count = 0
        seen_feedback = False
        last_debug_ts = 0.0

        while (time.time() - start_time) < self.action_timeout_sec:
            # 不断轮询发送指令，获取最新反馈
            self.serial_port.write(command_bytes)
            self.serial_port.flush()

            # 等待夹爪回传（12字节报文响应极快，等20毫秒足以接收完整）
            time.sleep(0.02)

            if self.serial_port.in_waiting >= 12:
                raw_data = self.serial_port.read(self.serial_port.in_waiting)

                # 寻找最新一帧数据的帧头 0x41 0x41
                idx = raw_data.rfind(b'\x41\x41')
                if idx != -1 and (len(raw_data) - idx) >= 12:
                    frame = raw_data[idx:idx+12]

                    # 严格按照 OmniPicker 协议手册索引解析
                    state_fb = frame[4]  # 状态: 00=到达, 01=运动中, 02=堵转, 03=掉落
                    pos_fb = frame[5]    # 当前位置
                    force_fb = frame[7]  # 当前力矩
                    seen_feedback = True
                    if pos_fb > max_pos_fb:
                        max_pos_fb = pos_fb
                    if pos_fb == last_pos_fb:
                        stable_count += 1
                    else:
                        stable_count = 0
                        last_pos_fb = pos_fb

                    # 发布实时状态 Topic
                    state_msg = OmniPickerState()
                    state_msg.raw_data = list(frame)
                    state_msg.rt_pos = float(pos_fb) / 255.0
                    state_msg.rt_force = float(force_fb) / 255.0
                    state_msg.rt_vel = 0.0
                    if state_fb == 0x00:
                        state_msg.picker_state = 'arrived'
                        state_msg.picker_fault_code = ''
                    elif state_fb == 0x01:
                        state_msg.picker_state = 'moving'
                        state_msg.picker_fault_code = ''
                    elif state_fb == 0x02:
                        state_msg.picker_state = 'stalled'
                        state_msg.picker_fault_code = 'object_grasped'
                    elif state_fb == 0x03:
                        state_msg.picker_state = 'dropped'
                        state_msg.picker_fault_code = 'object_dropped'
                    else:
                        state_msg.picker_state = f'unknown_0x{state_fb:02X}'
                        state_msg.picker_fault_code = ''
                    self.state_pub.publish(state_msg)

                    now = time.time()
                    if now - last_debug_ts > 0.5:
                        self.get_logger().info(
                            f'反馈 state=0x{state_fb:02X}, pos={pos_fb}, force={force_fb}, max_pos={max_pos_fb}'
                        )
                        last_debug_ts = now

                    # 判定 1: 官方明确反馈【到达目标】 (0x00)
                    if state_fb == 0x00:
                        # 加一层位置校验，防止极低概率读到旧的 00 状态
                        if is_closing and abs(pos_fb - target_pos_hex) <= 15:
                            self.get_logger().info(f'✅ 动作完成! 状态:0x00, 位置: {pos_fb}')
                            response.result, response.result_code = True, 'success'
                            return response
                        if (not is_closing) and (pos_fb >= self.open_pos_threshold):
                            self.get_logger().info(f'✅ 开口到位! 状态:0x00, 位置: {pos_fb}')
                            response.result, response.result_code = True, 'open_success'
                            return response

                    # 判定 2: 官方明确反馈【堵转/抓到物体】 (0x02)
                    elif state_fb == 0x02:
                        if is_closing:
                            self.get_logger().info(f'📦 触发堵转抓取成功! 状态:0x02, 力矩: {force_fb}, 位置: {pos_fb}')
                            response.result = True
                            response.result_code = 'grasped'
                            return response
                        else:
                            # 张开时若位置已接近最大开口，也认为成功
                            if pos_fb >= self.open_pos_threshold:
                                self.get_logger().info(f'✅ 开口到位(0x02容错)! 位置: {pos_fb}')
                                response.result = True
                                response.result_code = 'open_success'
                                return response

                    # 开口动作下，状态可能长期为 0x01（运动中）但位置已经到位，直接判成功
                    if (not is_closing) and (pos_fb >= self.open_pos_threshold):
                        self.get_logger().info(f'✅ 开口到位(状态容错)! state=0x{state_fb:02X}, 位置: {pos_fb}')
                        response.result = True
                        response.result_code = 'open_success'
                        return response

                    # 判定 3: 物体意外滑落 (0x03)
                    elif state_fb == 0x03:
                        self.get_logger().warn(f'⚠️ 夹取物体掉落! 状态:0x03')
                        response.result, response.result_code = False, 'dropped'
                        return response

            # 歇 30ms 继续下一次状态问询，降低 CPU 和串口负载
            time.sleep(0.03)

        # 判定 4: 5秒钟依然没有完成任何条件，抛出超时
        # 张开动作的容错：只要有回包且最大开口已明显增大，就判为开口成功，避免卡流程
        if (not is_closing) and seen_feedback and (max_pos_fb >= self.open_pos_min_progress):
            self.get_logger().warn(
                f'⚠️ 开口超时但位置已足够大(max_pos={max_pos_fb})，按成功处理'
            )
            response.result, response.result_code = True, 'open_timeout_pass'
            return response

        self.get_logger().warn(
            f'⏱️ 夹爪动作超时 ({self.action_timeout_sec:.1f}s), seen_feedback={seen_feedback}, max_pos={max_pos_fb}'
        )
        if seen_feedback:
            response.result, response.result_code = False, 'timeout_with_feedback'
        else:
            response.result, response.result_code = False, 'timeout_no_feedback'
        return response

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniPickerRS485Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
