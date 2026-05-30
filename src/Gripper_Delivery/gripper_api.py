#!/usr/bin/env python3
# =========================================================
# 夹爪控制 API (视觉队友 / 大模型专用)
# =========================================================
import rclpy
from rclpy.node import Node
from omnipicker_interfaces.srv import OmniPickerControl

class GripperCommander(Node):
    """夹爪控制接口类，供视觉端或大模型直接调用"""
    def __init__(self):
        super().__init__('gripper_commander_node')
        
        # 创建服务客户端，对接底层 RS485 驱动
        self.cli = self.create_client(OmniPickerControl, '/omnipicker_control')
        
        # 智能等待底层驱动上线
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 等待夹爪底层驱动 (omnipicker_control) 上线...')
        self.get_logger().info('✅ 夹爪 API 已连接，随时可以发送指令！')

    def send_command(self, pos, force, vel=0.5, acc=0.5, dec=0.5):
        """核心发送逻辑：发送请求并阻塞等待结果（双向通信）"""
        request = OmniPickerControl.Request()
        request.pos = float(pos)
        request.force = float(force)
        request.vel = float(vel)
        request.acc = float(acc)
        request.dec = float(dec)

        # 异步调用服务
        future = self.cli.call_async(request)
        
        # 阻塞等待底层返回结果 (比如抓到了，或者掉落了)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            print(f"📥 [API收到] 结果: {response.result}, 状态码: {response.result_code}")
            return response.result, response.result_code
        else:
            print("❌ [API报错] 服务调用异常")
            return False, 'error'

    # ================= 封装给队友的“绝招” =================

    def open(self, speed=0.8):
        """绝招1：张开夹爪 (松开物体)"""
        print("👉 [API发出] 命令夹爪：张开 🫱")
        # 假设 pos=1.0 是完全张开，张开时力度给小一点即可
        return self.send_command(pos=1.0, force=0.2, vel=speed)

    def close(self, force=0.5, speed=0.8):
        """绝招2：闭合夹爪 (抓取物体)"""
        print(f"👉 [API发出] 命令夹爪：闭合抓取 ✊ (设定力度:{force})")
        # 假设 pos=0.0 是完全闭合
        return self.send_command(pos=0.0, force=force, vel=speed)
        
    def custom_grasp(self, pos, force, speed=0.5):
        """绝招3：自定义位置抓取 (适用于知道物体大致尺寸的场景)"""
        print(f"👉 [API发出] 命令夹爪：自定义位置抓取 🤏 (目标位置:{pos})")
        return self.send_command(pos=pos, force=force, vel=speed)

# ================= 队友/大模型实战调用测试 =================
def main(args=None):
    rclpy.init(args=args)
    
    # 1. 实例化 API 类
    gripper = GripperCommander()
    
    print("\n-----------------------------------")
    print("测试1：命令夹爪张开")
    success, code = gripper.open(speed=0.8)
    print(f"测试1完成 -> 成功与否: {success}, 状态: {code}")
    
    print("\n-----------------------------------")
    print("测试2：命令夹爪进行闭合抓取 (测试堵转反馈)")
    success, code = gripper.close(force=0.6)
    print(f"测试2完成 -> 成功与否: {success}, 状态: {code}")
    
    if code == 'grasped':
        print("🎉 完美！成功感知到物体并抓紧！")
    elif code == 'timeout':
        print("⚠️ 抓空了或者动作超时！")

    # 销毁节点并关闭 ROS 2
    gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()