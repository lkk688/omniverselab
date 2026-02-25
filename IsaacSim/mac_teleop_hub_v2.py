#pip install pygame pyserial requests h5py

import argparse
import json
import time
import socket
import serial
import pygame
import h5py
import numpy as np
import os
from datetime import datetime

# ==========================================
# 🔧 核心配置区
# ==========================================
UDP_IP = "192.168.1.100"  # 5090 的 IP
UDP_PORT = 8212
ROARM_SERIAL_PORT = '/dev/cu.usbserial-0001' # Mac 识别的串口名

import requests # 确保顶部已导入 requests

class RoArmController:
    """全面集成 Waveshare RoArm-M2-S (完美支持 Serial 与 Wi-Fi 双模式)"""
    def __init__(self, mode="serial", serial_port='/dev/cu.usbserial-0001', wifi_ip="192.168.1.50"):
        self.mode = mode.lower()
        self.serial_port = serial_port
        self.wifi_ip = wifi_ip
        self.ser = None

        if self.mode == "serial":
            try:
                self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
                print(f"🟢 [RoArm] 已通过串口连接: {self.serial_port}")
            except Exception as e:
                print(f"❌ [RoArm] 串口连接失败: {e}")
                
        elif self.mode == "wifi":
            try:
                # 启动时发送一个测试请求，验证 IP 是否正确
                print(f"🔄 [RoArm] 正在尝试连接 Wi-Fi: {self.wifi_ip} ...")
                requests.get(f"http://{self.wifi_ip}/", timeout=2.0)
                print(f"🟢 [RoArm] 已成功通过 Wi-Fi 连接: {self.wifi_ip}")
            except Exception as e:
                print(f"❌ [RoArm] Wi-Fi 连接失败, 请检查 IP 或网络: {e}")
        else:
            raise ValueError("模式必须为 'serial' 或 'wifi'")

    def send_cmd(self, cmd_dict):
        """发送 JSON 指令，根据模式自动路由到底层协议"""
        if self.mode == "serial" and self.ser:
            cmd_str = json.dumps(cmd_dict) + "\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                print(f"⚠️ [RoArm] 串口写入错误: {e}")

        elif self.mode == "wifi":
            try:
                # 💡 核心：设置极短的 timeout (0.05秒)。
                # 因为在 50Hz 的控制环中，如果 Wi-Fi 丢包，我们宁愿跳过这帧，也不能让整个程序卡住等待！
                url = f"http://{self.wifi_ip}/js?json={json.dumps(cmd_dict)}"
                requests.get(url, timeout=0.05)
            except requests.exceptions.Timeout:
                pass # 实时遥控允许偶尔的丢包，静默处理
            except Exception as e:
                print(f"⚠️ [RoArm] Wi-Fi 发送错误: {e}")

    def torque_off(self):
        """💥 释放力矩 (教导模式) - 允许手动拖拽"""
        print("[RoArm] 🟢 已开启教导模式 (释放电机力矩)")
        self.send_cmd({"T": 210, "cmd": 0})

    def torque_on(self):
        """💥 开启力矩 (控制模式) - 锁死电机等待指令"""
        print("[RoArm] 🔴 已开启伺服控制模式 (锁定电机力矩)")
        self.send_cmd({"T": 210, "cmd": 1})

    def move_ik(self, x, y, z, pitch, spd=0):
        self.send_cmd({"T": 101, "x": x, "y": y, "z": z, "t": pitch, "spd": spd})

    def set_gripper(self, val):
        self.send_cmd({"T": 104, "b": val})

    def get_pose(self):
        """T:105 获取当前位姿，双协议适配解析"""
        cmd_dict = {"T": 105}

        if self.mode == "serial" and self.ser:
            self.send_cmd(cmd_dict)
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    return json.loads(line)
            except:
                pass

        elif self.mode == "wifi":
            try:
                # 获取数据需要等待返回，稍微放宽一点超时时间 (0.15秒)
                url = f"http://{self.wifi_ip}/js?json={json.dumps(cmd_dict)}"
                res = requests.get(url, timeout=0.15)
                if res.status_code == 200:
                    data = res.text.strip()
                    if data.startswith('{') and data.endswith('}'):
                        return json.loads(data)
            except requests.exceptions.Timeout:
                pass # 没拿到数据就跳过，返回 None
            except Exception:
                pass

        return None

class TeleopHub:
    def __init__(self, input_src, output_dest):
        self.input_src = input_src
        self.output_dest = output_dest
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.roarm = RoArmController(mode="serial")
        
        # 数据采集缓冲区 (针对 Real-to-Real)
        self.recording = False
        self.trajectory_states = []
        self.trajectory_actions = []
        self.demo_count = 0
        os.makedirs("real_demos", exist_ok=True)

        # 1. 初始化输入源
        if self.input_src == "ps5":
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                raise Exception("未检测到 PS5 手柄，请通过蓝牙连接！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[Input] 已挂载游戏手柄: {self.joystick.get_name()}")
            print("\n🎮 PS5 录制控制指南:")
            print("  - 按 [Share/创建] 键: 保存当前轨迹并开启下一条")
            print("  - 按 [Options/选项] 键: 放弃当前轨迹并重置")
            
        elif self.input_src == "roarm":
            # 💡 核心：当 RoArm 作为输入设备时，必须释放力矩！
            self.roarm.torque_off()
            
        elif self.input_src == "quest3":
            print("[Input] 预留 Quest 3 局域网 UDP 接收器已就绪。")
            # 这里未来可以开启一个本地 UDP 端口，接收 Quest 3 App 发来的手柄位姿

        # 2. 初始化输出源
        if self.output_dest == "roarm":
            # 💡 核心：当 RoArm 作为受控设备时，必须开启力矩！
            self.roarm.torque_on()

    def save_demo(self):
        if len(self.trajectory_actions) > 0:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"real_demos/demo_{self.demo_count}_{timestamp}.hdf5"
            with h5py.File(filename, "w") as f:
                f.create_dataset("actions", data=np.array(self.trajectory_actions))
                f.create_dataset("states", data=np.array(self.trajectory_states))
            print(f"✅ 成功保存真实世界数据！({len(self.trajectory_actions)} 帧) -> {filename}")
            self.demo_count += 1
        self.trajectory_actions.clear()
        self.trajectory_states.clear()

    def run(self):
        print(f"\n🚀 异地中枢启动: {self.input_src.upper()} -> {self.output_dest.upper()}")
        last_x, last_y, last_z = 200, 0, 150 # RoArm 的初始安全物理坐标 (mm)
        
        try:
            while True:
                action = [0.0]*7 # [dx, dy, dz, droll, dpitch, dyaw, gripper]
                
                # --- 1. 读取输入 ---
                if self.input_src == "ps5":
                    pygame.event.pump()
                    action[0] = -self.joystick.get_axis(1) * 0.05 
                    action[1] = -self.joystick.get_axis(0) * 0.05 
                    hat = self.joystick.get_hat(0)
                    action[2] = hat[1] * 0.05 
                    action[4] = -self.joystick.get_axis(5) * 0.1 
                    action[5] = -self.joystick.get_axis(2) * 0.1 
                    r2 = (self.joystick.get_axis(4) + 1) / 2 
                    action[6] = 1.0 if r2 > 0.5 else -1.0

                    # 🔘 监听手柄特殊按键 (数据采集控制)
                    share_btn = self.joystick.get_button(4)   # Share 键
                    options_btn = self.joystick.get_button(6) # Options 键
                    
                    if share_btn:
                        self.save_demo()
                        time.sleep(0.5) # 防误触抖动
                    if options_btn:
                        print("🗑️ 已取消/清空当前轨迹数据。")
                        self.trajectory_actions.clear()
                        self.trajectory_states.clear()
                        time.sleep(0.5)

                elif self.input_src == "roarm":
                    pose = self.roarm.get_pose()
                    if pose:
                        dx = (pose.get('x', last_x) - last_x) * 0.001
                        dy = (pose.get('y', last_y) - last_y) * 0.001
                        dz = (pose.get('z', last_z) - last_z) * 0.001
                        last_x, last_y, last_z = pose.get('x', 0), pose.get('y', 0), pose.get('z', 0)
                        action[0], action[1], action[2] = dx, dy, dz
                        action[6] = 1.0 if pose.get('b', 255) < 100 else -1.0 
                
                # --- 2. 发送输出 & 数据记录 ---
                if self.output_dest == "isaac":
                    msg = ",".join(map(str, action))
                    self.sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
                    
                elif self.output_dest == "roarm" and self.input_src == "ps5":
                    # PS5 控制真实物理机械臂
                    last_x += action[0] * 100 # 转回 mm
                    last_y += action[1] * 100
                    last_z += action[2] * 100
                    
                    # 限制物理死区，防止把 RoArm 撞坏
                    last_x = max(100, min(300, last_x))
                    last_z = max(50, min(250, last_z))
                    
                    gripper_val = 0 if action[6] > 0 else 255
                    self.roarm.move_ik(last_x, last_y, last_z, pitch=0)
                    self.roarm.set_gripper(gripper_val)

                    # 💾 记录真实世界的数据
                    self.trajectory_actions.append(action)
                    self.trajectory_states.append([last_x, last_y, last_z, gripper_val])

                time.sleep(0.02) # 50Hz 控制环

        except KeyboardInterrupt:
            self.roarm.torque_off() # 退出时安全释放力矩
            print("\n已安全断开并释放物理机械臂。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", choices=["ps5", "roarm", "quest3"], required=True)
    parser.add_argument("--output", choices=["isaac", "roarm"], required=True)
    
    # 新增：让用户在命令行指定 RoArm 的连接方式和配置
    parser.add_argument("--roarm_mode", choices=["serial", "wifi"], default="serial", help="RoArm 通信模式")
    parser.add_argument("--roarm_ip", default="192.168.1.50", help="RoArm Wi-Fi 模式下的 IP")
    parser.add_argument("--roarm_port", default="/dev/cu.usbserial-0001", help="RoArm Serial 模式下的端口")
    
    args = parser.parse_args()
    
    # 实例化中枢并传入参数
    hub = TeleopHub(args.input, args.output)
    
    # 在初始化时覆盖 RoArm 的配置
    hub.roarm = RoArmController(
        mode=args.roarm_mode, 
        serial_port=args.roarm_port, 
        wifi_ip=args.roarm_ip
    )
    
    # 重新触发一下初始化逻辑（比如关力矩）
    if hub.input_src == "roarm":
        hub.roarm.torque_off()
    elif hub.output_dest == "roarm":
        hub.roarm.torque_on()

    hub.run()

"""
python mac_teleop_hub.py --input roarm --output isaac --roarm_mode serial

python mac_teleop_hub.py --input roarm --output isaac --roarm_mode wifi --roarm_ip 192.168.1.50
"""