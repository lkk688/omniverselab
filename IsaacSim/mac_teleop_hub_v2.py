import argparse
import json
import time
import socket
import serial
import h5py
import numpy as np
import os
import requests
from datetime import datetime

# 隐藏 Pygame 每次启动时的烦人提示
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# ==========================================
# 🔧 核心配置区
# ==========================================
UDP_IP = "192.168.6.53"  # 💥 注意替换为你 5090 的 IP
UDP_PORT = 8212

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
                print(f"🔄 [RoArm] 正在尝试连接 Wi-Fi: {self.wifi_ip} ...")
                requests.get(f"http://{self.wifi_ip}/", timeout=2.0)
                print(f"🟢 [RoArm] 已成功通过 Wi-Fi 连接: {self.wifi_ip}")
            except Exception as e:
                print(f"❌ [RoArm] Wi-Fi 连接失败: {e}")

    def send_cmd(self, cmd_dict):
        if self.mode == "serial" and self.ser:
            cmd_str = json.dumps(cmd_dict) + "\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception: pass
        elif self.mode == "wifi":
            try:
                url = f"http://{self.wifi_ip}/js?json={json.dumps(cmd_dict)}"
                requests.get(url, timeout=0.05)
            except Exception: pass

    def torque_off(self):
        print("[RoArm] 🟢 已开启教导模式 (释放电机力矩)")
        self.send_cmd({"T": 210, "cmd": 0})

    def torque_on(self):
        print("[RoArm] 🔴 已开启伺服控制模式 (锁定电机力矩)")
        self.send_cmd({"T": 210, "cmd": 1})

    def move_ik(self, x, y, z, pitch, spd=0):
        self.send_cmd({"T": 101, "x": x, "y": y, "z": z, "t": pitch, "spd": spd})

    def set_gripper(self, val):
        self.send_cmd({"T": 104, "b": val})

    def get_pose(self):
        cmd_dict = {"T": 105}
        if self.mode == "serial" and self.ser:
            self.send_cmd(cmd_dict)
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    return json.loads(line)
            except: pass
        elif self.mode == "wifi":
            try:
                res = requests.get(f"http://{self.wifi_ip}/js?json={json.dumps(cmd_dict)}", timeout=0.15)
                if res.status_code == 200:
                    data = res.text.strip()
                    if data.startswith('{') and data.endswith('}'): return json.loads(data)
            except Exception: pass
        return None

class TeleopHub:
    def __init__(self, input_src, output_dest, roarm_mode, roarm_port, roarm_ip):
        self.input_src = input_src
        self.output_dest = output_dest
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.trajectory_states = []
        self.trajectory_actions = []
        self.demo_count = 0
        os.makedirs("real_demos", exist_ok=True)

        self.roarm = None
        if self.input_src == "roarm" or self.output_dest == "roarm":
            self.roarm = RoArmController(mode=roarm_mode, serial_port=roarm_port, wifi_ip=roarm_ip)
            if self.input_src == "roarm": self.roarm.torque_off()
            elif self.output_dest == "roarm": self.roarm.torque_on()

        if self.input_src == "ps5":
            # 💥 核心修改：移除 dummy 驱动，开启真实的 Pygame GUI 窗口！
            pygame.init()
            # 设置一个 500x300 的实体窗口
            self.screen = pygame.display.set_mode((500, 300))
            pygame.display.set_caption("🕹️ PS5 Teleop Hub (请保持窗口激活)")
            # 尝试加载字体用于仪表盘显示
            try:
                self.font = pygame.font.SysFont("menlo", 24)
                self.small_font = pygame.font.SysFont("menlo", 16)
            except:
                self.font = pygame.font.Font(None, 36)
                self.small_font = pygame.font.Font(None, 24)

            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                raise Exception("未检测到 PS5 手柄，请通过蓝牙连接！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[Input] 已挂载游戏手柄: {self.joystick.get_name()}")

    def save_demo(self):
        if len(self.trajectory_actions) > 0:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"real_demos/demo_{self.demo_count}_{timestamp}.hdf5"
            with h5py.File(filename, "w") as f:
                f.create_dataset("actions", data=np.array(self.trajectory_actions))
                f.create_dataset("states", data=np.array(self.trajectory_states))
            print(f"\n✅ 成功保存数据！({len(self.trajectory_actions)} 帧) -> {filename}")
            self.demo_count += 1
        self.trajectory_actions.clear()
        self.trajectory_states.clear()

    def run(self):
        print(f"\n🚀 异地中枢启动: {self.input_src.upper()} -> {self.output_dest.upper()}")
        if self.input_src == "ps5":
            print("⚠️ 注意：请务必用鼠标点击激活弹出的 Pygame 黑框窗口，否则 macOS 会拦截手柄信号！\n")
            
        last_x, last_y, last_z = 200, 0, 150 
        
        try:
            while True:
                action = [0.0]*7 
                
                # --- 1. 读取输入 ---
                if self.input_src == "ps5":
                    # 💥 处理 Pygame 窗口系统事件，防止 macOS 将窗口判定为“无响应”卡死
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            print("\n已关闭调试窗口。")
                            return

                    action[0] = -self.joystick.get_axis(1) * 0.05 
                    action[1] = -self.joystick.get_axis(0) * 0.05 
                    
                    dz = 0.0
                    if self.joystick.get_numbuttons() >= 11:
                        if self.joystick.get_button(9): dz = -0.05
                        if self.joystick.get_button(10): dz = 0.05
                    action[2] = dz

                    if self.joystick.get_numaxes() >= 4:
                        action[4] = -self.joystick.get_axis(3) * 0.1 
                        action[5] = -self.joystick.get_axis(2) * 0.1 
                    
                    if self.joystick.get_numaxes() >= 6:
                        r2 = (self.joystick.get_axis(5) + 1) / 2 
                        action[6] = 1.0 if r2 > 0.5 else -1.0

                    if self.joystick.get_numbuttons() >= 7:
                        if self.joystick.get_button(4):
                            self.save_demo()
                            time.sleep(0.5) 
                        if self.joystick.get_button(6):
                            print("\n🗑️ 已取消当前轨迹。")
                            self.trajectory_actions.clear()
                            self.trajectory_states.clear()
                            time.sleep(0.5)

                    # 新增：第 8 位命令符 (0: 正常, 1: 保存, -1: 重置)
                    cmd_flag = 0.0
                    pygame.event.pump()
                    if self.joystick.get_button(4): # Share 键
                        cmd_flag = 1.0
                        print("\n按下 Share: 发送保存指令...")
                    elif self.joystick.get_button(6): # Options 键
                        cmd_flag = -1.0
                        print("\n按下 Options: 发送重置指令...")

                    # 组装 8 维数据包
                    full_action = list(action) + [cmd_flag]

                    # --- 2. 发送输出 ---
                    if self.output_dest == "isaac":
                        msg = ",".join(map(str, full_action)) # 现在发 8 个数
                        self.sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

                    # ==========================================
                    # 🖥️ 终端与 GUI 仪表盘实时渲染
                    # ==========================================
                    debug_str = f"X:{action[0]:5.2f} Y:{action[1]:5.2f} Z:{action[2]:5.2f} P:{action[4]:5.2f} Y:{action[5]:5.2f} Grip:{action[6]:4.1f}"
                    print(f"\r📡 发送指令: {debug_str}", end="", flush=True)

                    # 1. 清空背景 (深灰蓝色)
                    self.screen.fill((25, 30, 40)) 
                    
                    # 2. 渲染顶部标题与实时数据
                    title_text = self.font.render("🎮 DualSense Telemetry Hub", True, (0, 255, 150))
                    data_text = self.small_font.render(debug_str, True, (255, 255, 0)) # 数据高亮为黄色
                    self.screen.blit(title_text, (20, 20))
                    self.screen.blit(data_text, (20, 60))

                    # 3. 💥 渲染控制指南 (使用分隔线美化)
                    pygame.draw.line(self.screen, (100, 100, 100), (20, 95), (480, 95), 1)
                    
                    guide_lines = [
                        "🕹️ [Left Stick] : Move (X / Y)",
                        "⬆️ [ L1/R1 ] : Move (Z)",
                        "🔄 [Right Stick] : Rotate (Pitch / Yaw)",
                        "✊ [R2] : Gripper",
                        "-----------------------------------------",
                        "💾 [Share] : Save Demo",
                        "🗑️ [Options] : Reset Demo"
                    ]
                    
                    y_offset = 110
                    for line in guide_lines:
                        # 区分颜色：按键操作用白色，保存/重置用特殊颜色
                        color = (200, 200, 200)
                        if "💾" in line: color = (100, 255, 100)
                        elif "🗑️" in line: color = (255, 100, 100)
                        
                        guide_text = self.small_font.render(line, True, color)
                        self.screen.blit(guide_text, (20, y_offset))
                        y_offset += 25

                    # 4. 底部状态栏
                    pygame.draw.line(self.screen, (100, 100, 100), (20, 310), (480, 310), 1)
                    status_text = self.small_font.render(f"Target: {self.output_dest.upper()} | UDP: {UDP_IP}:{UDP_PORT}", True, (150, 150, 150))
                    self.screen.blit(status_text, (20, 320))
                    
                    # 刷新屏幕
                    pygame.display.flip()

                elif self.input_src == "roarm" and self.roarm:
                    pose = self.roarm.get_pose()
                    if pose:
                        action[0] = (pose.get('x', last_x) - last_x) * 0.001
                        action[1] = (pose.get('y', last_y) - last_y) * 0.001
                        action[2] = (pose.get('z', last_z) - last_z) * 0.001
                        last_x, last_y, last_z = pose.get('x', 0), pose.get('y', 0), pose.get('z', 0)
                        action[6] = 1.0 if pose.get('b', 255) < 100 else -1.0 
                
                # --- 2. 发送输出 ---
                if self.output_dest == "isaac":
                    msg = ",".join(map(str, action))
                    self.sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
                    
                time.sleep(0.02) # 50Hz

        except KeyboardInterrupt:
            if self.roarm: self.roarm.torque_off()
            print("\n已安全退出。")
            pygame.quit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", choices=["ps5", "roarm", "quest3"], required=True)
    parser.add_argument("--output", choices=["isaac", "roarm"], required=True)
    parser.add_argument("--roarm_mode", choices=["serial", "wifi"], default="serial")
    parser.add_argument("--roarm_ip", default="192.168.1.50")
    parser.add_argument("--roarm_port", default="/dev/cu.usbserial-0001")
    
    args = parser.parse_args()
    hub = TeleopHub(args.input, args.output, args.roarm_mode, args.roarm_port, args.roarm_ip)
    hub.run()