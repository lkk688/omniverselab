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
UDP_IP = "192.168.6.53"  # 你的 5090 IP
UDP_PORT = 8212

# ==========================================
# 🤖 物理机械臂控制器 (带硬件级安全护盾)
# ==========================================
class RoArmController:
    def __init__(self, mode="serial", serial_port='/dev/cu.usbserial-0001', wifi_ip="192.168.1.50"):
        self.mode = mode.lower()
        self.serial_port = serial_port
        self.wifi_ip = wifi_ip
        self.ser = None
        
        # 🛡️ 安全状态机
        self.is_emergency_stopped = False
        self.target_pose = {"x": 200, "y": 0, "z": 150, "t": 0} # 默认安全位置
        self.stall_counter = 0
        self.STALL_THRESHOLD = 30.0 # 目标与实际误差 > 30mm 认定为受阻
        self.STALL_MAX_FRAMES = 5   # 连续受阻 5 次触发保护

        if self.mode == "serial":
            try:
                self.ser = serial.Serial(self.serial_port, 115200, timeout=0.05)
                print(f"🟢 [RoArm] 串口已连接: {self.serial_port}")
            except Exception as e:
                print(f"❌ [RoArm] 串口失败: {e}")
                
        elif self.mode == "wifi":
            try:
                requests.get(f"http://{self.wifi_ip}/", timeout=1.0)
                print(f"🟢 [RoArm] Wi-Fi 已连接: {self.wifi_ip}")
            except Exception as e:
                print(f"❌ [RoArm] Wi-Fi 失败: {e}")

    def send_cmd(self, cmd_dict):
        # 急停状态下，只允许发送扭矩控制指令，屏蔽运动指令
        if self.is_emergency_stopped and cmd_dict.get("T") != 210:
            return 
            
        if self.mode == "serial" and self.ser:
            try:
                self.ser.write((json.dumps(cmd_dict) + "\n").encode('utf-8'))
            except: pass
        elif self.mode == "wifi":
            try:
                requests.get(f"http://{self.wifi_ip}/js?json={json.dumps(cmd_dict)}", timeout=0.05)
            except: pass

    def torque_off(self):
        self.send_cmd({"T": 210, "cmd": 0})
        print("🟢 [RoArm] 释放电机力矩 (教导模式)")

    def torque_on(self):
        self.send_cmd({"T": 210, "cmd": 1})
        self.is_emergency_stopped = False
        print("🔴 [RoArm] 锁定电机力矩 (遥控模式)")

    def emergency_stop(self):
        if not self.is_emergency_stopped:
            print("\n🚨 [RoArm] 触发急停！已切断力矩！")
            self.torque_off()
            self.is_emergency_stopped = True

    def reset_home(self):
        if self.is_emergency_stopped:
            self.torque_on() # 恢复力矩
        print("🏠 [RoArm] 安全复位中...")
        self.target_pose = {"x": 200, "y": 0, "z": 150, "t": 0}
        self.send_cmd({"T": 101, "x": 200, "y": 0, "z": 150, "t": 0, "spd": 20}) # 极慢速复位

    def move_ik_safe(self, dx, dy, dz, dt, max_step=10.0):
        if self.is_emergency_stopped: return

        # 1. 增量限幅 (防止倍率过高导致暴走)
        dx = np.clip(dx, -max_step, max_step)
        dy = np.clip(dy, -max_step, max_step)
        dz = np.clip(dz, -max_step, max_step)
        dt = np.clip(dt, -max_step, max_step)
        
        # 2. 更新目标
        self.target_pose["x"] += dx
        self.target_pose["y"] += dy
        self.target_pose["z"] += dz
        self.target_pose["t"] += dt
        
        self.send_cmd({
            "T": 101, "x": self.target_pose["x"], "y": self.target_pose["y"], 
            "z": self.target_pose["z"], "t": self.target_pose["t"], "spd": 0
        })

    def get_pose(self):
        cmd_dict = {"T": 105}
        if self.mode == "serial" and self.ser:
            self.send_cmd(cmd_dict)
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'): return json.loads(line)
            except: pass
        return None

    def check_stall(self):
        """堵转防烧保护：检测真实位置与目标位置的偏差"""
        if self.is_emergency_stopped: return
        real_pose = self.get_pose()
        if real_pose and 'x' in real_pose:
            err = np.sqrt(
                (self.target_pose["x"] - real_pose["x"])**2 +
                (self.target_pose["y"] - real_pose["y"])**2 +
                (self.target_pose["z"] - real_pose["z"])**2
            )
            if err > self.STALL_THRESHOLD:
                self.stall_counter += 1
                if self.stall_counter > self.STALL_MAX_FRAMES:
                    print(f"\n🚨 [RoArm] 检测到严重堵转 (偏差 {err:.1f}mm)！触发保护断电！")
                    self.emergency_stop()
            else:
                self.stall_counter = 0

    def set_gripper(self, val):
        if not self.is_emergency_stopped:
            self.send_cmd({"T": 104, "b": val})

# ==========================================
# 🎮 跨界控制中枢与 GUI
# ==========================================
class TeleopHub:
    def __init__(self, input_src, output_dest, roarm_mode, roarm_port, roarm_ip):
        self.input_src = input_src
        self.output_dest = output_dest
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 数据采集存储
        self.trajectory_states, self.trajectory_actions = [], []
        self.demo_count = 0
        os.makedirs("real_demos", exist_ok=True)

        # 硬件初始化
        self.roarm = None
        if "roarm" in [self.input_src, self.output_dest]:
            self.roarm = RoArmController(roarm_mode, roarm_port, roarm_ip)
            if self.input_src == "roarm": self.roarm.torque_off()

        # GUI 与 交互变量
        self.speed_scale = 1.0     # 默认速度倍率
        self.slider_dragging = False
        self.frame_count = 0       # 用于降低串口读取频率
        self.msg_banner = ""
        self.msg_timer = 0
        self.last_s = False
        self.last_o = False

        self.init_pygame()

    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((700, 500))
        pygame.display.set_caption("🕹️ Embodied AI Teleop Dashboard")
        
        try:
            self.font_title = pygame.font.SysFont("impact", 36)
            self.font_bold = pygame.font.SysFont("arial", 20, bold=True)
            self.font_sm = pygame.font.SysFont("menlo", 16)
        except:
            self.font_title = pygame.font.Font(None, 40)
            self.font_bold = pygame.font.Font(None, 24)
            self.font_sm = pygame.font.Font(None, 20)

        # UI 元素区域定义
        self.rect_slider_track = pygame.Rect(50, 200, 300, 10)
        self.rect_btn_estop = pygame.Rect(50, 260, 120, 50)
        self.rect_btn_reset = pygame.Rect(180, 260, 120, 50)
        self.rect_btn_save = pygame.Rect(310, 260, 120, 50)

        if self.input_src == "ps5":
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("❌ 未检测到手柄！")
                exit()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def show_msg(self, text):
        self.msg_banner = text
        self.msg_timer = time.time() + 2.0
        print(f"> {text}")

    def handle_ui_events(self):
        """处理鼠标点击和拖拽事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
                
            # 鼠标按下
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                # 点击滑杆
                slider_handle = pygame.Rect(50 + (self.speed_scale - 0.1)/4.9 * 300 - 10, 190, 20, 30)
                if slider_handle.collidepoint(mouse_pos) or self.rect_slider_track.collidepoint(mouse_pos):
                    self.slider_dragging = True
                
                # 点击按钮
                if self.rect_btn_estop.collidepoint(mouse_pos):
                    if self.roarm: self.roarm.emergency_stop()
                    self.show_msg("🚨 触发急停 (E-STOP)")
                elif self.rect_btn_reset.collidepoint(mouse_pos):
                    if self.roarm: self.roarm.reset_home()
                    self.show_msg("🏠 触发安全复位 (HOME)")
                elif self.rect_btn_save.collidepoint(mouse_pos):
                    self.show_msg("💾 触发保存指令")
                    return "SAVE" # 特殊返回值

            # 鼠标释放
            elif event.type == pygame.MOUSEBUTTONUP:
                self.slider_dragging = False

            # 鼠标拖拽滑杆
            elif event.type == pygame.MOUSEMOTION:
                if self.slider_dragging:
                    mouse_x = max(50, min(350, event.pos[0]))
                    ratio = (mouse_x - 50) / 300.0
                    self.speed_scale = 0.1 + ratio * 4.9 # 映射到 0.1 ~ 5.0 倍
        return True

    def run(self):
        print(f"\n🚀 Dashboard 就绪: {self.input_src.upper()} -> {self.output_dest.upper()}")
        clock = pygame.time.Clock()
        
        try:
            while True:
                # 1. 刷新 UI 事件
                ui_cmd = self.handle_ui_events()
                if ui_cmd is False: break
                
                cmd_flag = 0.0
                if ui_cmd == "SAVE": cmd_flag = 1.0
                
                action = [0.0]*7 
                
                # 2. 摇杆读取 (结合动态速度倍率)
                if self.input_src == "ps5":
                    # X/Y 控制
                    action[0] = -self.joystick.get_axis(1) * 0.05 * self.speed_scale
                    action[1] = -self.joystick.get_axis(0) * 0.05 * self.speed_scale
                    
                    # Z 控制
                    dz = 0.0
                    if self.joystick.get_button(9): dz = -0.05 * self.speed_scale
                    if self.joystick.get_button(10): dz = 0.05 * self.speed_scale
                    action[2] = dz

                    # 姿态控制
                    action[4] = -self.joystick.get_axis(3) * 0.1 * self.speed_scale
                    action[5] = -self.joystick.get_axis(2) * 0.1 * self.speed_scale
                    
                    # 夹爪
                    r2 = (self.joystick.get_axis(5) + 1) / 2 
                    action[6] = 1.0 if r2 > 0.5 else -1.0

                    # 手柄快捷键
                    curr_s = bool(self.joystick.get_button(0)) # ❌
                    curr_o = bool(self.joystick.get_button(1)) # ○
                    
                    if curr_s and not self.last_s:
                        cmd_flag = 1.0
                        self.show_msg("✅ 手柄触发保存")
                    elif curr_o and not self.last_o:
                        cmd_flag = -1.0
                        self.show_msg("🗑️ 手柄触发重置")
                        if self.roarm: self.roarm.reset_home()
                    
                    self.last_s, self.last_o = curr_s, curr_o

                # 3. 硬件/仿真执行
                if self.output_dest == "isaac":
                    full_action = list(action) + [cmd_flag]
                    msg = ",".join(map(str, full_action))
                    self.sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
                    
                elif self.output_dest == "roarm" and self.roarm:
                    # 将标准化摇杆输入放大为物理机械臂步长 (最大基准限制在 2.0mm + 动态倍率)
                    base_phys_speed = 2.0
                    dx = (action[0] / 0.05) * base_phys_speed * self.speed_scale
                    dy = (action[1] / 0.05) * base_phys_speed * self.speed_scale
                    dz = (action[2] / 0.05) * base_phys_speed * self.speed_scale
                    
                    # 发送安全限幅移动 (极限步长设为 15mm/帧，防止调速过高)
                    self.roarm.move_ik_safe(dx, dy, dz, action[4]*2, max_step=15.0)
                    gripper_val = 255 if action[6] > 0 else 0
                    self.roarm.set_gripper(gripper_val)

                    # 降频进行堵转检测 (5Hz，防止串口堵塞)
                    self.frame_count += 1
                    if self.frame_count % 10 == 0:
                        self.roarm.check_stall()

                # 4. 渲染 GUI
                self.draw_ui(action)
                clock.tick(50)

        except KeyboardInterrupt: pass
        finally:
            if self.roarm: self.roarm.torque_off()
            pygame.quit()
            print("安全退出。")

    def draw_ui(self, action):
        """现代化仪表盘渲染"""
        # 背景色：深邃太空灰
        self.screen.fill((30, 30, 46)) 
        
        # 1. 标题
        self.screen.blit(self.font_title.render("EMBODIED AI DASHBOARD", True, (137, 180, 250)), (30, 20))
        self.screen.blit(self.font_sm.render(f"Source: {self.input_src.upper()} | Target: {self.output_dest.upper()}", True, (166, 173, 200)), (30, 65))

        # 2. 状态读数区 (右侧)
        pygame.draw.rect(self.screen, (49, 50, 68), (400, 100, 260, 210), border_radius=10)
        self.screen.blit(self.font_bold.render("TELEMETRY", True, (205, 214, 244)), (420, 115))
        
        labels = ["X Axis", "Y Axis", "Z Axis", "Pitch", "Yaw", "Gripper"]
        vals = [action[0], action[1], action[2], action[4], action[5], action[6]]
        for i, (lbl, val) in enumerate(zip(labels, vals)):
            y_pos = 150 + i * 25
            self.screen.blit(self.font_sm.render(f"{lbl}:", True, (186, 194, 222)), (420, y_pos))
            # 数值变色
            color = (166, 227, 161) if abs(val) > 0.01 else (108, 112, 134)
            self.screen.blit(self.font_sm.render(f"{val:>6.2f}", True, color), (550, y_pos))

        # 3. 滑杆交互区
        self.screen.blit(self.font_bold.render("SPEED MULTIPLIER", True, (205, 214, 244)), (50, 160))
        self.screen.blit(self.font_sm.render(f"{self.speed_scale:.1f}x", True, (249, 226, 175)), (250, 162))
        
        # 画轨道
        pygame.draw.rect(self.screen, (88, 91, 112), self.rect_slider_track, border_radius=5)
        # 画填充
        fill_width = (self.speed_scale - 0.1)/4.9 * 300
        pygame.draw.rect(self.screen, (137, 180, 250), (50, 200, fill_width, 10), border_radius=5)
        # 画推子
        pygame.draw.circle(self.screen, (245, 194, 231), (50 + int(fill_width), 205), 10)

        # 4. 可点击安全按钮
        btn_estop_color = (243, 139, 168) if self.roarm and self.roarm.is_emergency_stopped else (160, 60, 80)
        pygame.draw.rect(self.screen, btn_estop_color, self.rect_btn_estop, border_radius=8)
        self.screen.blit(self.font_bold.render("E-STOP", True, (24, 24, 37)), (70, 275))

        pygame.draw.rect(self.screen, (137, 220, 235), self.rect_btn_reset, border_radius=8)
        self.screen.blit(self.font_bold.render("RESET", True, (24, 24, 37)), (210, 275))

        pygame.draw.rect(self.screen, (166, 227, 161), self.rect_btn_save, border_radius=8)
        self.screen.blit(self.font_bold.render("SAVE", True, (24, 24, 37)), (345, 275))

        # 5. 消息通知横幅
        if time.time() < self.msg_timer:
            pygame.draw.rect(self.screen, (49, 50, 68), (0, 440, 700, 60))
            txt = self.font_bold.render(self.msg_banner, True, (249, 226, 175))
            self.screen.blit(txt, txt.get_rect(center=(350, 470)))

        pygame.display.flip()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", choices=["ps5", "roarm"], required=True)
    parser.add_argument("--output", choices=["isaac", "roarm"], required=True)
    parser.add_argument("--roarm_mode", choices=["serial", "wifi"], default="serial")
    parser.add_argument("--roarm_ip", default="192.168.1.50")
    parser.add_argument("--roarm_port", default="/dev/cu.usbserial-0001")
    
    args = parser.parse_args()
    hub = TeleopHub(args.input, args.output, args.roarm_mode, args.roarm_port, args.roarm_ip)
    hub.run()