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
import zmq

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

UDP_IP = "192.168.6.53"  
UDP_PORT = 8212
ZMQ_PORT = 8213

# ==========================================
# 🤖 物理机械臂控制器 (带硬件级安全护盾)
# ==========================================
import json
import time
import math
import numpy as np
import serial
import requests

class RoArmController:
    def __init__(self, mode="serial", serial_port='/dev/cu.usbserial-0001', wifi_ip="192.168.1.50"):
        self.mode = mode.lower()
        self.serial_port = serial_port
        self.wifi_ip = wifi_ip
        self.ser = None
        self.is_emergency_stopped = False
        
        # 官方默认的复位中心坐标 (x:312, z:230)
        self.target_pose = {"x": 312.0, "y": 0.0, "z": 230.0, "t": 3.14} 
        
        # 发送节流阀，防止手柄 50Hz 的信号冲爆串口
        self.last_send_time = time.time()
        self.SEND_INTERVAL = 0.05  # 限制最高 20Hz 
        
        if self.mode == "serial":
            try:
                self.ser = serial.Serial(self.serial_port, 115200, timeout=0.05)
                print(f"🟢 [RoArm] 串口已连接: {self.serial_port}")
                self.reset_home() 
            except Exception as e:
                print(f"❌ [RoArm] 串口连接失败: {e}")

    def send_cmd(self, cmd_dict):
        if self.is_emergency_stopped and cmd_dict.get("T") != 210: return 
        
        # 组装带换行符的 JSON 字符串
        cmd_str = json.dumps(cmd_dict) + "\r\n"
        
        if self.mode == "serial" and self.ser:
            try: 
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e: 
                print(f"串口发送异常: {e}")

    def torque_off(self): self.send_cmd({"T": 210, "cmd": 0})
    def torque_on(self): self.send_cmd({"T": 210, "cmd": 1})
    
    def reset_home(self):
        print("🏠 [RoArm] 执行开机自检与复位...")
        self.send_cmd({"T": 114, "led": 0})
        time.sleep(0.5)
        self.send_cmd({"T": 114, "led": 255})
        time.sleep(0.5)
        
        # 硬件原生复位
        self.send_cmd({"T": 100})
        time.sleep(1.5)
        
        # 将内部坐标同步到微雪官方复位点
        self.target_pose = {"x": 312.0, "y": 0.0, "z": 230.0, "t": 3.14}

    def move_ik_safe(self, dx, dy, dz, dt, max_step=10.0):
        if self.is_emergency_stopped: return
        
        curr_time = time.time()
        if curr_time - self.last_send_time < self.SEND_INTERVAL:
            return
        
        # 死区过滤
        if abs(dx) < 0.1 and abs(dy) < 0.1 and abs(dz) < 0.1 and abs(dt) < 0.1:
            return
            
        self.last_send_time = curr_time
        
        dx = float(np.clip(dx, -max_step, max_step))
        dy = float(np.clip(dy, -max_step, max_step))
        dz = float(np.clip(dz, -max_step, max_step))
        dt = float(np.clip(dt, -0.5, 0.5))
        
        # 更新坐标，并框定在物理安全边界内
        new_x = np.clip(self.target_pose["x"] + dx, 150.0, 350.0)
        new_y = np.clip(self.target_pose["y"] + dy, -250.0, 250.0)
        new_z = np.clip(self.target_pose["z"] + dz, 100.0, 300.0)  
        new_t = np.clip(self.target_pose["t"] + dt, 1.08, 5.20)

        self.target_pose["x"] = float(new_x)
        self.target_pose["y"] = float(new_y)
        self.target_pose["z"] = float(new_z)
        self.target_pose["t"] = float(new_t)

        # 💡 核心修复：使用 1041 并且彻底移除 spd 参数！
        cmd = {
            "T": 1041, 
            "x": round(self.target_pose["x"], 2), 
            "y": round(self.target_pose["y"], 2), 
            "z": round(self.target_pose["z"], 2), 
            "t": round(self.target_pose["t"], 2)
        }
        self.send_cmd(cmd)

    def set_gripper(self, val):
        if not self.is_emergency_stopped: 
            # 夹爪角度
            angle = 135 if val > 128 else 255
            self.send_cmd({"T": 121, "joint": 4, "angle": angle, "spd": 200})


# ==========================================
# 🎮 控制中枢与现代 GUI
# ==========================================
class TeleopHub:
    def __init__(self, input_src, output_dest, roarm_mode, roarm_port, roarm_ip, isaac_mode):
        self.input_src = input_src
        self.output_dest = output_dest
        self.isaac_mode = isaac_mode 
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        if self.output_dest == "isaac" and self.isaac_mode == "shared":
            self.zmq_ctx = zmq.Context()
            self.cmd_sock = self.zmq_ctx.socket(zmq.REQ)
            self.cmd_sock.setsockopt(zmq.RCVTIMEO, 2000)
            self.cmd_sock.connect(f"tcp://{UDP_IP}:{ZMQ_PORT}")
            self.offset_x, self.offset_y, self.offset_z = 0.0, 0.0, 0.12

        self.roarm = None
        if "roarm" in [self.input_src, self.output_dest]:
            self.roarm = RoArmController(roarm_mode, roarm_port, roarm_ip)
            if self.input_src == "roarm": self.roarm.torque_off()

        self.speed_scale = 1.0     
        self.slider_dragging = False
        self.msg_banner = "System Ready"
        self.msg_timer = time.time() + 2.0
        self.last_btns = [False] * 20
        self.init_pygame()

    def load_font(self, size, is_bold=False):
        """精准加载 macOS 完美中文字体"""
        mac_fonts = [
            "/System/Library/Fonts/PingFang.ttc",
            "/System/Library/Fonts/STHeiti Light.ttc",
            "/System/Library/Fonts/Hiragino Sans GB.ttc"
        ]
        for f in mac_fonts:
            if os.path.exists(f): return pygame.font.Font(f, size)
        return pygame.font.SysFont("arial", size, bold=is_bold)

    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((850, 600))
        pygame.display.set_caption("🕹️ Embodied AI Shared Autonomy Hub")
        
        # 字体矩阵
        self.font_title = pygame.font.SysFont("impact", 42)
        self.font_hdr = pygame.font.SysFont("arial", 22, bold=True)
        self.font_cn = self.load_font(18)    # 专门用于中文渲染
        self.font_cn_sm = self.load_font(14) # 小号中文
        self.font_mono = pygame.font.SysFont("menlo", 18)

        self.rect_slider_track = pygame.Rect(50, 530, 300, 10)

        if self.input_src == "ps5":
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("❌ 未检测到手柄！")
                exit()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def show_msg(self, text):
        self.msg_banner = text
        self.msg_timer = time.time() + 3.0
        print(f"> {text}")

    def send_command(self, cmd_str):
        if self.output_dest != "isaac" or self.isaac_mode != "shared": return
        try:
            self.cmd_sock.send_string(cmd_str)
            reply = self.cmd_sock.recv_string()
            self.show_msg(f"✅ 5090: {reply}")
        except zmq.error.Again:
            self.show_msg(f"❌ 通信超时 ({cmd_str})")
            self.cmd_sock.close()
            self.cmd_sock = self.zmq_ctx.socket(zmq.REQ)
            self.cmd_sock.setsockopt(zmq.RCVTIMEO, 2000)
            self.cmd_sock.connect(f"tcp://{UDP_IP}:{ZMQ_PORT}")

    def handle_ui_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                slider_handle = pygame.Rect(50 + (self.speed_scale - 0.1)/4.9 * 300 - 15, 520, 30, 30)
                if slider_handle.collidepoint(event.pos) or self.rect_slider_track.collidepoint(event.pos):
                    self.slider_dragging = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.slider_dragging = False
            elif event.type == pygame.MOUSEMOTION and self.slider_dragging:
                mouse_x = max(50, min(350, event.pos[0]))
                self.speed_scale = 0.1 + ((mouse_x - 50) / 300.0) * 4.9 
        return True

    def run(self):
        clock = pygame.time.Clock()
        try:
            while True:
                if not self.handle_ui_events(): break
                action = [0.0]*7 
                
                if self.input_src == "ps5":
                    action[0] = -self.joystick.get_axis(1) * 0.05 * self.speed_scale
                    action[1] = -self.joystick.get_axis(0) * 0.05 * self.speed_scale
                    if self.joystick.get_button(9): action[2] = -0.05 * self.speed_scale
                    if self.joystick.get_button(10): action[2] = 0.05 * self.speed_scale
                    action[4] = -self.joystick.get_axis(3) * 0.1 * self.speed_scale
                    action[5] = -self.joystick.get_axis(2) * 0.1 * self.speed_scale
                    action[6] = 1.0 if ((self.joystick.get_axis(5) + 1) / 2) > 0.5 else -1.0

                    # 1. 发送高频 UDP 纯动作流
                    if self.output_dest == "isaac":
                        msg_str = ",".join([f"{x:.4f}" for x in action])
                        self.udp_sock.sendto(msg_str.encode(), (UDP_IP, UDP_PORT))

                    # 2. 读取按键 (触发 ZMQ)
                    curr_btns = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                    if self.output_dest == "isaac" and self.isaac_mode == "shared":
                        if curr_btns[3] and not self.last_btns[3]:   self.send_command("SEMI_HOVER") # △
                        elif curr_btns[2] and not self.last_btns[2]: self.send_command("SEMI_GRASP") # □
                        elif curr_btns[4] and not self.last_btns[4]: self.send_command("FULL_AUTO")  # Share
                        elif curr_btns[0] and not self.last_btns[0]: self.send_command("SAVE")       # ❌
                        elif curr_btns[1] and not self.last_btns[1]: self.send_command("RESET")      # ○
                    self.last_btns = curr_btns
                    
                    # 取出当前夹爪应该处于的状态
                    curr_gripper_state = 255 if action[6] > 0 else 0

                # 3. 物理机械臂控制
                if self.output_dest == "roarm" and self.roarm:
                    dx = (action[0] / 0.05) * 2.0 * self.speed_scale
                    dy = (action[1] / 0.05) * 2.0 * self.speed_scale
                    dz = (action[2] / 0.05) * 2.0 * self.speed_scale
                    dt = action[4] * 2
                    
                    # 死区/变化量过滤
                    if abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1 or abs(dt) > 0.1:
                        self.roarm.move_ik_safe(dx, dy, dz, dt, max_step=15.0)
                    
                    # 状态改变时才发送夹爪指令
                    if not hasattr(self, 'last_gripper_state'): 
                        self.last_gripper_state = -1 # 强制第一次发送
                    
                    if curr_gripper_state != self.last_gripper_state:
                        self.roarm.set_gripper(curr_gripper_state)
                        self.last_gripper_state = curr_gripper_state

                self.draw_ui(action)
                clock.tick(50)

        except KeyboardInterrupt: pass
        finally:
            if self.roarm: self.roarm.torque_off()
            pygame.quit()

    def draw_ui(self, action):
        self.screen.fill((24, 24, 37)) # 极客深渊色
        
        # 标题栏
        self.screen.blit(self.font_title.render("EMBODIED AI HUB", True, (137, 180, 250)), (30, 20))
        mode_txt = f"MODE: {self.isaac_mode.upper()} | TARGET: {self.output_dest.upper()}"
        self.screen.blit(self.font_hdr.render(mode_txt, True, (166, 227, 161)), (30, 70))

        # --- 左侧：操作手册 (分块对齐) ---
        pygame.draw.rect(self.screen, (49, 50, 68), (30, 110, 420, 180), border_radius=8)
        self.screen.blit(self.font_hdr.render("WORKFLOW: SEMI-AUTO", True, (249, 226, 175)), (45, 120))
        semi_guides = [
            ("🔺 [Triangle]", "Auto Align & Hover (自动寻的并悬停)"),
            ("🕹️ [Sticks]", "Manual Override (接管并手工微调)"),
            ("🟦 [Square]", "Auto Descend & Pick (自动下潜抓取)")
        ]
        for i, (btn, desc) in enumerate(semi_guides):
            self.screen.blit(self.font_mono.render(btn, True, (205, 214, 244)), (45, 160 + i*35))
            self.screen.blit(self.font_cn.render(desc, True, (186, 194, 222)), (190, 160 + i*35))

        pygame.draw.rect(self.screen, (49, 50, 68), (30, 310, 420, 160), border_radius=8)
        self.screen.blit(self.font_hdr.render("WORKFLOW: FULL-AUTO / SYS", True, (245, 194, 231)), (45, 320))
        data_guides = [
            ("🌟 [Share]", "One-Click Full Auto (完全无人工厂)"),
            ("❌ [Cross]", "Save Trajectory (保存成功轨迹)"),
            ("⭕️ [Circle]", "Reset Environment (立刻重置环境)")
        ]
        for i, (btn, desc) in enumerate(data_guides):
            self.screen.blit(self.font_mono.render(btn, True, (205, 214, 244)), (45, 360 + i*35))
            self.screen.blit(self.font_cn.render(desc, True, (186, 194, 222)), (180, 360 + i*35))

        # --- 右侧：UDP 实时遥测 ---
        pygame.draw.rect(self.screen, (30, 30, 46), (480, 110, 340, 260), border_radius=8, width=2)
        self.screen.blit(self.font_hdr.render("LIVE TELEMETRY (UDP)", True, (137, 180, 250)), (500, 125))
        labels = ["X-Axis", "Y-Axis", "Z-Axis", "Pitch", "Yaw", "Gripper"]
        vals = [action[0], action[1], action[2], action[4], action[5], action[6]]
        for i, (lbl, val) in enumerate(zip(labels, vals)):
            y_pos = 175 + i * 35
            self.screen.blit(self.font_mono.render(lbl, True, (186, 194, 222)), (500, y_pos))
            col = (166, 227, 161) if abs(val) > 0.01 else (108, 112, 134)
            self.screen.blit(self.font_mono.render(f"{val:>7.3f}", True, col), (720, y_pos))

        # --- 底部：交互控制 ---
        self.screen.blit(self.font_hdr.render("SPEED MULTIPLIER", True, (148, 226, 213)), (30, 495))
        self.screen.blit(self.font_hdr.render(f"{self.speed_scale:.1f}x", True, (205, 214, 244)), (250, 495))
        pygame.draw.rect(self.screen, (88, 91, 112), self.rect_slider_track, border_radius=5)
        fill_w = (self.speed_scale - 0.1)/4.9 * 300
        pygame.draw.rect(self.screen, (137, 180, 250), (50, 530, fill_w, 10), border_radius=5)
        pygame.draw.circle(self.screen, (245, 194, 231), (50 + int(fill_w), 535), 10)

        # 消息横幅
        if time.time() < self.msg_timer:
            pygame.draw.rect(self.screen, (137, 180, 250), (480, 480, 340, 60), border_radius=8)
            txt = self.font_cn.render(self.msg_banner, True, (24, 24, 37))
            self.screen.blit(txt, txt.get_rect(center=(650, 510)))

        pygame.display.flip()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", choices=["ps5", "roarm"], required=True)
    parser.add_argument("--output", choices=["isaac", "roarm"], required=True)
    parser.add_argument("--isaac_mode", choices=["manual", "shared"], default="shared")
    parser.add_argument("--roarm_mode", choices=["serial", "wifi"], default="wifi")
    parser.add_argument("--roarm_ip", default="192.168.5.79")
    parser.add_argument("--roarm_port", default="/dev/cu.usbserial-210")
    
    args = parser.parse_args()
    hub = TeleopHub(args.input, args.output, args.roarm_mode, args.roarm_port, args.roarm_ip, args.isaac_mode)
    hub.run()

"""
python IsaacSim/mac_teleop_hub_v5.py --input ps5 --output isaac --isaac_mode shared
"""