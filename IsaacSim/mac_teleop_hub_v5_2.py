import argparse
import json
import time
import socket
import serial
import threading
import math
import numpy as np
import os
import requests
import zmq

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

UDP_IP = "192.168.6.53"  
UDP_PORT = 8212
ZMQ_PORT = 8213

# ==========================================
# 🤖 物理机械臂控制器 (带硬件级安全护盾与双向通信)
#https://www.waveshare.com/wiki/RoArm-M2-S_WIFI_Configuration
#https://www.waveshare.com/wiki/RoArm-M2-S_Python_UART_Communication
# ==========================================
class RoArmController:
    def __init__(self, mode="serial", serial_port='/dev/cu.usbserial-0001', wifi_ip="192.168.1.50"):
        self.mode = mode.lower()
        self.serial_port = serial_port
        self.wifi_ip = wifi_ip
        self.ser = None
        self.is_emergency_stopped = False
        self.light_on = True
        
        # 👇 只需要在这里把连杆长度加回来，给 UI 画图用 👇
        self.L1 = 125.0  # 肩部连杆长度 (mm)
        self.L2 = 125.0  # 肘部连杆长度 (mm)
        
        # 坐标管理
        self.target_pose = {"x": 312.0, "y": 0.0, "z": 230.0, "t": 3.14} 
        self.real_pose = {"x": 312.0, "y": 0.0, "z": 230.0, "t": 3.14}
        
        # 发送节流阀
        self.last_send_time = time.time()
        self.SEND_INTERVAL = 0.05  # 限制最高 20Hz 控制频率
        
        # 防卡死监控
        self.stall_counter = 0
        self.STALL_THRESHOLD = 45.0  # 偏差超过 45mm 判定为受阻
        self.STALL_MAX_FRAMES = 5    # 连续 5 次受阻则触发急停
        
        if self.mode == "serial":
            try:
                self.ser = serial.Serial(self.serial_port, 115200, timeout=0.01)
                print(f"🟢 [RoArm] 串口已连接: {self.serial_port}")
                self.reset_home() 
                
                # 开启后台守护线程，监听反馈数据用于防卡死
                self.running = True
                self.recv_thread = threading.Thread(target=self._serial_recv_loop)
                self.recv_thread.daemon = True
                self.recv_thread.start()
                
            except Exception as e:
                print(f"❌ [RoArm] 串口连接失败: {e}")

    def _serial_recv_loop(self):
        """后台持续读取串口反馈，更新真实坐标，防止主线程阻塞"""
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    data = json.loads(line)
                    if "x" in data:
                        self.real_pose["x"] = data["x"]
                        self.real_pose["y"] = data["y"]
                        self.real_pose["z"] = data["z"]
            except:
                pass
            time.sleep(0.01)

    def send_cmd(self, cmd_dict):
        if self.is_emergency_stopped and cmd_dict.get("T") != 210: return 
        cmd_str = json.dumps(cmd_dict) + "\r\n"
        if self.mode == "serial" and self.ser:
            try: self.ser.write(cmd_str.encode('utf-8'))
            except: pass

    def torque_off(self):
        self.send_cmd({"T": 210, "cmd": 0})
        self.is_emergency_stopped = True
        print("🛑 [RoArm] 已切断电机力矩")

    def torque_on(self):
        self.send_cmd({"T": 210, "cmd": 1})
        self.is_emergency_stopped = False
        print("🟢 [RoArm] 已恢复电机力矩")
        
    def toggle_light(self):
        self.light_on = not self.light_on
        self.send_cmd({"T": 114, "led": 255 if self.light_on else 0})

    def emergency_stop(self):
        if not self.is_emergency_stopped:
            print("\n🚨🚨🚨 [RoArm] 硬件受阻或撞击！触发自动急停！ 🚨🚨🚨")
            self.torque_off()
            self.is_emergency_stopped = True

    def reset_home(self):
        print("🏠 [RoArm] 执行开机自检与复位...")
        if self.is_emergency_stopped: self.torque_on()
        
        self.send_cmd({"T": 114, "led": 0})
        time.sleep(0.3)
        self.send_cmd({"T": 114, "led": 255})
        time.sleep(0.3)
        self.send_cmd({"T": 100})
        
        self.target_pose = {"x": 312.0, "y": 0.0, "z": 230.0, "t": 3.14}
        self.stall_counter = 0

    def check_stall(self):
        """防卡死 / 碰撞检测"""
        if self.is_emergency_stopped: return
        
        # 每秒发送 10 次状态查询请求
        if time.time() % 0.1 < 0.02:
            self.send_cmd({"T": 105})
            
        # 计算目标与现实的欧氏距离误差
        err = math.sqrt(
            (self.target_pose["x"] - self.real_pose["x"])**2 + 
            (self.target_pose["y"] - self.real_pose["y"])**2 + 
            (self.target_pose["z"] - self.real_pose["z"])**2
        )
        if err > self.STALL_THRESHOLD:
            self.stall_counter += 1
            if self.stall_counter > self.STALL_MAX_FRAMES:
                self.emergency_stop()
        else:
            self.stall_counter = 0

    def move_ik_safe(self, dx, dy, dz, dt, speed_scale=1.0):
        if self.is_emergency_stopped: return
        
        # 安全速度上限钳制：无论倍率多大，单帧最大步长严格限制
        MAX_STEP = 12.0 
        dx = float(np.clip(dx * speed_scale * 1.5, -MAX_STEP, MAX_STEP))
        dy = float(np.clip(dy * speed_scale * 1.5, -MAX_STEP, MAX_STEP))
        dz = float(np.clip(dz * speed_scale * 1.5, -MAX_STEP, MAX_STEP))
        dt = float(np.clip(dt * speed_scale, -0.4, 0.4))
        
        # 节流阀过滤：如果不动就不发指令
        curr_time = time.time()
        if (abs(dx) < 0.1 and abs(dy) < 0.1 and abs(dz) < 0.1 and abs(dt) < 0.05) or (curr_time - self.last_send_time < self.SEND_INTERVAL):
            return
            
        self.last_send_time = curr_time
        
        # 释放机械臂的完整物理空间
        new_x = np.clip(self.target_pose["x"] + dx, 120.0, 430.0)
        new_y = np.clip(self.target_pose["y"] + dy, -300.0, 300.0)
        new_z = np.clip(self.target_pose["z"] + dz, -10.0, 320.0)  
        new_t = np.clip(self.target_pose["t"] + dt, 1.08, 5.20)

        self.target_pose["x"] = float(new_x)
        self.target_pose["y"] = float(new_y)
        self.target_pose["z"] = float(new_z)
        self.target_pose["t"] = float(new_t)

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
            angle = 135 if val > 128 else 255
            self.send_cmd({"T": 121, "joint": 4, "angle": angle, "spd": 200})

# ==========================================
# 🎮 控制中枢与现代 GUI (支持双UI渲染)
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

        self.roarm = None
        if "roarm" in [self.input_src, self.output_dest]:
            self.roarm = RoArmController(roarm_mode, roarm_port, roarm_ip)

        self.speed_scale = 1.0     
        self.slider_dragging = False
        self.msg_banner = "System Ready"
        self.msg_timer = time.time() + 2.0
        self.last_btns = [False] * 20
        
        # 按钮矩形 (RoArm UI 使用)
        self.btn_light = pygame.Rect(40, 350, 160, 50)
        self.btn_reset = pygame.Rect(220, 350, 160, 50)
        self.btn_stop = pygame.Rect(40, 420, 340, 50)

        self.init_pygame()

    def load_font(self, size, is_bold=False):
        mac_fonts = ["/System/Library/Fonts/PingFang.ttc", "/System/Library/Fonts/STHeiti Light.ttc"]
        for f in mac_fonts:
            if os.path.exists(f): return pygame.font.Font(f, size)
        return pygame.font.SysFont("arial", size, bold=is_bold)

    def init_pygame(self):
        pygame.init()
        # 判断窗口大小
        if self.output_dest == "roarm":
            self.screen = pygame.display.set_mode((900, 600))
        else:
            self.screen = pygame.display.set_mode((850, 600))
        pygame.display.set_caption("🕹️ Embodied AI Shared Autonomy Hub")
        
        self.font_title = pygame.font.SysFont("impact", 42)
        self.font_hdr = pygame.font.SysFont("arial", 22, bold=True)
        self.font_cn = self.load_font(18)    
        self.font_mono = pygame.font.SysFont("menlo", 18)

        self.rect_slider_track = pygame.Rect(50, 530, 300, 10)

        if self.input_src == "ps5":
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("❌ 未检测到手柄！")
                exit()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def handle_ui_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # 处理滑块
                slider_handle = pygame.Rect(50 + (self.speed_scale - 0.1)/4.9 * 300 - 15, 520, 30, 30)
                if slider_handle.collidepoint(event.pos) or self.rect_slider_track.collidepoint(event.pos):
                    self.slider_dragging = True
                
                # 处理 RoArm UI 的按钮点击
                if self.output_dest == "roarm" and self.roarm:
                    if self.btn_light.collidepoint(event.pos):
                        self.roarm.toggle_light()
                        self.msg_banner = "Light Toggled"
                        self.msg_timer = time.time() + 2.0
                    elif self.btn_reset.collidepoint(event.pos):
                        self.roarm.reset_home()
                        self.msg_banner = "Homing Routine Initiated"
                        self.msg_timer = time.time() + 2.0
                    elif self.btn_stop.collidepoint(event.pos):
                        if self.roarm.is_emergency_stopped:
                            self.roarm.torque_on()
                            self.msg_banner = "Motors Engaged"
                        else:
                            self.roarm.torque_off()
                            self.msg_banner = "Emergency Stop! Motors Released"
                        self.msg_timer = time.time() + 2.0

            elif event.type == pygame.MOUSEBUTTONUP:
                self.slider_dragging = False
            elif event.type == pygame.MOUSEMOTION and self.slider_dragging:
                mouse_x = max(50, min(350, event.pos[0]))
                self.speed_scale = 0.1 + ((mouse_x - 50) / 300.0) * 4.9 
        return True

    def run(self):
        clock = pygame.time.Clock()
        self.gripper_closed = False # 初始化夹爪状态为张开
        
        try:
            while True:
                if not self.handle_ui_events(): break
                action = [0.0]*7 
                
                if self.input_src == "ps5":
                    # macOS 下 PS5 手柄的标准轴映射
                    ax_lx = self.joystick.get_axis(0)
                    ax_ly = self.joystick.get_axis(1)
                    ax_rx = self.joystick.get_axis(2)
                    ax_ry = self.joystick.get_axis(3)
                    
                    action[0] = -ax_ly  # 左摇杆 Y -> X 前后伸缩
                    action[1] = -ax_lx  # 左摇杆 X -> Y 左右摆动
                    action[2] = -ax_ry  # 右摇杆 Y -> Z 上下升降 (取代了难用的L1/R1)
                    action[4] = -ax_rx  # 右摇杆 X -> 手腕俯仰
                    
                    # 获取当前所有的按键状态
                    curr_btns = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                    
                    # ❌键 (通常是 button 0) 控制夹爪翻转: 按一下闭合，再按一下张开
                    if curr_btns[0] and not self.last_btns[0]:
                        self.gripper_closed = not self.gripper_closed
                    action[6] = 1.0 if self.gripper_closed else -1.0
                    
                    # 1. 发送高频 UDP (给 Isaac Sim 备用)
                    if self.output_dest == "isaac":
                        msg_str = ",".join([f"{x:.4f}" for x in action])
                        self.udp_sock.sendto(msg_str.encode(), (UDP_IP, UDP_PORT))
                    
                    # 同步按键状态
                    self.last_btns = curr_btns

                # 3. 物理机械臂分流控制
                if self.output_dest == "roarm" and self.roarm:
                    # 这里的参数对应: dx, dy, dz, dt
                    self.roarm.move_ik_safe(action[0]*5, action[1]*5, action[2]*5, action[4]*2, self.speed_scale)
                    self.roarm.check_stall()  # 触发堵转检测
                    
                    curr_gripper_state = 255 if action[6] > 0 else 0
                    if not hasattr(self, 'last_gripper_state'): 
                        self.last_gripper_state = -1 
                    if curr_gripper_state != self.last_gripper_state:
                        self.roarm.set_gripper(curr_gripper_state)
                        self.last_gripper_state = curr_gripper_state

                    # UI 渲染
                    self.draw_roarm_ui(action)

                elif self.output_dest == "isaac":
                    self.draw_isaac_ui(action)

                clock.tick(50)
        except KeyboardInterrupt: pass
        finally:
            if self.roarm: self.roarm.torque_off()
            pygame.quit()


    def draw_roarm_ui(self, action):
        self.screen.fill((24, 24, 37))
        
        self.screen.blit(self.font_title.render("PHYSICAL ROARM-M2-S HUB", True, (250, 179, 135)), (30, 20))
        status_color = (243, 139, 168) if self.roarm.is_emergency_stopped else (166, 227, 161)
        status_txt = "STATUS: EMERGENCY STOP" if self.roarm.is_emergency_stopped else "STATUS: ACTIVE & SAFE"
        self.screen.blit(self.font_hdr.render(status_txt, True, status_color), (30, 70))

        # --- 左侧：硬件控制台 ---
        pygame.draw.rect(self.screen, (49, 50, 68), (30, 120, 360, 200), border_radius=8)
        self.screen.blit(self.font_hdr.render("REAL-TIME KINEMATICS", True, (137, 180, 250)), (45, 130))
        
        # 💡 核心修复：这里改为读取 real_pose (真实反馈坐标)，而不是目标坐标
        pose = self.roarm.real_pose 
        labels = ["X (Forward)", "Y (Lateral)", "Z (Height)", "Wrist Pitch"]
        vals = [pose["x"], pose["y"], pose["z"], pose["t"]]
        for i, (lbl, val) in enumerate(zip(labels, vals)):
            y_pos = 170 + i * 35
            self.screen.blit(self.font_mono.render(lbl, True, (186, 194, 222)), (45, y_pos))
            self.screen.blit(self.font_mono.render(f"{val:>7.2f}", True, (166, 227, 161)), (250, y_pos))

        # 按钮绘制
        btn_c1, btn_c2, btn_c3 = (69, 71, 90), (137, 180, 250), (243, 139, 168)
        pygame.draw.rect(self.screen, btn_c1, self.btn_light, border_radius=8)
        pygame.draw.rect(self.screen, btn_c2 if self.roarm.light_on else btn_c1, self.btn_light, border_radius=8, width=2)
        txt_l = self.font_cn.render("💡 切换灯光", True, (205, 214, 244))
        self.screen.blit(txt_l, txt_l.get_rect(center=self.btn_light.center))

        pygame.draw.rect(self.screen, btn_c1, self.btn_reset, border_radius=8)
        txt_r = self.font_cn.render("🏠 归零复位", True, (205, 214, 244))
        self.screen.blit(txt_r, txt_r.get_rect(center=self.btn_reset.center))

        pygame.draw.rect(self.screen, btn_c3 if not self.roarm.is_emergency_stopped else btn_c1, self.btn_stop, border_radius=8)
        txt_s = self.font_cn.render("🛑 紧急切断/恢复力矩", True, (24, 24, 37) if not self.roarm.is_emergency_stopped else (205, 214, 244))
        self.screen.blit(txt_s, txt_s.get_rect(center=self.btn_stop.center))

        # --- 右侧：2D 可视化机械臂真实姿态 ---
        canvas_rect = pygame.Rect(420, 120, 450, 350)
        pygame.draw.rect(self.screen, (30, 30, 46), canvas_rect, border_radius=12)
        
        origin_x, origin_y = canvas_rect.centerx - 80, canvas_rect.bottom - 40
        L1_px, L2_px = 120, 120 
        
        # 提取真实坐标用于渲染
        tx = max(0.1, pose["x"]) 
        tz = pose["z"] - 80 
        d = math.sqrt(tx**2 + tz**2)
        
        # 只有在坐标合理时才渲染骨骼，防止刚启动时坐标为0闪烁
        if d > 10.0:
            d = min(d, self.roarm.L1 + self.roarm.L2 - 1) 
            try:
                cos_elbow = (self.roarm.L1**2 + self.roarm.L2**2 - d**2) / (2 * self.roarm.L1 * self.roarm.L2)
                elbow_rad = math.acos(np.clip(cos_elbow, -1.0, 1.0))
                alpha = math.atan2(tz, tx)
                cos_shoulder = (self.roarm.L1**2 + d**2 - self.roarm.L2**2) / (2 * self.roarm.L1 * d)
                shoulder_rad = alpha + math.acos(np.clip(cos_shoulder, -1.0, 1.0))
                
                elbow_x = origin_x + L1_px * math.cos(shoulder_rad)
                elbow_y = origin_y - L1_px * math.sin(shoulder_rad)
                wrist_x = elbow_x + L2_px * math.cos(shoulder_rad - elbow_rad)
                wrist_y = elbow_y - L2_px * math.sin(shoulder_rad - elbow_rad)

                pygame.draw.line(self.screen, (88, 91, 112), (origin_x, origin_y+20), (origin_x, origin_y), 15)
                pygame.draw.line(self.screen, (137, 180, 250), (origin_x, origin_y), (elbow_x, elbow_y), 10)
                pygame.draw.line(self.screen, (166, 227, 161), (elbow_x, elbow_y), (wrist_x, wrist_y), 8)
                
                pygame.draw.circle(self.screen, (249, 226, 175), (int(origin_x), int(origin_y)), 12)
                pygame.draw.circle(self.screen, (249, 226, 175), (int(elbow_x), int(elbow_y)), 10)
                pygame.draw.circle(self.screen, (243, 139, 168), (int(wrist_x), int(wrist_y)), 8)
            except ValueError:
                pass 

        self.draw_bottom_slider()
        pygame.display.flip()

    # --- 渲染 Isaac Sim 原有界面 ---
    def draw_isaac_ui(self, action):
        self.screen.fill((24, 24, 37)) 
        self.screen.blit(self.font_title.render("ISAAC SIM TELEOP HUB", True, (137, 180, 250)), (30, 20))
        # （此处省略了你原本的左侧提示和右侧遥测数据，由于篇幅原因精简渲染，你可以把原本的 draw_ui 贴回这里）
        self.draw_bottom_slider()
        pygame.display.flip()


    def draw_bottom_slider(self):
        self.screen.blit(self.font_hdr.render("SPEED MULTIPLIER", True, (148, 226, 213)), (30, 495))
        self.screen.blit(self.font_hdr.render(f"{self.speed_scale:.1f}x", True, (205, 214, 244)), (250, 495))
        pygame.draw.rect(self.screen, (88, 91, 112), self.rect_slider_track, border_radius=5)
        fill_w = (self.speed_scale - 0.1)/4.9 * 300
        pygame.draw.rect(self.screen, (137, 180, 250), (50, 530, fill_w, 10), border_radius=5)
        pygame.draw.circle(self.screen, (245, 194, 231), (50 + int(fill_w), 535), 10)

        if time.time() < self.msg_timer:
            pygame.draw.rect(self.screen, (137, 180, 250), (480, 490, 390, 50), border_radius=8)
            txt = self.font_cn.render(self.msg_banner, True, (24, 24, 37))
            self.screen.blit(txt, txt.get_rect(center=(675, 515)))

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
python IsaacSim/mac_teleop_hub_v5_1.py --input ps5 --output roarm --roarm_mode serial --roarm_port /dev/cu.usbserial-210
"""