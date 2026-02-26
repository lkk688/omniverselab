import argparse
import json
import time
import socket
import os
import pygame
from datetime import datetime

# 隐藏 Pygame 提示
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

# ==========================================
# 🔧 核心配置区
# ==========================================
UDP_IP = "192.168.6.53"  
UDP_PORT = 8212

class TeleopHub:
    def __init__(self, input_src, output_dest):
        self.input_src = input_src
        self.output_dest = output_dest
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        pygame.init()
        self.screen = pygame.display.set_mode((600, 450))
        pygame.display.set_caption("🕹️ PS5 控制中枢 (Final Fix)")
        
        self.font_path = "/System/Library/Fonts/STHeiti Light.ttc"
        if not os.path.exists(self.font_path):
            self.font_path = pygame.font.get_default_font()
        
        self.font_title = pygame.font.Font(self.font_path, 28)
        self.font_status = pygame.font.Font(self.font_path, 18)
        self.font_msg = pygame.font.Font(self.font_path, 24)

        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise Exception("❌ 未检测到手柄！")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.last_msg = ""
        self.msg_expiry = 0
        # 💥 统一初始化状态变量
        self.last_btn0 = False 
        self.last_btn1 = False

    def show_message(self, text, duration=1.5):
        self.last_msg = text
        self.msg_expiry = time.time() + duration

    def run(self):
        print(f"🚀 启动中枢: {self.input_src} -> {self.output_dest}")
        clock = pygame.time.Clock()
        time.sleep(1.0)
        
        try:
            while True:
                pygame.event.pump()
                cmd_flag = 0.0 # 每一帧强制重置为 0
                
                # 1. 读取 7 维动作
                action = [0.0] * 7
                action[0] = -self.joystick.get_axis(1) * 0.05
                action[1] = -self.joystick.get_axis(0) * 0.05
                dz = 0.0
                if self.joystick.get_button(9): dz = -0.05
                if self.joystick.get_button(10): dz = 0.05
                action[2] = dz
                action[4] = -self.joystick.get_axis(3) * 0.1
                action[5] = -self.joystick.get_axis(2) * 0.1
                r2 = (self.joystick.get_axis(5) + 1) / 2
                action[6] = 1.0 if r2 > 0.5 else -1.0

                # 2. 💥 修正按键逻辑 (使用统一的变量名)
                curr_btn0 = bool(self.joystick.get_button(0)) # ❌ 键
                curr_btn1 = bool(self.joystick.get_button(1)) # ○ 键

                if curr_btn0 and not self.last_btn0:
                    cmd_flag = 1.0
                    self.show_message("✅ 发送保存 (SAVE)")
                    print("DEBUG: 触发保存脉冲")
                elif curr_btn1 and not self.last_btn1:
                    cmd_flag = -1.0
                    self.show_message("🗑️ 发送重置 (RESET)")
                    print("DEBUG: 触发重置脉冲")
                
                # 更新状态记录，防止连发
                self.last_btn0 = curr_btn0
                self.last_btn1 = curr_btn1

                # 3. 发送 8 维数据包
                full_packet = action + [cmd_flag]
                msg_str = ",".join([f"{x:.4f}" for x in full_packet])

                #full_packet = list(action) + [cmd_flag]
                #msg_str = ",".join([f"{x:.4f}" for x in full_packet])
                
                # 💥 发送前，我们在 Mac 终端强行打印出我们要发的包！
                print(f"[{time.time():.2f}] 正在发送: {msg_str}")
                self.sock.sendto(msg_str.encode(), (UDP_IP, UDP_PORT))

                self.draw_ui(action, curr_btn0, curr_btn1)
                clock.tick(50)

        except KeyboardInterrupt:
            pygame.quit()

    def draw_ui(self, action, b0, b1):
        self.screen.fill((30, 35, 45))
        title = self.font_title.render("PS5 具身智能遥控终端", True, (0, 255, 180))
        self.screen.blit(title, (30, 30))

        # 实时显示坐标
        coord_str = f"X:{action[0]:.2f} Y:{action[1]:.2f} Z:{action[2]:.2f} Grip:{action[6]:.1f}"
        self.screen.blit(self.font_status.render(coord_str, True, (200, 200, 200)), (30, 80))

        # 指示灯
        col0 = (0, 255, 0) if b0 else (50, 80, 50)
        pygame.draw.circle(self.screen, col0, (50, 150), 12)
        self.screen.blit(self.font_status.render("× 键 (保存轨迹)", True, (255, 255, 255)), (75, 140))

        col1 = (255, 0, 0) if b1 else (80, 50, 50)
        pygame.draw.circle(self.screen, col1, (50, 190), 12)
        self.screen.blit(self.font_status.render("○ 键 (重置环境)", True, (255, 255, 255)), (75, 180))

        # 弹出消息
        if time.time() < self.msg_expiry:
            pygame.draw.rect(self.screen, (0, 100, 200), (0, 380, 600, 70))
            txt = self.font_msg.render(self.last_msg, True, (255, 255, 255))
            self.screen.blit(txt, txt.get_rect(center=(300, 415)))

        pygame.display.flip()

if __name__ == "__main__":
    hub = TeleopHub("ps5", "isaac")
    hub.run()