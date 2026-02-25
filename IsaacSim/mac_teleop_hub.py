import argparse
import json
import time
import socket
import serial
import pygame
import requests
#pip install pygame pyserial requests
# ==========================================
# 🔧 核心配置区
# ==========================================
UDP_IP = "192.168.6.53"  # 💥 替换为你 5090 的局域网 IP
UDP_PORT = 8212
ROARM_SERIAL_PORT = '/dev/cu.usbserial-0001' # 替换为 Mac 识别的串口名
ROARM_IP = "192.168.1.50" # 如果用 Wi-Fi 控制 RoArm，替换为 RoArm 的 IP

class RoArmController:
    """全面集成 Waveshare RoArm-M2-S 官方 JSON 指令集
    https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control
    """
    def __init__(self, mode="serial"):
        self.mode = mode
        self.ser = None
        if mode == "serial":
            try:
                self.ser = serial.Serial(ROARM_SERIAL_PORT, 115200, timeout=0.1)
                print(f"[RoArm] 已通过串口连接: {ROARM_SERIAL_PORT}")
            except Exception as e:
                print(f"[RoArm] 串口连接失败: {e}")
        else:
            print(f"[RoArm] 已启用 Wi-Fi HTTP 模式: {ROARM_IP}")

    def send_cmd(self, cmd_dict):
        """发送 JSON 指令"""
        cmd_str = json.dumps(cmd_dict) + "\n"
        if self.mode == "serial" and self.ser:
            self.ser.write(cmd_str.encode('utf-8'))
        elif self.mode == "wifi":
            try:
                requests.get(f"http://{ROARM_IP}/js?json={json.dumps(cmd_dict)}", timeout=0.5)
            except:
                pass

    def move_ik(self, x, y, z, pitch, spd=0):
        """T:101 笛卡尔坐标控制 (单位: mm, 弧度)"""
        self.send_cmd({"T": 101, "x": x, "y": y, "z": z, "t": pitch, "spd": spd})

    def set_gripper(self, val):
        """T:104 夹爪控制 (0完全闭合 - 255完全张开)"""
        self.send_cmd({"T": 104, "b": val})

    def get_pose(self):
        """T:105 获取当前位姿"""
        if self.mode == "serial" and self.ser:
            self.send_cmd({"T": 105})
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('{') and line.endswith('}'):
                try:
                    return json.loads(line)
                except:
                    pass
        return None

class TeleopHub:
    def __init__(self, input_src, output_dest):
        self.input_src = input_src
        self.output_dest = output_dest
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.roarm = RoArmController(mode="serial") # 也可改 wifi
        
        if self.input_src == "ps5":
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                raise Exception("未检测到 PS5 手柄，请通过蓝牙连接！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[Input] 已挂载游戏手柄: {self.joystick.get_name()}")

    def run(self):
        print(f"🚀 中枢启动: {self.input_src.upper()} -> {self.output_dest.upper()}")
        last_x, last_y, last_z = 0, 0, 0
        try:
            while True:
                action = [0.0]*7 # [dx, dy, dz, droll, dpitch, dyaw, gripper]
                
                # --- 1. 读取输入 ---
                if self.input_src == "ps5":
                    pygame.event.pump()
                    # PS5 左摇杆控制 X/Y
                    action[0] = -self.joystick.get_axis(1) * 0.05 # dx (前后)
                    action[1] = -self.joystick.get_axis(0) * 0.05 # dy (左右)
                    # 十字键控制 Z
                    hat = self.joystick.get_hat(0)
                    action[2] = hat[1] * 0.05 # dz (上下)
                    # 右摇杆控制 Pitch/Yaw
                    action[4] = -self.joystick.get_axis(5) * 0.1 # dpitch
                    action[5] = -self.joystick.get_axis(2) * 0.1 # dyaw
                    # 扳机键控制夹爪 (R2)
                    r2 = (self.joystick.get_axis(4) + 1) / 2 # 归一化 0~1
                    action[6] = 1.0 if r2 > 0.5 else -1.0

                elif self.input_src == "roarm":
                    pose = self.roarm.get_pose()
                    if pose:
                        # 计算 RoArm 真实移动的差值 (缩放到 Isaac 的尺度)
                        dx = (pose.get('x', last_x) - last_x) * 0.001
                        dy = (pose.get('y', last_y) - last_y) * 0.001
                        dz = (pose.get('z', last_z) - last_z) * 0.001
                        last_x, last_y, last_z = pose.get('x', 0), pose.get('y', 0), pose.get('z', 0)
                        
                        action[0], action[1], action[2] = dx, dy, dz
                        action[6] = 1.0 if pose.get('b', 255) < 100 else -1.0 # 简易夹爪映射
                
                # --- 2. 发送输出 ---
                if self.output_dest == "isaac":
                    msg = ",".join(map(str, action))
                    self.sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
                    
                elif self.output_dest == "roarm" and self.input_src == "ps5":
                    # PS5 控制真实 RoArm
                    # (由于 RoArm T:101 需要绝对坐标，这里做简单的累加演示)
                    last_x += action[0] * 100 # 转回 mm
                    last_y += action[1] * 100
                    last_z += action[2] * 100
                    gripper_val = 0 if action[6] > 0 else 255
                    self.roarm.move_ik(last_x, last_y, last_z, pitch=0)
                    self.roarm.set_gripper(gripper_val)

                time.sleep(0.02) # 50Hz 控制环

        except KeyboardInterrupt:
            print("已终止遥控。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", choices=["ps5", "roarm"], required=True)
    parser.add_argument("--output", choices=["isaac", "roarm"], required=True)
    args = parser.parse_args()
    hub = TeleopHub(args.input, args.output)
    hub.run()

"""
python mac_teleop_hub.py --input ps5 --output isaac
"""