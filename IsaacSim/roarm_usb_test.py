import serial
import argparse
import threading
import time

def read_serial():
    while True:
        try:
            # 加上 timeout 后，readline 不会一直死等
            data = ser.readline().decode('utf-8').strip()
            if data:
                print(f"\n[机械臂反馈]: {data}\n请输入JSON指令 > ", end='')
        except Exception as e:
            pass

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., /dev/cu.usbserial-XXXX)')

    args = parser.parse_args()

    print(f"尝试连接串口: {args.port} ...")
    # 添加了 timeout=0.5
    ser = serial.Serial(args.port, baudrate=115200, timeout=0.5, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)
    
    print("串口连接成功！")

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    # 稍微等一下，让接收线程准备好
    time.sleep(1)

    try:
        while True:
            # 增加视觉提示符
            command = input("请输入JSON指令 > ")
            if command.strip():
                ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        print("\n退出程序")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

"""
https://www.waveshare.com/wiki/RoArm-M2-S
pip install pyserial
python serial_simple_ctrl.py /dev/cu.usbserial-210

#控制机械臂获取设备信息的 JSON 指令
{"T":105,"cmd":1}
Return: {"T":1051,"x":312.6374906,"y":-1.438749869,"z":230.8977999,"b":-0.004601942,"s":0.010737866,"e":1.580000212,"t":3.146194596,"torB":28,"torS":44,"torE":40,"torH":28}
空间坐标： x: 312.6, y: -1.4, z: 230.8 (单位是毫米，目前它大概在正前方)
关节弧度： b (底座), s (肩部), e (肘部), t (手部/夹爪)
电机负载： torB, torS 等是当前各关节电机的扭矩负载。

开灯： {"T":114,"led":255}
关灯： {"T":114,"led":0}

回到初始位置 (归零): {"T":100}
解锁电机（现在你可以用手掰动它了）： {"T":210,"cmd":0}
锁定电机（固定在当前位置）： {"T":210,"cmd":1}

坐标控制: {"T":1041,"x":300,"y":80,"z":200,"t":3.14}
关节角度控制: 
控制底座旋转（比如转到 45度）： {"T":121, "joint":1, "angle":45, "spd":1000}
控制手爪开合： {"T":121, "joint":4, "angle":135, "spd":1000}
( joint 的对应关系是：1=底座Base，2=肩部Shoulder，3=肘部Elbow，4=夹爪Wrist/Hand；spd 是移动速度)
"""