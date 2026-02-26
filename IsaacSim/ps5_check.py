import pygame
pygame.init()
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()
try:
    while True:
        pygame.event.pump()
        # 打印所有按钮的状态
        btns = [j.get_button(i) for i in range(j.get_numbuttons())]
        print(f"当前所有按键状态: {btns}", end="\r")
except:
    pygame.quit()