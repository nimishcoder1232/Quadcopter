import pygame
import serial

import time

transmitter = serial.Serial("COM5", 9600)



pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print('no controler')
    quit()

controler = pygame.joystick.Joystick(0)

print(controler.get_name())

def deadband(x, d=0.2):
    if abs(x) < d:
        return 0
    else:
        return x

while True:
    pygame.event.pump()

    lx = deadband(controler.get_axis(0))
    ly = deadband(-(controler.get_axis(1)))
    rx = deadband(controler.get_axis(2))
    ry = deadband(-(controler.get_axis(3)))

    message = f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f}\n"

    print(message)

    transmitter.write(message.encode())



    time.sleep(0.02)