import pygame
import serial

import time


transmitter = serial.Serial("N.A.", 9600)



pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controler found")
    quit()

controler = pygame.joystick.Joystick(0)

print(controler.get_name())


while True:
    pygame.event.pump()

    lx = controler.get_axis(0)
    ly = controler.get_axis(1)
    rx = controler.get_axis(2)
    ry = controler.get_axis(3)

    message = f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f}\n"

    print(message)

    transmitter.write(message.encode())



    time.sleep(0.02)