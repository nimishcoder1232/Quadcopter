import pygame
import serial
import time

transmitter = serial.Serial("COM5", 9600)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print('no controller')
    quit()

controller = pygame.joystick.Joystick(0)
controller.init()

print(controller.get_name())

def deadband(x, d=0.2):
    return 0 if abs(x) < d else x

ABORT_BUTTON = 8  # 👈 changed to button 8

while True:
    pygame.event.pump()

    # 🛑 ABORT CHECK
    if controller.get_button(ABORT_BUTTON):
        print("ABORT PRESSED")
        transmitter.write(b"ABORT\n")
        break

    lx = deadband(controller.get_axis(0))
    ly = deadband(-controller.get_axis(1))
    rx = deadband(controller.get_axis(2))
    ry = deadband(-controller.get_axis(3))

    message = f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f}\n"

    print(message)
    transmitter.write(message.encode())

    time.sleep(0.02)

transmitter.close()
pygame.quit()