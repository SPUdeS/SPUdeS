import pyfirmata
from time import sleep

board = pyfirmata.Arduino('COM3')
print("Communication Successfully started")

while True:

    # Set mode of the pin 13 as SERVO
    pin1 = 7
    pin2 = 9
    board.digital[pin1].mode = pyfirmata.SERVO
    board.digital[pin2].mode = pyfirmata.SERVO


    # Custom angle to set Servo motor angle
    def setServoAngle(angle):
        board.digital[pin1].write(angle)
        board.digital[pin2].write(angle)
        sleep(0.015)

    # Testing the function by rotating motor in both direction
    while True:
        angle = input("Type angle")
        setServoAngle(angle)
