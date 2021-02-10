import pyfirmata
from time import sleep

board = pyfirmata.Arduino('COM3')  # port = 'COM3'
print("Communication Successfully started")
homingAngle = 0
pin1 = 7
pin2 = 9
pin3 = 11
pin4 = 13
pin5 = 2
pin6 = 5

def setUpMotors():
    board.digital[pin1].mode = pyfirmata.SERVO
    board.digital[pin2].mode = pyfirmata.SERVO
    board.digital[pin3].mode = pyfirmata.SERVO
    board.digital[pin4].mode = pyfirmata.SERVO
    board.digital[pin5].mode = pyfirmata.SERVO
    board.digital[pin6].mode = pyfirmata.SERVO

# Custom angle to set Servo motor angle
def setServoAngle(angle):
    setUpMotors()
    board.digital[pin1].write(angle)
    board.digital[pin2].write(angle)
    board.digital[pin3].write(angle)
    board.digital[pin4].write(angle)
    board.digital[pin5].write(angle)
    board.digital[pin6].write(angle)
    sleep(0.015)

def goToHomePosition():
    setServoAngle(homingAngle)

def goToUpDownPosition():
    for i in range(homingAngle, 195):
        setServoAngle(i)
    for j in range(195, 0):
        setServoAngle(j)
    for k in range(0, homingAngle):
        setServoAngle(k)

# Testing the function by rotating motor in both direction
while True:
    # angle = input("Type angle")
    # setServoAngle(angle)
    goToHomePosition()
