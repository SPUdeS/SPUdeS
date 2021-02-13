import pyfirmata
from time import sleep

board = pyfirmata.Arduino('COM3')  # port = 'COM3'
print("Communication Successfully started")
homingAngle = 5
max = 195
min = 0
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

def goToUpDownPosition(max, min):
    for i in range(homingAngle, max):
        setServoAngle(i)
        sleep(0.015)
    for j in range(max, min):
        setServoAngle(j)
        sleep(0.015)
    for k in range(min, homingAngle):
        setServoAngle(k)
        sleep(0.015)

# Testing the function by rotating motor in both direction
# while True:
    # angle = input("Type angle")
    # setServoAngle(angle)
    # goToHomePosition()
