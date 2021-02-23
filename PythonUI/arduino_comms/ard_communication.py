import pyfirmata
from time import sleep

board = pyfirmata.Arduino('COM3')  # port = 'COM3'
print("Communication Successfully started")
homingAngle = 5
max = 90
min = 0
pin1 = 7
pin2 = 9
pin3 = 11
pin4 = 13
pin5 = 5
pin6 = 3

def setUpMotors():
    board.digital[pin1].mode = pyfirmata.SERVO
    board.digital[pin2].mode = pyfirmata.SERVO
    board.digital[pin3].mode = pyfirmata.SERVO
    board.digital[pin4].mode = pyfirmata.SERVO
    board.digital[pin5].mode = pyfirmata.SERVO
    board.digital[pin6].mode = pyfirmata.SERVO

# Custom angle to set Servo motor angle
def setServoAngle(angle):
    angle_int = int(angle)
    setUpMotors()
    board.digital[pin1].write(180 - angle_int)
    board.digital[pin2].write(angle_int)
    board.digital[pin3].write(180 - angle_int)
    board.digital[pin4].write(angle_int)
    board.digital[pin5].write(180 - angle_int)
    board.digital[pin6].write(angle_int)
    sleep(0.015)

def goToHomePosition():
    setServoAngle(homingAngle)
    print("HOMING NOW")

def goToUpDownPosition():
    setServoAngle(homingAngle)
    for i in range(homingAngle, max):
        setServoAngle(i)
        sleep(0.5)
    for j in range(max, min):
        setServoAngle(j)
        sleep(0.5)
    for k in range(min, homingAngle):
        setServoAngle(k)
        sleep(0.5)

def goUPandDown(angle):
    setServoAngle(angle)
    sleep(0.015)

### FOR TESTING ONLY ###
#while True:
    #angle = input("Type angle")
    #goToUpDownPosition()

    #goUP(angle)
    #setServoAngle(angle)
    # goToHomePosition()
