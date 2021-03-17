import pyfirmata
from time import sleep

class ard_communication:
    """ Class that lets us control the motors with Communication to an Arduino with pyfirmata"""

    def __init__(self):
        self.board = pyfirmata.Arduino('COM3')  # port = 'COM3'
        self.homingAngle = 5
        self.pin1 = 7
        self.pin2 = 8
        self.pin3 = 11
        self.pin4 = 12
        self.pin5 = 5
        self.pin6 = 3
        self.max_angle = 180
        self.min_angle = 0
        self.setUpMotors()


    def setUpMotors(self):
        self.board.digital[self.pin1].mode = pyfirmata.PWM
        self.board.digital[self.pin2].mode = pyfirmata.PWM
        self.board.digital[self.pin3].mode = pyfirmata.PWM
        self.board.digital[self.pin4].mode = pyfirmata.PWM
        self.board.digital[self.pin5].mode = pyfirmata.PWM
        self.board.digital[self.pin6].mode = pyfirmata.PWM

    # Custom angle to set Servo motor angle
    def setServoAngle(self, angle):
        #angle_int = int(angle)
        self.board.analog.write[self.pin1].write(180.0-angle[1])
        self.board.analog.write[self.pin2].write(angle[2])
        self.board.analog.write[self.pin3].write(180.0 - angle[3])
        self.board.analog.write[self.pin4].write(angle[4])
        self.board.analog.write[self.pin5].write(180.0 - angle[5])
        self.board.analog.write[self.pin6].write(angle[6])
        sleep(0.015)

    def goToHomePosition(self):
        self.setServoAngle(self.homingAngle)
        print("HOMING NOW")

    def goToUpDownPosition(self):
        self.setServoAngle(self.homingAngle)
        for i in range(self.homingAngle, max):
            self.setServoAngle(i)
            sleep(0.5)
        for j in range(max, min):
            self.setServoAngle(j)
            sleep(0.5)
        for k in range(min, self.homingAngle):
            self.setServoAngle(k)
            sleep(0.5)

    def goUpPosition(self):
        self.setServoAngle(self.homingAngle)
        for i in range(self.homingAngle, max):
            self.setServoAngle(i)
            sleep(0.5)

    def goUpAndDown(self, angle):
        self.setServoAngle(angle)
        sleep(0.015)

    ### FOR TESTING ONLY ###
    #while True:
        #angle = input("Type angle")
        #goToUpDownPosition()

        #goUP(angle)
        #setServoAngle(angle)
        # goToHomePosition()
