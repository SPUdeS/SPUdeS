import pyfirmata
from time import sleep

class ard_communication:
    """ Class that lets us control the motors with Communication to an Arduino with pyfirmata"""

    def __init__(self):
        self.board = pyfirmata.Arduino('COM5')  # Check on which port the arduino is connected
        self.alpha0 = -2.7
        self.homingAngle = [[90+self.alpha0, 90+self.alpha0, 90+self.alpha0,
                            90+self.alpha0, 90+self.alpha0, 90+self.alpha0]]
        self.pin1 = 7
        self.pin2 = 9
        self.pin3 = 11
        self.pin4 = 12
        self.pin5 = 5
        self.pin6 = 3
        self.max_angle = 180
        self.min_angle = 0
        self.setUpMotors()
        self.goToHomePosition()


    def setUpMotors(self):
        self.board.digital[self.pin1].mode = pyfirmata.SERVO
        self.board.digital[self.pin2].mode = pyfirmata.SERVO
        self.board.digital[self.pin3].mode = pyfirmata.SERVO
        self.board.digital[self.pin4].mode = pyfirmata.SERVO
        self.board.digital[self.pin5].mode = pyfirmata.SERVO
        self.board.digital[self.pin6].mode = pyfirmata.SERVO

    # Custom angle to set Servo motor angle
    def setServoAngle(self, angle, isRad=1):
        #angle_int = int(angle)
        if isRad:
            rad_to_deg = 180.0/3.1416
            alpha = 90
        else:
            rad_to_deg = 1
            alpha = 0

        for i in range(len(angle)):
            self.board.digital[self.pin1].write(int(alpha + angle[i][0]*rad_to_deg))
            self.board.digital[self.pin2].write(int(180.0-(alpha + angle[i][1]*rad_to_deg)))
            self.board.digital[self.pin3].write(int(alpha + angle[i][2]*rad_to_deg))
            self.board.digital[self.pin4].write(int(180.0-(alpha + angle[i][3]*rad_to_deg)))
            self.board.digital[self.pin5].write(int(alpha + angle[i][4]*rad_to_deg))
            self.board.digital[self.pin6].write(int(180.0-(alpha + angle[i][5]*rad_to_deg)))
            sleep(0.01)

    def goToHomePosition(self):
        self.setServoAngle(self.homingAngle, 0)
        print("HOMING NOW")

    def goToUpDownPosition(self):
        up = []
        down = []
        for i in range(90, 140, 10):
            up.append([i, i, i, i, i, i])
        self.setServoAngle(up,0)
        for i in range(140, 60, 10):
            down.append([i, i, i, i, i, i])
        self.setServoAngle(down, 0)

        self.goToHomePosition()

    def goToTiltsPosition(self):
        tilt = []
        for i in range(90, 170, 10):
            tilt.append([i, i, 90, 90, 90, 90])
        self.setServoAngle(tilt,0)
        self.goToHomePosition()
        for i in range(90, 170, 10):
            tilt.append([90, 90, i, i, 90, 90])
        self.setServoAngle(tilt,0)
        self.goToHomePosition()
        for i in range(90, 170, 10):
            tilt.append([90, 90, 90, 90, i, i])
        self.setServoAngle(tilt,0)
        self.goToHomePosition()


    def goUpPosition(self):
        self.setServoAngle(self.homingAngle)
        for i in range(self.homingAngle, 140):
            self.setServoAngle(i, 0)
            sleep(0.5)

    def getServoAngle(self):
        servo1 = (self.board.digital[self.pin1].read())
        servo2 = (self.board.digital[self.pin2].read())
        servo3 = (self.board.digital[self.pin3].read())
        servo4 = (self.board.digital[self.pin4].read())
        servo5 = (self.board.digital[self.pin5].read())
        servo6 = (self.board.digital[self.pin6].read())

        servoAngles = [servo1, servo2, servo3, servo4, servo5, servo6]
        return servoAngles

    ### FOR TESTING ONLY ###
    #while True:
        #angle = input("Type angle")
        #goToUpDownPosition()

        #goUP(angle)
        #setServoAngle(angle)
        # goToHomePosition()
if __name__ == "__main__":
     # Initialize ard_communication class#

    arduino_coms = ard_communication()
    #arduino_coms.goToHomePosition()
    #print(arduino_coms.getServoAngle())
    #arduino_coms.goToUpDownPosition()
    #arduino_coms.goToTiltsPosition()
    #angle_rad = [[1, 1, 1, 1, 1, 1], [0.4123, 0.4123, 0.4123, 0.4123, 0.4123, 0.4123]]
    #arduino_coms.setServoAngle(angle_rad, 1)
    #angle_deg = [[-190, 190, -190, 190, -190, 190], [90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45], [0, 0, 0, 0, 0, 0]]
    #arduino_coms.setServoAngle(angle_deg,0)
    #print(arduino_coms.getServoAngle())

