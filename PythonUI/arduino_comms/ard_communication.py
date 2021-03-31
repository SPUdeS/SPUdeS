import pyfirmata
from time import sleep

class ard_communication:
    """ Class that lets us control the motors with Communication to an Arduino with pyfirmata"""

    def __init__(self):
        self.board = pyfirmata.ArduinoMega('COM3')  # port = 'COM3'
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
        self.board.digital[self.pin1].mode = pyfirmata.SERVO
        self.board.digital[self.pin2].mode = pyfirmata.SERVO
        self.board.digital[self.pin3].mode = pyfirmata.SERVO
        self.board.digital[self.pin4].mode = pyfirmata.SERVO
        self.board.digital[self.pin5].mode = pyfirmata.SERVO
        self.board.digital[self.pin6].mode = pyfirmata.SERVO

    # Custom angle to set Servo motor angle
    def setServoAngle(self, angle, isRad=0):
        #angle_int = int(angle)
        if isRad:
            rad_to_deg = 180.0/3.1416
        else:
            rad_to_deg = 1

        for i in range(len(angle)):
            self.board.digital[self.pin1].write(180.0-angle[i][0]*rad_to_deg)
            self.board.digital[self.pin2].write(angle[i][1]*rad_to_deg)
            self.board.digital[self.pin3].write(180.0 - angle[i][2]*rad_to_deg)
            self.board.digital[self.pin4].write(angle[i][3]*rad_to_deg)
            self.board.digital[self.pin5].write(180.0 - angle[i][4]*rad_to_deg)
            self.board.digital[self.pin6].write(angle[i][5]*rad_to_deg)
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
# if __name__ == "__main__":
#     # Initialize ard_communication class
#     arduino_coms = ard_communication()
#     angle = [[1, 2, 1, 2, 1, 2], [1.5, 1.6, 1.7, 1.8, 1.9, 1.1]]
#     arduino_coms.setServoAngle(angle, 1)
#     print(arduino_coms.board.digital[arduino_coms.pin1].read())
#     print(arduino_coms.board.digital[arduino_coms.pin2].read())
#     print(arduino_coms.board.digital[arduino_coms.pin3].read())
#     print(arduino_coms.board.digital[arduino_coms.pin4].read())
#     print(arduino_coms.board.digital[arduino_coms.pin5].read())
#     print(arduino_coms.board.digital[arduino_coms.pin6].read())

