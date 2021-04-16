"""This file handles the motor functions to send to Arduino. It uses a library called pyfirmata to communicate with the Arduino and implements ino language.
The functions declared here are used in the Server.py file."""
import pyfirmata
from time import sleep

class ard_communication:
    """The ard_communication class handles all the communication protocol between the Arduino and the Python
    functions using the library pyfirmata. """

    def __init__(self):
        """ The __init__() function sets up the connection to the Arduino via USB, it initializes the Motor to Pin
        connections and some basic set up angles (min, max, homing angles). This function also calls the
        setUpMotors() function to initialize the motor connections with the Arduino. It finally calls the
        gotoHomePosition() function which sends the homing angles command to the motors. """
        self.board = pyfirmata.Arduino('COM3')  # RPI: '/dev/ttyACM0'
        self.alpha0 = -2.7
        self.homingAngle = [[90+self.alpha0, 90+self.alpha0, 90+self.alpha0,
                            90+self.alpha0, 90+self.alpha0, 90+self.alpha0]]
        # Assign a pin number to each motor
        self.motor1 = 7
        self.motor2 = 9
        self.motor3 = 11
        self.motor4 = 12
        self.motor5 = 5
        self.motor6 = 3
        self.max_angle = 180
        self.min_angle = 0
        self.setUpMotors()
        self.goToHomePosition()

    def setUpMotors(self):
        """The setUpMotors() function initializes the pin connections to the servo motors."""
        self.board.digital[self.motor1].mode = pyfirmata.SERVO
        self.board.digital[self.motor2].mode = pyfirmata.SERVO
        self.board.digital[self.motor3].mode = pyfirmata.SERVO
        self.board.digital[self.motor4].mode = pyfirmata.SERVO
        self.board.digital[self.motor5].mode = pyfirmata.SERVO
        self.board.digital[self.motor6].mode = pyfirmata.SERVO

    def setServoAngle(self, angle, isRad=1):
        """The setServoAngle() function receives a set of arrays with each 6 elements. The sets of angles are for
        sending small movements to the motors and the 6 elements in each array are the angles for each one of the
        motors. There is also an option for sending the list of angle arrays in radian or not. By default,
        we assume that the angles are in radians. """
        if isRad:
            rad_to_deg = 180.0/3.1416
            alpha = 90
        else:
            rad_to_deg = 1
            alpha = 0

        for i in range(len(angle)):
            self.board.digital[self.motor1].write(int(alpha + angle[i][0] * rad_to_deg))
            self.board.digital[self.motor2].write(int(180.0 - (alpha + angle[i][1] * rad_to_deg)))
            self.board.digital[self.motor3].write(int(alpha + angle[i][2] * rad_to_deg))
            self.board.digital[self.motor4].write(int(180.0 - (alpha + angle[i][3] * rad_to_deg)))
            self.board.digital[self.motor5].write(int(alpha + angle[i][4] * rad_to_deg))
            self.board.digital[self.motor6].write(int(180.0 - (alpha + angle[i][5] * rad_to_deg)))
            sleep(0.01)

    def goToHomePosition(self):
        """The goToHomePosition() function calls the setServoAngle() function with the arguments of the homing angles
        initialized in the __init__(). """
        self.setServoAngle(self.homingAngle, 0)
        print("HOMING NOW")

if __name__ == "__main__":
    arduino_coms = ard_communication()


