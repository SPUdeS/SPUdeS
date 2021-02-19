import pyfirmata

board = pyfirmata.Arduino('COM3')   # USB PORT FOR ARDUINO MEGA

pin7 = board.get_pin('d:7:s')  # SET UP SERVO TO PIN 7

def move_servo(angle):
    pin7.write(angle)   # MOVE SERVO TO GIVEN ANGLE