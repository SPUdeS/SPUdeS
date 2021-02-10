import serial   # import Arduino Serial library

arduinoSerialPort = serial.Serial("COM3", 9600)

arduinoSerialPort.write('b'.encode())