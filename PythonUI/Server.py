from flask import Flask, render_template, Response, request
import cv2
import json
import os
import shutil
from Platform.stewartPlatform import stewartPlatform
from Platform import config as spConfig
from Arduino.ard_communication import ard_communication

class Server():
    """ Web server class. """

    def __init__(self):
        self.sp = stewartPlatform()
        try:
            self.arduinoCommunication = ard_communication()
        except:
            self.arduinoCommunication = None
            pass
        self.camera = None
        self.app = None
        self.initPlot()
        self.updateCamera()
        self.initiateFlaskApp()
        self.run()
        #TODO: initialize with requestSP

    def run(self):
        self.app.run(debug=True, use_reloader=False)

    def updateHost(self): #todo:host
        self.app.run(debug=True, use_reloader=False, host="192.168.176.1")

    def initiateFlaskApp(self):
        app = Flask(__name__)

        @app.route('/video_feed')
        def video_feed():
            # Route for streaming video
            return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/')
        def index():
            """Video Streaming Index Page"""
            return render_template('index.html')

        @app.route('/NewDisplacementRequest', methods=["POST", "GET"])
        def displacement_request():
            if request.method == "POST":
                data = json.loads(request.data)
                self.requestSP(data["type_"], data["data_"])
                return render_template('index.html')

        @app.route('/UpdateCamera', methods=["POST", "GET"])
        def updateCameraRequest():
            data = json.loads(request.data)
            self.updateCamera(int(data["cameraNumber"]))

        # Assign app to Server variable Server.app
        self.app = app

    @staticmethod
    def initPlot():
        try:
            os.remove(spConfig.plot3DPath)
        except OSError:
            pass
        try:
            os.remove(spConfig.plotUpViewPath)
        except OSError:
            pass
        try:
            os.remove(spConfig.plotFrontViewPath)
        except OSError:
            pass
        try:
            os.remove(spConfig.plotRightViewPath)
        except OSError:
            pass
        shutil.copy(spConfig.plot3DHomePath, spConfig.plot3DPath)
        shutil.copy(spConfig.plotUpViewHomePath, spConfig.plotUpViewPath)
        shutil.copy(spConfig.plotFrontViewHomePath, spConfig.plotFrontViewPath)
        shutil.copy(spConfig.plotRightViewHomePath, spConfig.plotRightViewPath)

    def updateCamera(self, cameraNumber = 0):
        self.camera = cv2.VideoCapture(cameraNumber)

    def requestSP(self, type_, data):
        listOfServoAngles = self.sp.requestFromFlask(type_, data)
        if self.arduinoCommunication is not None:
            self.arduinoCommunication.setServoAngle(listOfServoAngles)

        # TODO: confirm update of photo before posting

    def generate_frames(self):
        """ Generate frame by frame. """

        while True:
            # read() returns the retval(successful or not) and the image
            ret, frame = self.camera.read()
            if not ret:
                break
            else:
                # Encodes image into memory buffer
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


if __name__ == '__main__':
    print('start')
    serv = Server()
    print('end')
