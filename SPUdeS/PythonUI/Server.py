"""This Server.py file is a class called Server() that sets up the server and runs it. This is the link between the client side inputs and the cinematic
functions to send commands and new graphs."""
from flask import Flask, render_template, Response, request
import cv2
import json
import os
import shutil
from SPUdeS.Platform.stewartPlatform import stewartPlatform
from SPUdeS.Platform import config as spConfig
from SPUdeS.Arduino.ard_communication import ard_communication

class Server:
    """ Web server class. """

    def __init__(self):
        """ The __init__ function instantiates a stewartPlatform(), an ard_communication() and
        a Flask object. This function also sets up the home plot for the 3D graphs and sets up a camera. It then initializes
        the routes for the Flask server and runs the server."""

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

    def run(self):
        """The run() function runs an instance of the Flask server called app."""
        self.app.run(debug=True, use_reloader=False)

    def initiateFlaskApp(self):
        """The initiateFlaskApp() function calls an instance of the Flask object."""
        app = Flask(__name__)

        @app.route('/video_feed')
        def video_feed():
            """Video Feed Route: Generate video feed page"""
            return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/')
        def index():
            """Index Page Route: General instance of web page HTML"""
            return render_template('index.html')

        @app.route('/NewDisplacementRequest', methods=["POST", "GET"])
        def displacement_request():
            """New Displacement Request Route: Handles the json payload from JavaScript function to send to requestSP() function. The payload
            will be used to sent a command to the Arduino for the motors and to regenerate a new 3D graph."""
            if request.method == "POST":
                data = json.loads(request.data)
                self.requestSP(data["type_"], data["data_"])
                return render_template('index.html')

        @app.route('/NewShowoffRequest', methods=["POST", "GET"])
        def showoffRequest():
            """New Showoff Request Route: This route sends a sequence of pre-programmed motions to the Arduino and regenerates the 3D plots of the platform"""
            self.requestShowoffSP()
            return render_template('index.html')

        @app.route('/UpdateCamera', methods=["POST", "GET"])
        def updateCameraRequest():
            """Update Camera Route: This route handles the change of camera button request."""
            data = json.loads(request.data)
            self.updateCamera(int(data["cameraNumber"]))

        # Assign app to Server variable Server.app
        self.app = app

    @staticmethod
    def initPlot():
        """initPlot() refactors the 3D plots created to give us a clean slate to work with. It re-initializes the plots
        to the default ones. The plots are in SPUdeS/PythonUI/static/img."""
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
        """updateCamera() has by default value 0 for the number of the camera to use on the server. If a camera change is requested cameraNumber will
        change to the number requested. cv2 is the imported library for OpenCV which allows us to have a live feed."""
        self.camera = cv2.VideoCapture(cameraNumber)

    def requestSP(self, type_, data):
        """requestSP() is called through the /NewDisplacementRequest route. It uses the json payload sent from the JavaScript function to send a command in angles to the Arduino
        and to update the 3D plots."""
        listOfServoAngles = self.sp.requestFromFlask(type_, data)
        if self.arduinoCommunication is not None:
            self.arduinoCommunication.setServoAngle(listOfServoAngles)
        self.sp.updateAllPlots()

    def requestShowoffSP(self):
        """requestShowoffSP() is called through the /NewShowoffRequest route. It calls a function to send predetermined movement commands to the Arduino
                and to update the 3D plots."""
        listOfServoAngles = self.sp.requestShowoffFromFlask()
        if self.arduinoCommunication is not None:
            self.arduinoCommunication.setServoAngle(listOfServoAngles)
        self.sp.updateAllPlots()

    def generate_frames(self):
        """ generate_frame() function generates the frame by frame of the camera to create the live feed on the web page. """

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
    serv = Server()
