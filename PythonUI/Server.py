from flask import Flask, render_template, Response, request
import cv2
# from arduino_comms.ard_communication import setServoAngle, goUPandDown, goToHomePosition, goUpPosition
from Platform.stewartPlatform import stewartPlatform
import json

class Server():
    """ Web server class. """

    def __init__(self):
        self.sp = stewartPlatform()
        # Choose the right camera with the argument for VideoCapture
        self.camera = cv2.VideoCapture(0)
        self.app = None
        self.initiateFlaskApp()
        self.run()

    def run(self):
        self.app.run(debug=True, use_reloader=False)

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
                self.requestTarget(data["type_"], data["displacement"])
                return render_template('index.html')


        # Assign app to Server variable Server.app
        self.app = app

    def requestTarget(self, type_, destinations):
        pass ## TODO: send data to sp

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
    def requestToStewartPlatform(self, jsonData):
        plot = sp.requestFromFlask(jsonData)
        plot.savefig()
        #TODO: confirm update of photo before posting


if __name__ == '__main__':
    print('start')
    serv = Server()
    print('end')
