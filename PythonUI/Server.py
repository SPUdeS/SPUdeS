from flask import Flask, render_template, Response, request
import cv2
# from arduino_comms.ard_communication import setServoAngle, goUPandDown, goToHomePosition, goUpPosition
from Platform.stewartPlatform import stewartPlatform


class Server():
    """ Web server class. """

    def __init__(self):
        # self.sp = Platform()
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

        @app.route('/sendAngle', methods=["POST", "GET"])
        def sendAngleToArduino():
            if request.method == "POST":
                angle_str = request.form["angle"]
                angle_int = int(angle_str)
                goUPandDown(angle_int)
                return "Angle received: " + angle_str
            return render_template('index.html')

        @app.route('/HomingAnglePage', methods=["POST"])
        def responseHomingAngle():
            if request.method == "POST":
                homing = request.form["Homing"]
                if homing == "true":
                    goToHomePosition()
                    return homing
                return 0
            return render_template('index.html')

        @app.route('/MovingUpPage', methods=["POST"])
        def responseMovingUp():
            if request.method == "POST":
                moveUp = request.form["MovingUp"]
                if moveUp == "true":
                    goUpPosition()
                    return moveUp
                return 0
            return render_template('index.html')

        # Assign app to Server variable Server.app
        self.app = app

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
