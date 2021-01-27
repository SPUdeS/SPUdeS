from flask import Flask, render_template, Response
import cv2

app = Flask(__name__)

# Choose the right camera with the argument for VideoCapture
camera = cv2.VideoCapture(0)

# Generate frame by frame
def generate_frames():
    while True:
        # read() returns the retval(successful or not) and the image
        ret, frame = camera.read()
        if not ret:
            break
        else:
            # Encodes image into memory buffer
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    # Route for streaming video
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Video Streaming Index Page"""
    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True)
