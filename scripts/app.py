import cv2
import zmq
import numpy as np
from flask import Flask, render_template, Response


def get_frame():
    while True:
        image_bytes = receiver.recv()
        # image_buffer = np.frombuffer(image_bytes, dtype=np.uint8)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image_bytes + b'\r\n')

app = Flask(__name__)


@app.route("/")
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(get_frame(), mimetype='multipart/x-mixed-replace;boundary=frame')



if __name__ == "__main__":

    context = zmq.Context()
    receiver = context.socket(zmq.SUB)
    receiver.connect("tcp://localhost:9000")
    receiver.setsockopt_string(zmq.SUBSCRIBE, "")

    app.run(host="0.0.0.0", port=1121, threaded=True)


