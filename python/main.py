# activate anaconda environment:export PATH="/home/nduarte/anaconda3/bin:$PATH" && source activate pupilos

# python3 main.py (optional) --buffer 68
from ball_tracking import Ball
from face_detector import FaceDetector
from gaze_behaviour import GazeBehaviour
from pupil_lsl_yarp import LSL

import numpy as np
import cv2
import imutils

# initialize packages
lsl = LSL()
faceTracking = FaceDetector()
ballTracking = Ball()
gazeTracking = GazeBehaviour(lsl.outlet)

i = 0   # to skip frames for faster Frame Rate
while cv2.waitKey(1):
    topic, msg = lsl.recv_from_sub()

    if topic == 'frame.world' and i % 4 == 0:
        frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

        if frame is not None:
            frame = imutils.resize(frame, width=750)
            height, width, channels = frame.shape

            # object (color) detection          [G, R, B, Y, C]
            ball = ballTracking.tracking(frame, [1, 0, 0, 1, 1], lsl.pts)

            # iCub face
            # face = faceTracking.detecting(frame)

            # pupil
            sample, timestamp = lsl.inlet.pull_chunk()
            if sample:
                # push to yarp port
                gazeTracking.push(frame, sample, ball, [], width, height)

            # clear buffer of object for new frame
            ballTracking.ball_all = []

        cv2.imshow('frame', frame)
    i = i + 1

cv2.destroyAllWindows()
