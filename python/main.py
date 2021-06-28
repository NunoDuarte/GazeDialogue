# python3 main.py --buffer 68
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from ball_tracking import Ball
from face_detector import FaceDetector
from face_detector_gpu import FaceGPU
from gaze_behaviour import GazeBehaviour
from pupil_lsl_yarp import LSL

import tensorflow as tf
import numpy as np
import cv2
import imutils

# initialize packages
lsl = LSL()
faceTracking = FaceGPU()
ballTracking = Ball()
gazeTracking = GazeBehaviour(lsl.outlet)

i = 0
with faceTracking.detection_graph.as_default():
    with tf.Session(graph=faceTracking.detection_graph) as sess:
        while cv2.waitKey(1):
            topic, msg = lsl.recv_from_sub()

            if topic == 'frame.world' and i % 2 == 0:
                frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

                if frame is not None:
                    frame = imutils.resize(frame, width=430)
                    height, width, channels = frame.shape

                    # object (color) detection          [G, R, B, Y, C]
                    ball = ballTracking.tracking(frame, [1, 1, 0, 0, 1])

                    # iCub face
                    if i % 8 == 0:
                        faceTracking.detect(frame, sess)

                    # pupil
                    sample, timestamp = lsl.inlet.pull_chunk()
                    if sample:
                        # push to yarp port
                        gazeTracking.push(frame, sample, [], width, height, lsl)

                    # clear buffer of object for new frame
                    ballTracking.ball_all = []

                cv2.imshow('frame', frame)
            i = i + 1

cv2.destroyAllWindows()