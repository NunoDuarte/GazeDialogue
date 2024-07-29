# activate anaconda environment:export PATH="/home/nduarte/anaconda3/bin:$PATH" && source activate pupilos
# python3 main.py --buffer 68

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from ball_tracking import Ball
# from face_detector import FaceDetector as Face
from face_detector_gpu import FaceGPU as Face
from gaze_behaviour import GazeBehaviour
from pupil_lsl_yarp import LSL

#import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

import numpy as np
import cv2
import imutils

# initialize packages
lsl = LSL()
faceTracking = Face()
ballTracking = Ball()
gazeTracking = GazeBehaviour(lsl.outlet)

# A window will open. To close, press 'q'.
i = 0
with faceTracking.detection_graph.as_default():
    with tf.Session(graph=faceTracking.detection_graph) as sess:
        while cv2.waitKey(1) & 0xFF != ord('q'):
            topic, msg = lsl.recv_from_sub()

            if topic == 'frame.world' and i % 2 == 0:
                #print('msg', np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8))
                frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(720,1280, 3)

                if frame is not None:
                    frame = imutils.resize(frame, width=640)
                    height, width, channels = frame.shape

                    # object (color) detection          [G, R, B, Y, C]
                    ball = ballTracking.tracking(frame, [1, 1, 1, 0, 1])

                    # iCub face
                    face = None
                    if i % 8 == 0:
                        face = faceTracking.detect(frame, sess)

                    # pupil
                    sample, timestamp = lsl.inlet.pull_chunk()
                    if sample:
                        # push to yarp port
                        gazeTracking.push(frame, sample, ball, face, width, height, lsl)

                    # clear buffer of object for new frame
                    ballTracking.ball_all = []

                cv2.imshow('frame', frame)
            i = i + 1

print(gazeTracking.gaze_sequence)
cv2.destroyAllWindows()
