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
import csv
import math

def findNearest(array, value):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return array[idx-1], idx
    else:
        return array[idx], idx

# initialize packages
#lsl = LSL()
faceTracking = Face()
ballTracking = Ball()
gazeTracking = GazeBehaviour()

folder = 'pupil_data_test/exports/'
filename = '000'
cap = cv2.VideoCapture(folder+filename+'/world_viz.mp4')
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5) # 600
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5) # 337
size = (width, height)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(filename+ '.avi', fourcc, 30.0, size)


timestamps_gaze = list()
norm_pos_x = list()
norm_pos_y = list()

with open(folder+filename+'/gaze_positions.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        timestamps_gaze.append(float(row['timestamp']))
        norm_pos_x.append(row['norm_pos_x'])
        norm_pos_y.append(row['norm_pos_y'])
        # print(row['timestamp'], row['norm_pos_x'], row['norm_pos_y'])

# print(timestamps_gaze[2])
# print(norm_pos_y[2])      # dont forget it starts with 0
# print(norm_pos_x[2])

timestamps = np.load(folder+filename+'/world_viz_timestamps.npy')

i = 0

with faceTracking.detection_graph.as_default():
    with tf.Session(graph=faceTracking.detection_graph) as sess:
        while i < length:
            ret, frame = cap.read()

            if frame is not None:
                frame = imutils.resize(frame) #, width=600)
                height, width, channels = frame.shape
                # object (color) detection          [G, R, B, Y, C]
                ball = ballTracking.tracking(frame, [0, 1, 0, 0, 1])

                # iCub face
                face = faceTracking.detect(frame, sess)

                # pupil
                # calculate the nearest timestamp for the current frame
                time = timestamps[i]
                time_close, ind = findNearest(timestamps_gaze, float(time))
                # use the x, y position of the closest timestamp norm_pos_*
                sample = np.zeros([1,3])
                sample[0][0] = time_close
                sample[0][1] = norm_pos_x[ind]
                sample[0][2] = norm_pos_y[ind]
                if sample.any():
                    # push to yarp port
                    gazeTracking.push(frame, sample, ball, face, width, height, [])

                # clear buffer of object for new frame
                ballTracking.ball_all = []

                cv2.imshow('frame', frame)
                # write the flipped frame
                out.write(frame)

            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
            # cv2.waitKey(0)
            i = i + 1

print(gazeTracking.gaze_sequence)
cap.release()
out.release()
cv2.destroyAllWindows()
