# python3 main.py --buffer 68
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from ball_tracking import Ball
from face_detector import FaceDetector
from gaze_behaviour import GazeBehaviour
from pupil_lsl_yarp import LSL

import tensorflow as tf
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
faceTracking = FaceDetector()
ballTracking = Ball()
gazeTracking = GazeBehaviour()

filename = '/home/nduarte/software/pupil/recordings/2021_06_23/002/exports/004'
cap = cv2.VideoCapture(filename+'/world_viz.mp4')
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

timestamps_gaze = list()
norm_pos_x = list()
norm_pos_y = list()

with open(filename+'/gaze_positions.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        timestamps_gaze.append(float(row['timestamp']))
        norm_pos_x.append(row['norm_pos_x'])
        norm_pos_y.append(row['norm_pos_y'])
        # print(row['timestamp'], row['norm_pos_x'], row['norm_pos_y'])

# print(timestamps_gaze[2])
# print(norm_pos_y[2])      # dont forget it starts with 0
# print(norm_pos_x[2])

timestamps = np.load(filename+'/world_viz_timestamps.npy')

i = 0
while i < length:
    ret, frame = cap.read()

    if frame is not None:
        frame = imutils.resize(frame, width=1020)
        height, width, channels = frame.shape

        # object (color) detection          [G, R, B, Y, C]
        ball = ballTracking.tracking(frame, [0, 1, 0, 0, 1])

        # iCub face
        face = faceTracking.detecting(frame)

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
            gazeTracking.push(frame, sample, ball, [], width, height)

        # clear buffer of object for new frame
        ballTracking.ball_all = []

        cv2.imshow('frame', frame)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    cv2.waitKey(0)
    i = i + 1

cap.release()
cv2.destroyAllWindows()
