# python3 onlineDetection.py --buffer 68
from pylsl import StreamInlet, resolve_stream
from pylsl import StreamInfo, StreamOutlet

# import files
from balltracking import Ball
from faceDetector import faceDetector
from gazeBehaviour import gazeBehaviour

# import necessary libraries
from msgpack import unpackb, packb
from collections import deque
import numpy as np
import cv2
import csv
import zmq
import argparse
import imutils
import logging as log

"""
Receive world camera data from Pupil using ZMQ.
Make sure the frame publisher plugin is loaded and configuredsource  to gray or rgb
"""

context = zmq.Context()
# open a req port to talk to pupil
addr = '10.0.3.20'  # remote ip or localhost
req_port = "50020"  # same as in the pupil remote gui
req = context.socket(zmq.REQ)
req.connect("tcp://{}:{}".format(addr, req_port))
# ask for the sub port
req.send_string('SUB_PORT')
sub_port = req.recv_string()

# create a new stream info
info = StreamInfo("GazePose", "NormPose2IP", 4, 100, "float32", "myuid34234")
info.desc().append_child_value("manufacturer", "Vislab")
outlet = StreamOutlet(info)

# send notification:
def notify(notification):
    """Sends ``notification`` to Pupil Remote"""
    topic = 'notify.' + notification['subject']
    payload = packb(notification, use_bin_type=True)
    req.send_string(topic, flags=zmq.SNDMORE)
    req.send(payload)
    return req.recv_string()

# Start frame publisher with format BGR
notify({'subject': 'start_plugin', 'name': 'Frame_Publisher', 'args': {'format': 'bgr'}})

# open a sub port to listen to pupil
sub = context.socket(zmq.SUB)
sub.connect("tcp://{}:{}".format(addr, sub_port))

# set subscriptions to topics
# recv just pupil/gaze/notifications
sub.setsockopt_string(zmq.SUBSCRIBE, 'frame.')

def recv_from_sub():
    '''Recv a message with topic, payload.

    Topic is a utf-8 encoded string. Returned as unicode object.
    Payload is a msgpack serialized dict. Returned as a python dict.

    Any addional message frames will be added as a list
    in the payload dict with key: '__raw_data__' .
    '''
    topic = sub.recv_string()
    payload = unpackb(sub.recv(), encoding='utf-8')
    extra_frames = []
    while sub.get(zmq.RCVMORE):
        extra_frames.append(sub.recv())
    if extra_frames:
        payload['__raw_data__'] = extra_frames
    return topic, payload

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])

ballTracking = Ball()

cascPath = "cascade-icub-60v60.xml"
faceCascade = cv2.CascadeClassifier(cascPath)
log.basicConfig(filename='faceDetected.log', level=log.INFO)
anterior = 0
face = faceDetector()

print("Preparing Data...")
knownFaces, knownLabels = face.prepare_training_data("training-data", faceCascade)
print(knownFaces)
print(knownLabels)
print("Data prepared")

# create our LBPH face recognizer
face_recognizer = cv2.face.LBPHFaceRecognizer_create()
face_recognizer.train(knownFaces, np.array(knownLabels))

gaze = gazeBehaviour(outlet)

print("looking for an NormPose2IP stream...")
streams = resolve_stream('name', 'NormPose2IP')
# create a new inlet to read from the stream
inlet = StreamInlet(streams[0])

i = 0
ball = []
while cv2.waitKey(1):
    topic, msg = recv_from_sub()

    if topic == 'frame.world' and i % 4 == 0:
        frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame is not None:
            frame = imutils.resize(frame, width=750)
            height, width, channels = frame.shape

            frame, pts, ballG = ballTracking.trackingGreen(frame, pts)
            if ballG is not []:
                ball.append([ballG, 1])
            frame, pts, ballR = ballTracking.trackingRed(frame, pts)
            if ballR is not []:
                ball.append([ballR, 2])
            frame, pts, ballB = ballTracking.trackingBlue(frame, pts)
            if ballB is not [] and len(ballB) != 0:
                ball.append([ballB, 3])

            anterior, faces, facesTrained = face.detecting(frame, anterior, faceCascade)
            labels = face.predict(frame, face_recognizer, faces, facesTrained)

            sample, timestamp = inlet.pull_chunk()
            if sample:
                pos_x = sample[0][1]
                pos_y = sample[0][2]

                # print(int(float(pos_x)*width))
                # print(int(height - int(float(pos_y)*height)))
                cv2.circle(frame, (int(float(pos_x) * width), int(height - int(float(pos_y) * height))), 10, (0, 255, 1),
                           thickness=5, lineType=8, shift=0)  # draw circle
                fixation = [(int(float(pos_x) * width)), int(height - int(float(pos_y) * height))]

                # check the gaze behaviour
                if len(ball) is not 0:
                    mysample = gaze.record(sample[0][0], ball, faces, fixation, [])
                    if len(mysample) is not 0:
                        #print(mysample)
                        outlet.push_sample(mysample)

        cv2.imshow('frame', frame)
    i = i + 1

# cap.release()
cv2.destroyAllWindows()
