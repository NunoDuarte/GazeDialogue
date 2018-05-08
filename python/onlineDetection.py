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
import threading

"""
Receive world camera data from Pupil using ZMQ.
Make sure the frame publisher plugin is loaded and configuredsource  to gray or rgb
"""

class myThread (threading.Thread):

    def __init__(self, threadID, name, args):
        threading.Thread.__init__(self)
        self.args = args
        context = zmq.Context()
        # open a req port to talk to pupil
        addr = '127.0.0.1'  # remote ip or localhost
        req_port = "50020"  # same as in the pupil remote gui
        self.req = context.socket(zmq.REQ)
        self.req.connect("tcp://{}:{}".format(addr, req_port))
        # ask for the sub port
        self.req.send_string('SUB_PORT')
        sub_port = self.req.recv_string()

        # create a new stream info
        info = StreamInfo("GazePose", "NormPose2IP", 4, 100, "float32", "myuid34234")
        info.desc().append_child_value("manufacturer", "Vislab")
        self.outlet = StreamOutlet(info)

        # Start frame publisher with format BGR
        self.notify({'subject': 'start_plugin', 'name': 'Frame_Publisher', 'args': {'format': 'bgr'}})

        # open a sub port to listen to pupil
        self.sub = context.socket(zmq.SUB)
        self.sub.connect("tcp://{}:{}".format(addr, sub_port))

        # set subscriptions to topics
        # recv just pupil/gaze/notifications
        self.sub.setsockopt_string(zmq.SUBSCRIBE, 'frame.')

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video", help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
        args = vars(ap.parse_args())
        pts = deque(maxlen=args["buffer"])

        self.ballTracking = Ball()

        self.gaze = gazeBehaviour(self.outlet)

        print("looking for an NormPose2IP stream...")
        streams = resolve_stream('name', 'NormPose2IP')
        # create a new inlet to read from the stream
        self.inlet = StreamInlet(streams[0])

        # while cv2.waitKey(1):
        # Define a function for the thread
        self.i = 0
        print(self.args)

    def run(self):
        print("Starting " + self.name)
        while 1:
            detection(self)
        print("Exiting " + self.name)

    # send notification:
    def notify(self, notification):
        """Sends ``notification`` to Pupil Remote"""
        topic = 'notify.' + notification['subject']
        payload = packb(notification, use_bin_type=True)
        self.req.send_string(topic, flags=zmq.SNDMORE)
        self.req.send(payload)
        return self.req.recv_string()


    def recv_from_sub(self, ):
        '''Recv a message with topic, payload.

        Topic is a utf-8 encoded string. Returned as unicode object.
        Payload is a msgpack serialized dict. Returned as a python dict.

        Any addional message frames will be added as a list
        in the payload dict with key: '__raw_data__' .
        '''
        topic = self.sub.recv_string()
        payload = unpackb(self.sub.recv(), encoding='utf-8')
        extra_frames = []
        while self.sub.get(zmq.RCVMORE):
            extra_frames.append(self.sub.recv())
        if extra_frames:
            payload['__raw_data__'] = extra_frames
        return topic, payload


def detection(self, ):
# while cv2.waitKey(1):

    topic, msg = self.recv_from_sub()

    if topic == 'frame.world' and self.i % 2 == 0:
        frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame is not None:
            frame = imutils.resize(frame, width=750)
            height, width, channels = frame.shape

            frame, pts, ball = self.ballTracking.tracking(frame, [], self.args)   # pts

            # anterior, faces, facesTrained = face.detecting(frame, anterior, faceCascade)
            # labels = face.predict(frame, face_recognizer, faces, facesTrained)

            sample, timestamp = self.inlet.pull_chunk()
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
                    mysample = self.gaze.record(sample[0][0], [], ball, [], fixation, [])
                    if mysample is not 0:
                        #print(mysample)
                        self.outlet.push_sample(mysample)

        cv2.imshow('frame', frame)
    self.i = self.i + 1


# Create new threads
thread1 = myThread(1, "Thread-1", "blue")
# thread2 = myThread(2, "Thread-2", 2)

# Start new Threads
thread1.start()
thread1.join()


# cap.release()
# cv2.destroyAllWindows()
