# python3 onlineDetection.py --buffer 68
from pylsl import StreamInlet, resolve_stream
from pylsl import StreamInfo, StreamOutlet

# import files
from balltracking import Ball
from faceDetector import faceDetector
from gazeBehaviour import gazeBehaviour

# import necessary libraries
from msgpack import unpackb
from collections import deque
import numpy as np
import cv2
import zmq
import argparse
import imutils

# import for face detection
import tensorflow as tf
import os
from utils import label_map_util
from utils import visualization_utils as vis_util

"""
Receive world camera data from Pupil using ZMQ.
Make sure the frame publisher plugin is loaded and configuredsource  to gray or rgb
"""

context = zmq.Context()
# open a req port to talk to pupil
addr = '127.0.0.1'  # remote ip or localhost
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
gaze = gazeBehaviour(outlet)

print("looking for an NormPose2IP stream...")
streams = resolve_stream('name', 'NormPose2IP')
# create a new inlet to read from the stream
inlet = StreamInlet(streams[0])

# What model to download.
MODEL_NAME = 'icub_graph'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'
# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('training', 'icub_detection.pbtxt')
NUM_CLASSES = 1

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

i = 0
ball = []
with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        while cv2.waitKey(1):
            topic, msg = recv_from_sub()

            if topic == 'frame.world' and i % 4 == 0:
                frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

                # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if frame is not None:
                    frame = imutils.resize(frame, width=750)
                    height, width, channels = frame.shape

                    frame, pts, ballG = ballTracking.trackingGreen(frame, pts)
                    if ballG is not [] and len(ballG) != 0:
                        ball.append([ballG, 1])
                    frame, pts, ballR = ballTracking.trackingRed(frame, pts)
                    if ballR is not [] and len(ballR) != 0:
                        ball.append([ballR, 2])
                    # frame, pts, ballB = ballTracking.trackingBlue(frame, pts)
                    # if ballB is not [] and len(ballB) != 0:
                    #      ball.append([ballB, 3])
                    frame, pts, ballY = ballTracking.trackingYellow(frame, pts)
                    if ballY is not [] and len(ballY) != 0:
                        ball.append([ballY, 4])
                    # frame, pts, ballC = ballTracking.trackingCyan(frame, pts)
                    # if ballC is not [] and len(ballC) != 0:
                    #     ball.append([ballC, 5])


                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(frame, axis=0)
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object was detected.
                    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    scores = detection_graph.get_tensor_by_name('detection_scores:0')
                    classes = detection_graph.get_tensor_by_name('detection_classes:0')
                    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                      [boxes, scores, classes, num_detections],
                      feed_dict={image_tensor: image_np_expanded})
                    # Visualization of the results of a detection.
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        frame,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=8)

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
                            mysample = gaze.record(sample[0][0], ball, [], fixation, [])
                            if len(mysample) is not 0:
                                #print(mysample)
                                outlet.push_sample(mysample)

                cv2.imshow('frame', frame)
            i = i + 1

# cap.release()
cv2.destroyAllWindows()
