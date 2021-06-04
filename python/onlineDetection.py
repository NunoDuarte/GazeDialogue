# python3 onlineDetection.py --buffer 68

from balltracking import Ball
from faceDetector import faceDetector
from gazeBehaviour import gazeBehaviour
from pupil_lsl_yarp import LSL

import numpy as np
import cv2
import imutils

# import for face detection
import tensorflow as tf
import os
from utils import label_map_util
from utils import visualization_utils as vis_util


lsl = LSL()
ballTracking = Ball()
gaze = gazeBehaviour(lsl.outlet)



# # What model to download.
# MODEL_NAME = 'icub_graph'
# # Path to frozen detection graph. This is the actual model that is used for the object detection.
# PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'
# # List of the strings that is used to add correct label for each box.
# PATH_TO_LABELS = os.path.join('training', 'icub_detection.pbtxt')
# NUM_CLASSES = 1
#
# label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
# categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
# category_index = label_map_util.create_category_index(categories)
#
# detection_graph = tf.Graph()
# with detection_graph.as_default():
#     od_graph_def = tf.GraphDef()
#     with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
#         serialized_graph = fid.read()
#         od_graph_def.ParseFromString(serialized_graph)
#         tf.import_graph_def(od_graph_def, name='')

i = 0
ball = []

# with detection_graph.as_default():
#     with tf.Session(graph=detection_graph) as sess:
while cv2.waitKey(1):
    topic, msg = lsl.recv_from_sub()

    if topic == 'frame.world' and i % 4 == 0:
        frame = np.frombuffer(msg['__raw_data__'][0], dtype=np.uint8).reshape(msg['height'], msg['width'], 3)

        if frame is not None:
            frame = imutils.resize(frame, width=750)
            height, width, channels = frame.shape

            # object (color) detection [G, R, B, Y, C]
            ball = ballTracking.tracking(frame, [1, 1, 1, 0, 0], lsl.pts)

            # # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            # image_np_expanded = np.expand_dims(frame, axis=0)
            # image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # # Each box represents a part of the image where a particular object was detected.
            # boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # # Each score represent how level of confidence for each of the objects.
            # # Score is shown on the result image, together with the class label.
            # scores = detection_graph.get_tensor_by_name('detection_scores:0')
            # classes = detection_graph.get_tensor_by_name('detection_classes:0')
            # num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            # # Actual detection.
            # (boxes, scores, classes, num_detections) = sess.run(
            #   [boxes, scores, classes, num_detections],
            #   feed_dict={image_tensor: image_np_expanded})
            # # Visualization of the results of a detection.
            # vis_util.visualize_boxes_and_labels_on_image_array(
            #     frame,
            #     np.squeeze(boxes),
            #     np.squeeze(classes).astype(np.int32),
            #     np.squeeze(scores),
            #     category_index,
            #     use_normalized_coordinates=True,
            #     line_thickness=8)

            sample, timestamp = lsl.inlet.pull_chunk()
            if sample:
                pos_x = sample[0][1]
                pos_y = sample[0][2]

                # print(int(float(pos_x)*width))
                # print(int(height - int(float(pos_y)*height)))
                # cv2.circle(frame, (int(float(pos_x) * width), int(height - int(float(pos_y) * height))), 10, (0, 255, 1),
                #            thickness=5, lineType=8, shift=0)  # draw circle
                fixation = [(int(float(pos_x) * width)), int(height - int(float(pos_y) * height))]

                # check the gaze behaviour
                if len(ball) is not 0:
                    mysample = gaze.record(sample[0][0], ball, [], fixation, [])
                    if len(mysample) is not 0:
                        #print(mysample)
                        lsl.outlet.push_sample(mysample)

        cv2.imshow('frame', frame)
    i = i + 1

cv2.destroyAllWindows()
