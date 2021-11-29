from utils import label_map_util
from utils import visualization_utils as vis_util
import tensorflow as tf
import os
import numpy as np

def length_of_bounding_box(bbox, IMG_WIDTH):
    return bbox[3]*IMG_WIDTH - bbox[1]*IMG_WIDTH

class FaceGPU:

    def __init__(self):
        # What model to download.
        FOLDER_NAME = 'icub_face'
        MODEL_NAME = 'icub_graph'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_FROZEN_GRAPH = FOLDER_NAME + '/' + MODEL_NAME + '/frozen_inference_graph.pb'
        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join(FOLDER_NAME, 'icub_detection.pbtxt')
        NUM_CLASSES = 1

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.compat.v1.GraphDef() #tf.GraphDef()
            with tf.compat.v2.io.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:  #tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def detect(self, frame, sess):
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(frame, axis=0)
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # Actual detection.
        (boxes, scores, classes, num_detections) = sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        # find best score
        id = np.argmax(np.squeeze(scores))
        maxId = np.squeeze(boxes)[id]
        # Visualization of the results of a detection.
        _, width, _ = frame.shape
        if 50 < length_of_bounding_box(maxId, width) < 170:
            # print(length_of_bounding_box(maxId, width))
            vis_util.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw = 1,
                skip_scores=True,       # to not show the score confidence
                line_thickness=8,
                min_score_thresh=0)
            #
            return maxId
