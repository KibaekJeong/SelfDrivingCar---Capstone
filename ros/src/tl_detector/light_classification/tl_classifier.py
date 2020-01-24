from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
from PIL import Image
import cv2
import numpy as np
from keras.models import load_model
import os

class TLClassifier(object):
    def __init__(self):
        cwd = os.path.dirname(os.path.realpath(__file__))

        self.classifier_graph = tf.get_default_graph()
        # Load detection graph
        self.detection_graph = self.load_graph(cwd+'/model/frozen_inference_graph.pb')
        self.config = tf.ConfigProto(log_device_placement=True)
        self.tf_session = tf.Session(graph=self.detection_graph,config=self.config)

        #Input place holder for images
        #get_tensor_by_name returns the Tensor with the associated name
        self.image_tensor= self.detection_graph.get_tensor_by_name('image_tensor:0')
        #Box represents particular object in the image, in our case Traffic light
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        #Detection Score represents level of confidence for detecing  object
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        #Detection class represents classification of the object
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        #Number of detections
        self.num_detection = self.detection_graph.get_tensor_by_name('num_detections:0')
        print("Traffic Light Classifier Ready")

    def load_graph(self,graph_file):
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            # Find first detection of traffic signal,which is labeled with number 10
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # CV2 reads image in BGR space, our model is trained with RGB image
        # Convert from BGR to RGB space

        np_image = np.expand_dims(image,axis=0)
        with self.detection_graph.as_default():
            (boxes,scores,classes,num) = self.tf_session.run([self.detection_boxes,self.detection_scores,
            self.detection_classes,self.num_detection],feed_dict={self.image_tensor:np_image})

        boxes=np.squeeze(boxes)
        scores=np.squeeze(scores)
        classes=np.squeeze(classes).astype(np.int32)

        detection_threshold=0.3
        boxes, scores, classes = self.filter_boxes(detection_threshold, boxes, scores, classes)

        if (len(classes)==0):
            return TrafficLight.UNKNOWN
        else:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.YELLOW
            elif classes[0] == 3:
                return TrafficLight.RED
        return TrafficLight.UNKNOWN
