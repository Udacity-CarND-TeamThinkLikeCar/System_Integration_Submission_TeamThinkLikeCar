from styx_msgs.msg import TrafficLight
import sys
import numpy as np
from io import BytesIO
import cv2
import tensorflow as tf
import os 
from PIL import Image
from keras.preprocessing import image
from keras.models import load_model
from keras.applications.mobilenet import MobileNet, preprocess_input, relu6, DepthwiseConv2D
import rospy

class TLClassifier(object):
    def __init__(self):
        self.target_size = (224, 224) # 224 for InceptionV3
	    dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model = load_model(dir_path + '/mobilenet-ft.model',custom_objects={
                   'relu6': relu6,
                   'DepthwiseConv2D': DepthwiseConv2D})
	    self.graph = tf.get_default_graph()
        

    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	    img = self.cv2ToPIL(img)
        predictions = self.predict(img)
        return self.model_indexs_to_styx_msgs_index(predictions)
    
    def predict(self, img):
        """Run model prediction on image
        Args:
            model: keras model
            img: cv2 format image
            target_size: (w,h) tuple
        Returns:
            list of predicted labels and their probabilities
        """
        if img.size != self.target_size:
            img = img.resize(self.target_size)

        img_array = image.img_to_array(img)
        img_array = np.expand_dims(img_array, axis=0)
        img_array = preprocess_input(img_array)
        predictions = None
        with self.graph.as_default():
            predictions = self.model.predict(img_array)
        
        return predictions[0]

    def model_indexs_to_styx_msgs_index(self, predictions):
        """  The labels in the model are in ['red', 'green', 'none', 'yellow']
            the .msg requires green = 2, yellow = 1, red = 0, unknown = 4
        Args:
            predictions: list of label probabilities from a trained model

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        labels = ['green', 'none', 'red', 'yellow']
        max_index = np.argmax(predictions)
        best_label = labels[max_index]
        rospy.logdebug(best_label)
        if(best_label) == 'green':
            return TrafficLight.GREEN
        if(best_label) == 'red':
            return TrafficLight.RED
        if(best_label) == 'yellow':
            return TrafficLight.YELLOW
        return TrafficLight.UNKNOWN
    
    def cv2ToPIL(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return Image.fromarray(img)








