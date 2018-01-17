from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import os 
from io import BytesIO
import tensorflow as tf
import numpy as np
import cv2
from PIL import Image
from styx_msgs.msg import TrafficLight
import rospy

class TLClassifier(object):
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_file = dir_path + "/mobilenet-ft.pb"
        input_name = "import/input_1"
        output_name = "import/output_node0"
        self.input_height = 224
        self.input_width = 224
        self.input_mean = 128
        self.input_std = 128
        self.graph = tf.get_default_graph()
        self.graph = self.__load_graph(model_file)
        self.input_operation = self.graph.get_operation_by_name(input_name)
        self.output_operation = self.graph.get_operation_by_name(output_name)
  
    def get_classification(self, img):
        """ Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #img = self.__cv2ToPIL(img)
        predictions = self.__predict(img)
        return self.__model_indexs_to_styx_msgs_index(predictions)

    def __load_graph(self, model_file):
        graph = tf.Graph()
        graph_def = tf.GraphDef()

        with open(model_file, "rb") as f:
            graph_def.ParseFromString(f.read())
        with graph.as_default():
            tf.import_graph_def(graph_def)

        return graph

    def __read_tensor_from_image(self, image):
        """ Preprocess the image for training

        Args: 
            image: PIL format image

        Returns:
            Tensor of an image
        """  
        img_tf = tf.placeholder(tf.float32, (None, None, 3))
        float_caster = tf.cast(img_tf, tf.float32)
        dims_expander = tf.expand_dims(float_caster, 0)
        resized = tf.image.resize_bilinear(dims_expander, [self.input_height, self.input_width])
        normalized = tf.divide(tf.subtract(resized, [self.input_mean]), [self.input_std])
        sess = tf.Session()
        result = sess.run(normalized, feed_dict= {img_tf: image})
        return result
    
    def __predict(self, img):
        """ Run model prediction on image

        Args:
            model: keras model
            img: PIL format image
            
        Returns:
            List of predicted probabilities
        """       
        tensor = self.__read_tensor_from_image(img)
        
        with tf.Session(graph=self.graph) as sess:
            results = sess.run(self.output_operation.outputs[0],
                            {self.input_operation.outputs[0]: tensor})
        return np.squeeze(results)

    def __model_indexs_to_styx_msgs_index(self, predictions):
        """  The labels in the model are in ['green', 'none', 'red', 'yellow']
            the .msg is in format green = 2, yellow = 1, red = 0, unknown = 4

        Args:
            predictions: List of label probabilities from a trained model

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
    
    def __cv2ToPIL(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return Image.fromarray(img)








