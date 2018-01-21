import os
import tensorflow as tf
import numpy as np
import cv2
from styx_msgs.msg import TrafficLight
import rospy

class TLClassifier(object):
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_file = dir_path + "/mobilenet-ft.pb"
        input_name = "import/input_1"
        output_name = "import/output_node0"
        graph = self.__load_graph(model_file)
        self.input_operation = graph.get_operation_by_name(input_name)
        self.output_operation = graph.get_operation_by_name(output_name)
        self.sess = tf.Session(graph=graph)
        self.input_height = 192
        self.input_width = 192
        self.input_mean = 128
        self.input_std = 128 
  
    def get_classification(self, image):
        """ Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        predictions = self.__predict(image)
        return self.__model_indexes_to_styx_msgs_index(predictions)

    def __load_graph(self, model_file):
        graph = tf.get_default_graph()
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
        image_tf = tf.placeholder(tf.float32, (None, None, 3))
        dims_expander = tf.expand_dims(image_tf, 0)
        resized = tf.image.resize_bilinear(dims_expander, [self.input_height, self.input_width])
        normalized = tf.divide(tf.subtract(resized, [self.input_mean]), [self.input_std])
        sess = tf.Session()
        tensor = sess.run(normalized, feed_dict={image_tf: image})
        return tensor
    
    def __predict(self, image):
        """ Run model prediction on image

        Args:
            model: keras model
            img: PIL format image
            
        Returns:
            List of predicted probabilities
        """       
        tensor = self.__read_tensor_from_image(image)       
        predictions = self.sess.run(self.output_operation.outputs[0],
                            {self.input_operation.outputs[0]: tensor})
        return np.squeeze(predictions)

    def __model_indexes_to_styx_msgs_index(self, predictions):
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

    def __del__(self):
        self.sess.close()








