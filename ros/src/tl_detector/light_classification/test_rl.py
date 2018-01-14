import argparse
import colorsys
import imghdr
import os
import random
import cv2 

import numpy as np
import tensorflow as tf
from keras import backend as K
from keras.layers import Input, Lambda, Conv2D
from keras.models import load_model, Model
from PIL import Image, ImageDraw, ImageFont

from yad2k.models.keras_yolo import yolo_eval, yolo_head,yolo_loss,yolo_body

YOLO_ANCHORS = np.array(
    ((0.57273, 0.677385), (1.87446, 2.06253), (3.33843, 5.47434),
     (7.88282, 3.52778), (9.77052, 9.16828)))

class TestRL(object):
    def __init__(self,model_path):
        self.model_path = model_path


        sess = K.get_session()

        class_names = ["Red","Green","Yellow","off"]

        anchors = YOLO_ANCHORS

        yolo_model, model = self.create_model(anchors, class_names)

        yolo_model.load_weights(model_path)

        num_classes = len(class_names)
        num_anchors = len(anchors)

        print(num_classes,num_anchors)

        # TODO: Assumes dim ordering is channel last
        model_output_channels = yolo_model.layers[-1].output_shape[-1]
        # assert model_output_channels == num_anchors * (num_classes + 5), \
        #     'Mismatch between model and given anchor and class sizes. ' \
        #     'Specify matching anchors and classes with --anchors_path and ' \
        #     '--classes_path flags.'
        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Check if model is fully convolutional, assuming channel last order.
        model_image_size = yolo_model.layers[0].input_shape[1:3]
        is_fixed_size = model_image_size != (None, None)

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(class_names), 1., 1.)
                      for x in range(len(class_names))]
        colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                colors))
        random.seed(10101)  # Fixed seed for consistent colors across runs.
        random.shuffle(colors)  # Shuffle colors to decorrelate adjacent classes.
        random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        # TODO: Wrap these backend operations with Keras layers.
        yolo_outputs = yolo_head(yolo_model.output, anchors, len(class_names))
        input_image_shape = K.placeholder(shape=(2, ))
        boxes, scores, classes = yolo_eval(
            yolo_outputs,
            input_image_shape,
            score_threshold=0.3,
            iou_threshold=0.5)

        self.boxes, self.scores, self.classes = boxes, scores, classes
        self.K = K
        self.learning_phase = self.K.learning_phase()
        self.sess = sess
        self.yolo_model = yolo_model
        self.input_image_shape = input_image_shape
        self.class_names = class_names

    def classify(self,image):

        resized_image = cv2.resize(np.array(image), (416, 416))

        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        # self.K.learning_phase(): 0
        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.shape[1], image.shape[0]],
                self.learning_phase:0
            })

        indd = np.argmax(out_scores)
        indd_class = out_classes[indd]
        print(self.class_names[out_classes[indd]],indd)

        return self.class_names[out_classes[indd]]


    def create_model(self,anchors, class_names, load_pretrained=True, freeze_body=True):

        detectors_mask_shape = (13, 13, 5, 1)
        matching_boxes_shape = (13, 13, 5, 5)
    
        # Create model input layers.
        image_input = Input(shape=(416, 416, 3))
        boxes_input = Input(shape=(None, 5))
        detectors_mask_input = Input(shape=detectors_mask_shape)
        matching_boxes_input = Input(shape=matching_boxes_shape)
    
        # Create model body.
        yolo_model = yolo_body(image_input, len(anchors), len(class_names))
        topless_yolo = Model(yolo_model.input, yolo_model.layers[-2].output)
    
        if load_pretrained:
            # Save topless yolo:
            topless_yolo_path = os.path.join('light_classification','model_data', 'yolo_topless.h5')
            if not os.path.exists(topless_yolo_path):
                print("CREATING TOPLESS WEIGHTS FILE")
                yolo_path = os.path.join('light_classification','model_data', 'yolo.h5')
                model_body = load_model(yolo_path)
                model_body = Model(model_body.inputs, model_body.layers[-2].output)
                model_body.save_weights(topless_yolo_path)
            topless_yolo.load_weights(topless_yolo_path)
    
        if freeze_body:
            for layer in topless_yolo.layers:
                layer.trainable = False
        final_layer = Conv2D(len(anchors)*(5+len(class_names)), (1, 1), activation='linear')(topless_yolo.output)
    
        model_body = Model(image_input, final_layer)
    
        # Place model loss on CPU to reduce GPU memory usage.
        with tf.device('/cpu:0'):
            # TODO: Replace Lambda with custom Keras layer for loss.
            model_loss = Lambda(
                yolo_loss,
                output_shape=(1, ),
                name='yolo_loss',
                arguments={'anchors': anchors,
                           'num_classes': len(class_names)})([
                               model_body.output, boxes_input,
                               detectors_mask_input, matching_boxes_input
                           ])
    
        model = Model(
            [model_body.input, boxes_input, detectors_mask_input,
             matching_boxes_input], model_loss)
    
        return model_body, model



if __name__ == '__main__':
    testRL = TestRL("./trained_stage_1.h5")

    for image_file in os.listdir("images"):
        try:
            image_type = imghdr.what(os.path.join("images", image_file))
            if not image_type:
                continue
        except IsADirectoryError:
            continue

        image = Image.open(os.path.join("images", image_file))

        testRL.classify(image)

    print("done")
