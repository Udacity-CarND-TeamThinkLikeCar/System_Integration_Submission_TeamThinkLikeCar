from styx_msgs.msg import TrafficLight
import cv2
#from cv2.cv import CV_HOUGH_GRADIENT
import numpy as np
import rospy
import os
import time
from test_rl import TestRL

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        fname = os.path.join('light_classification', 'trained_stage_1.h5')
        self.testRL = TestRL(fname)
        print("initialized")


    def cv_classification(self,image):

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        frame_threshed = cv2.inRange(hsv_img, np.array([0, 120, 120], np.uint8), np.array([10, 255, 255], np.uint8))
        r = cv2.countNonZero(frame_threshed)
        if r > 50:
            return "Red"

        frame_threshed = cv2.inRange(hsv_img, np.array([40.0 / 360 * 255, 120, 120], np.uint8), np.array([66.0 / 360 * 255, 255, 255], np.uint8))
        y = cv2.countNonZero(frame_threshed)
        if y > 50:
            return "Yellow"

        frame_threshed = cv2.inRange(hsv_img, np.array([90.0 / 360 * 255, 120, 120], np.uint8), np.array([140.0 / 360 * 255, 255, 255], np.uint8))
        g = cv2.countNonZero(frame_threshed)
        if g > 50:
            return "Green"

        return None

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # state = None
        #
        # cv2.imwrite("/capstone/img" + str(time.time()) + ".jpeg", image)
        #
        # chsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #
        # mask = cv2.inRange(chsv, np.array([11, 100, 100]), np.array([59, 255, 255]))
        #
        # # sides, vertices = find_vertices(lines[0], valid_angles=[0., 90.])
        # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 2, 50, param1=60, param2=30,
        #                            minRadius=2, maxRadius=30)
        #
        # if circles is not None:
        #
        #     state = "yellow"
        #
        # lower_red_hue_range = cv2.inRange(chsv, np.array([0, 255, 255]), np.array([10, 255, 255]))
        #
        # circles = cv2.HoughCircles(lower_red_hue_range, cv2.HOUGH_GRADIENT, 2, 50, param1=60, param2=30,
        #                            minRadius=2, maxRadius=30)
        #
        # if circles is not None:
        #     state = "red"
        #
        # mask = cv2.inRange(chsv, np.array([60, 255, 255], dtype="uint8"), np.array([60, 255, 255], dtype="uint8"))
        #
        # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 2, 50, param1=60, param2=30,
        #                            minRadius=2, maxRadius=30)
        #
        # if circles is not None:
        #     state = "green"

        state = self.testRL.classify(image)

        trafficlight = TrafficLight.UNKNOWN

        cv_state = self.cv_classification(image)
        print("cv_state ",cv_state)

        if "Red" == state:
            trafficlight = TrafficLight.RED
        elif "Green" == state:
            trafficlight = TrafficLight.GREEN
        elif "Yellow" == state:
            trafficlight = TrafficLight.YELLOW


        rospy.logdebug("state is" + str(state))


        return trafficlight
