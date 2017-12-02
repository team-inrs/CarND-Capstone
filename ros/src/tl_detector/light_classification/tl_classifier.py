import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
from model import Model, image_width, image_height
from keras.preprocessing.image import img_to_array
import numpy as np
import scipy

class TLClassifier(object):
    def __init__(self):

        # Counters for writing images
        self.red_image_number = 0
        self.yellow_image_number = 0
        self.green_image_number = 0
        self.unknown_image_number = 0

        self.model = Model()
        self.model.load_weights("light_classification/model.h5")
        self.graph = tf.get_default_graph()


    def get_classification(self, image, traffic_light_state_truth):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #PATH = "/home/mikep/Documents/Udacity/Autonomous/CarND-Capstone/"

        # Time to run the network?
        INFERENCE = False
        if INFERENCE:
            image = scipy.misc.imresize(image, (image_height, image_width))
            image = img_to_array(image)
            image /= 255.0
            image = np.expand_dims(image, axis=0)
            with self.graph.as_default():
                preds = self.model.predict(image)[0]
            #print(preds)
            prediction = np.argmax(preds)
            #rospy.loginfo("Model says: " + str(prediction))
            if prediction == 0: return TrafficLight.RED
            if prediction == 1: return TrafficLight.YELLOW
            if prediction == 2: return TrafficLight.GREEN
            return TrafficLight.UNKNOWN

        # Save training data
        else:
            if traffic_light_state_truth == TrafficLight.RED:
                self.red_image_number += 1
                cv2.imwrite(PATH+"new/red/image_" + str(self.red_image_number) + ".png", image)
                print('red')

            if traffic_light_state_truth == TrafficLight.YELLOW:
                self.yellow_image_number += 1
                cv2.imwrite(PATH+"new/yellow/image_" + str(self.yellow_image_number) + ".png", image)
                print('yellow')

            if traffic_light_state_truth == TrafficLight.GREEN:
                self.green_image_number += 1
                cv2.imwrite(PATH+"new/green/image_" + str(self.green_image_number) + ".png", image)
                print('green')

            if traffic_light_state_truth == TrafficLight.UNKNOWN:
                self.unknown_image_number += 1
                cv2.imwrite(PATH+"new/none/image_" + str(self.unknown_image_number) + ".png", image)
                print('unknown')

        # Save
        return TrafficLight.UNKNOWN
