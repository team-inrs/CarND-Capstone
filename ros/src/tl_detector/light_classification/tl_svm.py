import rospy
from styx_msgs.msg import TrafficLight
import numpy as np
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
import pickle
import cv2
from time import time

RED = 0
YELLOW = 1
GREEN = 2

class TL_SVM():
    def __init__(self):
        path = rospy.get_param("traffic_light_classifier")
        #path = "/home/mikep/Documents/DNN-Tensorflow-Models/Traffic_Light/SVM/svm.p"

        with open(path, mode='rb') as f:
            data = pickle.load(f)

        self.svm = data['svm']
        self.X_scaler = data['X_scaler']

    def classify(self, img, boxes):
        t1 = time()
        classes = self.classify_rois(img, boxes)
        #print(classes)

        counts = np.bincount(classes)
        light = np.argmax(counts)
        #print(light)

        t2 = time()
        #print(t2-t1)

        if light == RED: return TrafficLight.RED
        if light == YELLOW: return TrafficLight.YELLOW
        if light == GREEN: return TrafficLight.GREEN

    def classify_rois(self, img, boxes):
        classes = []
        for box in boxes:
            xmin = box[0]
            xmax = box[1]
            ymin = box[2]
            ymax = box[3]
            roi = img[xmin:xmax, ymin:ymax, :]

            roi_feat = self.color_hist(roi)
            scaled_feat = self.X_scaler.transform([roi_feat])

            roi_class = self.svm.predict(scaled_feat)

            classes.append(roi_class[0])

        return classes


    def color_hist(self, img, nbins=32, bins_range=(0, 256)):
        # Compute the histogram of the color channels separately
        channel1_hist = np.histogram(img[:,:,0], bins=nbins, range=bins_range)
        channel2_hist = np.histogram(img[:,:,1], bins=nbins, range=bins_range)
        channel3_hist = np.histogram(img[:,:,2], bins=nbins, range=bins_range)
        # Concatenate the histograms into a single feature vector
        hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
        # Return the individual histograms, bin_centers and feature vector
        return hist_features
