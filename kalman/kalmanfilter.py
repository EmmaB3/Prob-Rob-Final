# Kalman Filter Class
# Adapted by Allison Moore
# 
# Referenced from 
#https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
# and standard cv2 Kalman Filter Library https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html

#imports
import cv2 as cv
import numpy as np

class Kalman:
    kalman = cv.KalmanFilter(4, 2)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def estimate(self, x, y):

        #send measured values to kalmanfilter
        self.kalman.correct(np.array([[np.float32(x)], [np.float32(y)]]))

        #find prediction based on measured values
        p = self.kalman.predict()

        #return new values
        newx = int(p[0])
        newy = int(p[1])

        return newx,newy
