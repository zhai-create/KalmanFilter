import cv2
import numpy as np



class KalmanFilter(object):
    def __init__(self):

        self.kalman = cv2.KalmanFilter(4, 2) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) # 系统测量矩阵
        self.kalman.transitionMatrix = np.array([[1, 0, 6, 0], [0, 1, 0, 6], [0, 0, 1, 0], [0, 0, 0, 1]],      np.float32) # 状态转移矩阵
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*1# 系统过程噪声协方差
        #self.kalman.measurementNoiseCov*=300
        self.current_measurement = np.array((2, 1), np.float32)
        self.last_measurement = np.array((2, 1), np.float32)
        self.current_prediction = np.zeros((2, 1), np.float32)
        self.last_prediction = np.zeros((2, 1), np.float32)
        self.error_frame = 0


    def set_current_measurement(self,array1):
        self.current_measurement=array1

    def set_last_measurement(self,array2):
        self.last_measurement=array2

    def set_last_prediction(self,array3):
        self.last_prediction=array3

    def set_current_prediction(self,array4):
        self.current_prediction=array4

    def set_measurementMatrix(self,array5):
        self.kalman.measurementMatrix=array5

    def set_transitionMatrix(self,array6):
        self.kalman.transitionMatrix=array6

    def set_processNoiseCov(self,array7):
        self.kalman.processNoiseCov=array7

    def track(self,x,y):
        self.last_prediction = self.current_prediction # 把当前预测存储为上一次预测
        self.last_measurement = self.current_measurement # 把当前测量存储为上一次测量
        if (self.last_measurement.ndim==2):
             if abs(self.last_measurement[0][0] - x) > 2 or abs(self.last_measurement[1][0] - y) > 2:
                     self.error_frame = self.error_frame + 1
             else :
                     pass
        # if (self.last_measurement.ndim == 3):
        #     if abs(self.last_measurement[0][0][0] - x) > 2 or abs(self.last_measurement[1][0][0] - y) > 2:
        #             self.error_frame = self.error_frame + 1
        #     else :
        #             pass

        if x ==0 and y == 0 : 
                self.error_frame = self.error_frame + 1
        else :
                pass
        # if self.error_frame <3 :
        #     self.current_measurement = np.array([[np.float32(self.last_prediction[0])], [np.float32(self.last_prediction[1])]])
        #
        # else:
        self.current_measurement = np.array([[np.float32(x)], [np.float32(y)]]) # 当前测量
        self.error_frame = 0

        #print("error:",self.error_frame)
        self.kalman.correct(self.current_measurement) # 用当前测量来校正卡尔曼滤波器
        self.current_prediction = self.kalman.predict() # 计算卡尔曼预测值，作为当前预测

        lmx, lmy = self.last_measurement[0], self.last_measurement[1] # 上一次测量坐标
        cmx, cmy = self.current_measurement[0], self.current_measurement[1] # 当前测量坐标
        lpx, lpy = self.last_prediction[0], self.last_prediction[1] # 上一次预测坐标
        cpx, cpy = self.current_prediction[0], self.current_prediction[1] # 当前预测坐标

        return int(cpx[0]),int(cpy[0])

        





