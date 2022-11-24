from orange_detector import OrangeDetector
from KF import KalmanFilter
import numpy as np
import cv2

kf=KalmanFilter()
kf.set_last_measurement(np.array((2, 1), np.float32))
kf.set_current_measurement(np.array((2, 1), np.float32))

kf.set_last_prediction(np.zeros((2, 1), np.float32))
kf.set_current_prediction(np.zeros((2, 1), np.float32))

kf.set_measurementMatrix(np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32))# 系统测量矩阵
kf.set_transitionMatrix(np.array([[1, 0, 6, 0], [0, 1, 0, 6], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32))  # 状态转移矩阵
kf.set_processNoiseCov(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*0.03 ) # 系统过程噪声协方差



# cap=cv2.VideoCapture("E:\\Pysource-Kalman-filter\\Pysource Kalman filter\\orange.mp4")
cap=cv2.VideoCapture(0)

detector=OrangeDetector()

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, dsize=(800, 600))
    if ret == False:
        break

    x, y, x2, y2 = detector.detect(frame)
    xx = int((x + x2) / 2)
    yy = int((y + y2) / 2)
    px, py = kf.track(xx, yy)
    px2, py2 = kf.track(x2, y2)
    print("真实：",(x,y))
    print("预测：",(px2,py2))

    #cv2.rectangle(frame, (x, y), (x2, y2), (0, 0, 255), 5)
    #cv2.rectangle(frame, (px, py), (px2, py2), (255, 0, 0), 5)


    cv2.circle(frame,(xx,yy),20,(0,0,255),5)
    cv2.circle(frame,(px,py),20,(255,0,0),5)
    # cv2.rectangle(frame,(x,y),(x2,y2),(0,0,255),4)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(50)
    if key == 27:
        # 通过esc键退出摄像
        cv2.destroyAllWindows()
        break
    elif key == ord("s"):
        # 通过s键保存图片，并退出。
        cv2.imwrite("image2.jpg", frame)
        cv2.destroyAllWindows()
        break

cap.release()


