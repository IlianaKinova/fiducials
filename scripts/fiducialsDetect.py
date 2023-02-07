from time import sleep
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd




def color2depth(xy):
    return 

colorStream = 0
colorStream = "rtsp://192.168.1.10/color"
depthStream = 0
depthStream = "rtsp://192.168.1.10/depth"

if __name__ == "__main__":
    try:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        useDepth = True
        color=None
        depth=None

        color = cv2.VideoCapture(colorStream)
        if not color.isOpened():
            raise IOError("Cannot open color")
        depth = cv2.VideoCapture(depthStream)
        if not depth.isOpened():
            useDepth = False
            print("Cannot open depth")
            # raise IOError("Cannot open depth")
        

        while True:
            ret, frame = color.read()
            frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.imshow("Color", frame)

            
            framecp = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(framecp, corners, ids)
            if useDepth:
                for corner in corners:
                    c = corner[0]
                    cc = c[0]
                    orig = tuple(cc.astype('int32'))
                    cv2.addText(frame_markers, f"dist:{1}", orig,cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))


            cv2.imshow("Markers", frame_markers)

            c = cv2.waitKey(1)
            if c == 27:
                break
            # if ids != None:
            #     for i in range(len(ids)):
            #         c = corners[i][0]
            #         plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
            #     plt.legend()
            # plt.show(block=False)
    except InterruptedError:
        pass
    except IOError as e:
        print(e)
    if color:
        color.release()
    if depth:
        depth.release()
    cv2.destroyAllWindows()
