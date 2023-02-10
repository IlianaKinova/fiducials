from time import sleep
import numpy as np
import cv2
from cv2 import aruco




colorStream = "rtsp://192.168.1.10/color"

if __name__ == "__main__":
    try:
        # Define aruco params
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)


        color=None

        # Open color stream
        color = cv2.VideoCapture(colorStream)
        if not color.isOpened():
            raise IOError("Cannot open color")
        

        while True:
            # Read frame
            ret, frame = color.read()

            # Resize and show
            frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.imshow("Color", frame)

            # This is required, OpenCV hates if you use the original
            framecp = frame.copy()
            # Convert to gray and detect markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Draw markers on frame
            frame_markers = aruco.drawDetectedMarkers(framecp, corners, ids)

            # Show image with markers
            cv2.imshow("Markers", frame_markers)

            c = cv2.waitKey(1)
            if c == 27: # Press 'esc' to end
                break
    except InterruptedError:
        pass
    except IOError as e:
        print(e)
    # Close color stream if opened
    if color:
        color.release()
    # Destroy windows
    cv2.destroyAllWindows()
