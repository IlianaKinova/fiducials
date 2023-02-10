"""
This is supposed to be able to handle a 16 bit gray stream,
but I just can't get it to work.
I know OpenCV does not by default read 16 bit VideoCaptures,
however, GStreamer can, and OpenCV has integration for GStreamer.
"""




from time import sleep
import numpy as np
import cv2
from cv_bridge import *
import rospy as ros
from sensor_msgs.msg import Image

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst


# url and ros topic definitions
depthStream = "rtsp://10.0.100.131/depth"
rosDepth = "/camera/depth/image"


def open_cam_rtsp(uri, width, height, latency):
    """[FAIL] Method 1 to open rtsp depth stream with opencv 
    using GStreamer"""

    # GStreamer params
    gst_str = ("rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! "
               "nvvidconv ! video/x-raw, format=(string)BGRx ! "
               "videoconvert ! appsink").format(uri, latency)#, width, height)
    # Open camera
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    return cap

def open_cam_rtsp2():
    """[FAIL] Open camera with opencv using v4l2src with GStreamer
    """
    cap = cv2.VideoCapture(f"v4l2src location={depthStream} ! video/x-raw,format=GRAY16_LE ! videoconvert ! video/x-raw, format=GRAY8 ! appsink", cv2.CAP_GSTREAMER)
    return cap

def open_cam_rtsp3(uri):
    """[FAIL] Tried using simply v4l2"""
    cap=cv2.VideoCapture(uri, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',''))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return cap

def open_cam_rtsp4():
    """[FAIL] Again using GStreamer, I have put dozens of hours in this"""
    cap = cv2.VideoCapture(f"v4l2src location={depthStream} ! videoscale ! video/x-raw, width=1024, height=768 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    return cap

def open_cam_rtsp5():
    cap = cv2.VideoCapture(depthStream, cv2.CAP_GSTREAMER)
    return cap

def open_cam_rtsp6():
    gst_str = f"rtspsrc location={depthStream} ! queue ! videoconvert ! video/x_raw, format=GRAY8 ! appsink"
    cap = cv2.VideoCapture(depthStream, cv2.CAP_GSTREAMER)
    return cap

def testRTSP():
    """Pick your poison, none of them work"""

    capType = [
        open_cam_rtsp(depthStream, 480, 270, 300),
        open_cam_rtsp2(),
        open_cam_rtsp3(depthStream),
        open_cam_rtsp4(),
        open_cam_rtsp5(),
        open_cam_rtsp6()]

    for cap in capType:

        # Calling the constructor of VideoCapture is equivalent to
        # calling cap.open(...)
        if not cap.isOpened():
            print("Cannot open depth")
            sleep(0.5)
        else:
            print("SUCCESS")
            cap.release()


class imageViewer:
    """[SUCCESS]"""
    def __init__(self, topic:str, encoding:str):
        """Simple viewer from ros to OpenCV"""
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber(topic,Image,self.callback)
        self.name = topic.split('/')[2] # Get the stream name from the topic
        self.encoding = encoding

    def callback(self,data):
        key = cv2.waitKey(3)
        if key == ord('q'):
            ros.signal_shutdown('User requested quit')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, self.encoding)
        except CvBridgeError as e:
            print(e)
        cv2.imshow(self.name, cv_image)

def testROS():
    """[SUCCESS] Start the kinova vision: roslaunch kinova_vision kinova_vision_rgbd.launch"""
    """Run using roslaunch fiducials testDepth.launch"""

    ros.init_node('read_depth')
    conv = imageViewer(rosDepth, '16UC1')
    rate = ros.Rate(30)
    while not ros.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    print(cv2.getBuildInformation())
    # gstreamerMain();exit()
    testRTSP();exit()
    try:
        testROS()
    except KeyboardInterrupt:
        pass
    except ros.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

    