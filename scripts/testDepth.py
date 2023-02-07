import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import *
import rospy as ros
from sensor_msgs.msg import Image
import sys

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

depthStream = "rtsp://10.0.100.131/depth"
rosDepth = "/camera/depth/image"

def open_cam_rtsp(uri, width, height, latency):
    gst_str = ("rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! "
               "nvvidconv ! video/x-raw, format=(string)BGRx ! "
               "videoconvert ! appsink").format(uri, latency)#, width, height)
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    cap.setExceptionMode(True)
    if not cap.isOpened():
        cap.open(gst_str, cv2.CAP_GSTREAMER)
    return cap

def open_cam_rtsp2():
    cap = cv2.VideoCapture(f"v4l2src location={depthStream} ! video/x-raw,format=GRAY16_LE ! videoconvert ! video/x-raw, format=GRAY8 ! appsink", cv2.CAP_GSTREAMER)
    return cap

def open_cam_rtsp3(uri):
    cap=cv2.VideoCapture(uri, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',''))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return cap

def open_cam_rtsp4():
    cap = cv2.VideoCapture("v4l2src ! videoscale ! video/x-raw, width=1024, height=768 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    return cap


def testRTSP():
    # depth = open_cam_rtsp(depthStream, 480, 270, 300)
    # depth = open_cam_rtsp2()
    # depth = open_cam_rtsp3(depthStream)
    depth = open_cam_rtsp4()
    if not depth.isOpened():
        useDepth = False
        print("Cannot open depth")
    else:
        depth.release()

class imageConverter:
    def __init__(self, rostopic:str):
        self.rostopic = rostopic
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber(rostopic,Image,self.callback, queue_size=5)

    def callback(self, data:Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
        except CvBridgeError as e:
            print(e)
            return
        cv_image = cv_image - cv_image.min()
        delta = cv_image.max() - cv_image.min()
        cv_image = cv_image / delta * 255
        cv_image8 = cv2.cvtColor(cv_image.astype('uint8'), cv2.COLOR_GRAY2BGR)

        cv2.imshow("Image window", cv_image8)


def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        loop.quit()
    return True


def new_sample(sink, data) -> Gst.FlowReturn:
    sample = sink.emit('pull-sample')
    arr = extract_data(sample)
    pipeLineOutputVec.put(arr.copy())
    time.sleep(0.01)
    return Gst.FlowReturn.OK

def gstreamerMain():
    GObject.threads_init()
    Gst.init(None)
        
    playbin = Gst.ElementFactory.make("playbin", None)
    if not playbin:
        sys.stderr.write("'playbin' gstreamer plugin missing\n")
        sys.exit(1)

    # take the commandline argument and ensure that it is a uri
    if Gst.uri_is_valid(depthStream):
        uri = depthStream
    else:
        uri = Gst.filename_to_uri(depthStream)
    playbin.set_property('uri', uri)

    # create and event loop and feed gstreamer bus mesages to it
    loop = GObject.MainLoop()

    bus = playbin.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)

    src = Gst.ElementFactory.make("appsrc", "src")
    sink = Gst.ElementFactory.make("appsink", "sink")
    sink.set_property('emit-signals', True)
    sink.connect('new-sample', new_sample, None)
    ### elements
    pipeline_elements = [src, sink]

    establish_pipeline(pipeline, pipeline_elements)

    # start play back and listed to events
    playbin.set_state(Gst.State.PLAYING)
    try:
        
        loop.run()
    except:
        pass
    
    # cleanup
    playbin.set_state(Gst.State.NULL)

def testROS():
    bridge = CvBridge()

    ros.init_node('read_depth')
    conv = imageConverter(rosDepth)
    rate = ros.Rate(30)
    while not ros.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    # gstreamerMain();exit()
    testRTSP();exit()
    try:
        testROS()
    except KeyboardInterrupt:
        pass
    except ros.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

    