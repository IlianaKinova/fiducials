import numpy as np
import cv2
from cv_bridge import *
import rospy as ros
from sensor_msgs.msg import Image
import sys

class imageViewer:
    def __init__(self, topic:str, encoding:str):
        """Simple viewer"""
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

def main():
    ros.init_node('image_viewer', anonymous=True)
    ros.loginfo("Starting color")
    colorView = imageViewer('/camera/color/image_raw', 'bgr8')
    ros.loginfo('Started color')
    ros.loginfo('Starting depth')
    depthView = imageViewer('/camera/depth/image_raw', '16UC1')
    ros.loginfo('Started depth')

    try:
        ros.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()