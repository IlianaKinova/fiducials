import numpy as np
import cv2
from cv_bridge import *
import rospy as ros
from sensor_msgs.msg import Image
import sys
import multiprocessing as mp

class imageViewer:
    def __init__(self, topic:str, encoding:str, q:mp.Queue):
        """Simple viewer"""
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber(topic,Image,self.callback)
        self.name = topic.split('/')[2] # Get the stream name from the topic
        self.encoding = encoding
        self.q = q

    def callback(self,data):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, self.encoding)
        except CvBridgeError as e:
            print(e)
        if self.encoding == '16UC1':
            delta = cv_image.max() - cv_image.min()

            cv_image = np.uint8(np.float32(cv_image - cv_image.min()) / delta * 255)
        self.q.put((self.name, cv_image))

def main():
    ros.init_node('image_viewer', anonymous=True)
    ros.loginfo("Starting color")
    q = mp.Queue()
    colorView = imageViewer('/camera/color/image_raw', 'bgr8', q)
    ros.loginfo('Started color')
    ros.loginfo('Starting depth')
    depthView = imageViewer('/camera/depth/image_raw', '16UC1', q)
    ros.loginfo('Started depth')

    
    try:
        while True:
            name, img = q.get()
            cv2.imshow(name, img)
            ros.sleep(0.01)
            key = cv2.waitKey(1)
            if key == 27: # Press esc to quit
                ros.signal_shutdown('User requested quit')
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()