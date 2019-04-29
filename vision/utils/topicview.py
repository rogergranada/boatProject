#!/usr/bin/env python

"""
"""
import argparse
import sys
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt

class CameraViewer:
  def __init__(self, mode):
    """ interface between ros and opencv """
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)

    # Subscribe to ROS topics
    if mode == 'left' or mode == 'l':
        self.cam = rospy.Subscriber('/camera/left_image', Image, self.image_callback)
    elif mode == 'right' or mode == 'r':
        self.cam = rospy.Subscriber('/camera/right_image', Image, self.image_callback)
    elif mode == 'depth' or mode == 'd':
        self.cam = rospy.Subscriber('/camera/depth_image', Image, self.image_callback)


  def image_callback(self, msg):
    """
    Callback of the ROS camera subscription
    """
    # reads the image from camera topic and transform to OpenCV format
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.show()
    key = raw_input('Press ctrl+C for exit!')
    if key == 'e':
        sys.exit(0)

    #cv2.imshow("window", image)
    #cv2.waitKey(3)


def main(mode):
    rospy.init_node('listener')
    viewer = CameraViewer(mode)
    rospy.spin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', metavar='cam_mode', help='Camera modes: left (l), right (r), depth (d)', default='left')
    args = parser.parse_args()
    main(args.mode)
    
