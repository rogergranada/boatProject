#/usr/bin/env python

import os
import cv2
import sys
import rosbag
import numpy as np
from cv_bridge import CvBridge

def extractor(bagname):
	root_dir = "./%s" % bagname.split('.bag')[0]
	color_dir = "%s/color" % root_dir
	depth_dir = "%s/depth" % root_dir
	
	if not (os.path.exists(root_dir)):
		os.makedirs("%s" % color_dir)
		os.makedirs("%s" % depth_dir)

	bridge = CvBridge()
	bag = rosbag.Bag(bagname)
	print "Saving color images to %s" % color_dir
	k = 0
	for topic, msg, t in bag.read_messages(topics=['/zed/left/image_raw_color']):
		cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		img = np.array(cv_img)
		cv2.imwrite("%s/%d.jpg" % (color_dir, k), img)
		k = k+1

	print "Done\nSaved %d color images in total" % k

	print "Saving depth images to %s" % depth_dir
	k = 0
	for topic, msg, t in bag.read_messages(topics=['/zed/depth/depth_registered']):
		cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		img = np.array(cv_img)
		cv2.imwrite("%s/%d.jpg" % (depth_dir, k), img)
		k = k+1

	print "Done\nSaved %d depth images in total" % k



if __name__ == "__main__":
	if (len(sys.argv) < 2):
		print "Usage: python image_extractor.py <bag-filename>\n"
		exit()

	extractor(sys.argv[1])