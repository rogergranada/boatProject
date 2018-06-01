#/usr/bin/env python

"""
From a bag passed as argument, this script creates a folder with the same name
of the bag and extract all images from camera and GPS and IMU data. The recorded 
topics are:

    /camera/left_image
    /camera/right_image
    /camera/depth_image
    /gps/fix 
    /imu/data 
    /tf 
    /time_reference 
    /vel

"""

import os
import argparse
import cv2
import sys
import rosbag
import numpy as np
from cv_bridge import CvBridge
from os.path import exists, dirname, join

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import filehandler as fh


def extract_images(msg, index, dirout):
    """ Extract images from bag """
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = np.array(cv_img)
    impath = join(dirout, str(index)+'.jpg')
    cv2.imwrite(impath, img)
    return impath


def get_topics(bag):
    """ Return a list of camera topics and IMU topic """
    imu_topic = None
    cam_topic = []
    dtopics = bag.get_type_and_topic_info()[1]
    for topic in dtopics:
        topic_type = dtopics[topic][0]
        if topic_type == 'sensor_msgs/Imu':
            imu_topic = topic
        elif topic_type == 'sensor_msgs/Image':
            cam_topic.append(topic)
        else:
            logger.warning('Topic "%s" of type %s is not listed for extraction.' % (topic, topic_type))
    return imu_topic, cam_topic


def extract_imu_data(msg):
    """ Return a string with the content of IMU data """
    ori = "%f %f %f" % (msg.orientation.x, 
                        msg.orientation.y, 
                        msg.orientation.z)
    avl = "%f %f %f" % (msg.angular_velocity.x, 
                        msg.angular_velocity.y, 
                        msg.angular_velocity.z)
    lac = "%f %f %f" % (msg.linear_acceleration.x, 
                        msg.linear_acceleration.y, 
                        msg.linear_acceleration.z)
    imu_str = "%s %s %s" % (ori, avl, lac)
    return imu_str


def main(bagname):
    bagname = fh.is_file(bagname)
    fname, ext = fh.filename(bagname)
    if ext == '.bag':
        dirout = fh.mkdir_from_file(bagname)
        logger.info('Reading bag %s' % bagname)
        bag = rosbag.Bag(bagname)

        # check whether exists IMU data
        tpimu, tpcam = get_topics(bag)
        if tpimu:
            fout = open(join(dirout, 'imu_data.txt'), 'w')
        dcam = {}
        for cam_t in tpcam:
            fname = cam_t.split('/')[-1]+'.tmp'
            dirtp = fh.mkdir_from_file(join(dirout, fname))
            dcam[cam_t] = dirtp

        bagcontents = bag.read_messages()
        imumsg = ''
        index = 0
        for topic, msg, timestamp in bagcontents:
            if topic == tpimu:
                imumsg = msg
            elif topic in tpcam:
                impath = extract_images(msg, index, dcam[topic])
                if topic == tpcam[0]:
                    imu_str = extract_imu_data(imumsg)
                    fout.write('%s %s\n' % (impath, imu_str))
                    index += 1
                if index == 5:
                    break
        logger.info("Finished!\nSaved %d images in total" % index)
    else:
        logger.error('File %s is not a .bag file')
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', metavar='bag_file', help='Bag containing images')
    args = parser.parse_args()
    main(args.bagfile)
    
