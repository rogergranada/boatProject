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


def store_image(msg, index, dirout):
    """ Extract images from bag """
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = np.array(cv_img)
    impath = join(dirout, str(index)+'.jpg')
    cv2.imwrite(impath, img)
    return impath


def get_topics(bag):
    """ Return a list of camera topics and IMU topic. 
        
        Code: represents the elements that contain topic
        1000 : Image (camera)
         100 : IMU data
          10 : GPS data
           1 : Control data
        Thus, a code=1110 contains data of the camera, imu and gps
    """
    tpc_imu, tpc_gps, tpc_ctr = None, None, None
    code_imu, code_gps, code_ctr = 0, 0, 0
    tpc_cam = []
    code = 0
    dtopics = bag.get_type_and_topic_info()[1]
    for topic in dtopics:
        topic_type = dtopics[topic][0]
        if topic_type == 'sensor_msgs/Imu':
            tpc_imu = topic
            code_imu = 100
        elif topic_type == 'sensor_msgs/NavSatFix':
            tpc_gps = topic
            code_gps = 10
        elif topic_type == 'mavros_msgs/RCOut':
            tpc_ctr = topic
            code_ctr = 1
        elif topic_type == 'sensor_msgs/Image':
            tpc_cam.append(topic)
            code_cam = 1000
        else:
            logger.warning('Topic "%s" of type %s is not listed for extraction.' % (topic, topic_type))
    code = code_cam + code_imu + code_gps + code_ctr
    return code, tpc_imu, tpc_gps, tpc_ctr, tpc_cam


def extract_imu_data(msg):
    """ Return a string with the content of IMU data """
    ori = "%f; %f; %f" % (msg.orientation.x, 
                        msg.orientation.y, 
                        msg.orientation.z)
    avl = "%f; %f; %f" % (msg.angular_velocity.x, 
                        msg.angular_velocity.y, 
                        msg.angular_velocity.z)
    lac = "%f; %f; %f" % (msg.linear_acceleration.x, 
                        msg.linear_acceleration.y, 
                        msg.linear_acceleration.z)
    imu_str = "%s; %s; %s" % (ori, avl, lac)
    return imu_str


def extract_gps_data(msg):
    """ Return a string with the content of IMU data """
    gps_str = "%f; %f; %f" % (msg.latitude, 
                        msg.longitude, 
                        msg.altitude)
    return gps_str

def extract_ctr_data(msg):
    """ """
    roll, x1, throttle, x2, x3, x4, x5, x6 = msg.channels
    tpc_ctr = '%d; %d' % (roll, throttle)
    return tpc_ctr


def main(bagname):
    bagname = fh.is_file(bagname)
    fname, ext = fh.filename(bagname)
    if ext == '.bag':
        dirout = fh.mkdir_from_file(bagname)
        logger.info('Reading bag %s' % bagname)
        bag = rosbag.Bag(bagname)
        logger.info(bag.get_type_and_topic_info())

        # check whether exists other data
        code, tpc_imu, tpc_gps, tpc_ctr, tpc_cam = get_topics(bag) 

        fout = open(join(dirout, 'topics.csv'), 'w')
        if tpc_imu or tpc_gps or tpc_ctr:
            content = ''
            if tpc_cam:
                content += 'path; '
            if tpc_imu:
                content += 'imu_ori_x; imu_ori_y; imu_ori_z; '
                content += 'imu_avl_x; imu_avl_y; imu_avl_z; '
                content += 'imu_lac_x; imu_lac_y; imu_lac_z; '
            if tpc_gps:
                content += 'gps_lat; gps_long; gps_alt; '
            if tpc_ctr:
                content += 'roll; throttle; '
            fout.write(content+'\n')

        # create folders for images
        dcam = {}
        for cam_t in tpc_cam:
            fname = cam_t.split('/')[-1]+'.tmp'
            dirtp = fh.mkdir_from_file(join(dirout, fname))
            dcam[cam_t] = dirtp
    
        # extract bag messages
        id_right, id_left, id_depth = 0, 0, 0
        bag_messages = bag.read_messages()
        imudata, gpsdata, ctrdata = None, None, None
        for topic, msg, timestamp in bag_messages:
            if topic == tpc_imu:
                imudata = extract_imu_data(msg)
            elif topic == tpc_gps:
                gpsdata = extract_gps_data(msg)
            elif topic == tpc_ctr:
                ctrdata = extract_ctr_data(msg)
            elif topic in tpc_cam:
                if topic == tpc_cam[1]: 
                    # store right images
                    store_image(msg, id_right, dcam[topic])
                    id_right += 1
                elif topic == tpc_cam[2]:
                    # store left images
                    store_image(msg, id_left, dcam[topic])
                    id_left += 1
                else:
                    # store depth and sensor data
                    pathimg = '%s.jpg' % id_depth
                    store_image(msg, id_depth, dcam[topic])
                    if code == 1111 and (imudata and gpsdata and ctrdata):
                        content = pathimg+'; '+imudata+'; '+gpsdata+'; '+ctrdata+';\n'
                    elif code == 1110 and (imudata and gpsdata):
                        content = pathimg+'; '+imudata+'; '+gpsdata+';\n'
                    elif code == 1101 and (imudata and ctrdata):
                        content = pathimg+'; '+imudata+'; '+ctrdata+';\n'
                    elif code == 1100 and imudata:
                        content = pathimg+'; '+imudata+';\n'
                    elif code == 1011 and (gpsdata and ctrdata):
                        content = pathimg+'; '+gpsdata+'; '+ctrdata+';\n'
                    elif code == 1010 and gpsdata:
                        content = pathimg+'; '+gpsdata+';\n'
                    elif code == 1001 and ctrdata:
                        content = pathimg+'; '+ctrdata+';\n'
                    elif code == 1000:
                        content = pathimg+';\n'
                    fout.write(content)
                    id_depth += 1
        logger.info("Finished!\nSaved %d images in total" % id_depth)
    else:
        logger.error('File %s is not a .bag file')
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', metavar='bag_file', help='Bag containing images')
    args = parser.parse_args()
    main(args.bagfile)
    
