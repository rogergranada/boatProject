#!/usr/bin/python
#-*- coding: utf-8 -*-
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import argparse
import os
import cv2
import numpy as np
from os.path import join, dirname
from os.path import realpath, isdir, isfile

from progressbar import ProgressBar
from utils import count_lines, create_paths, move_files

def identify_green(img):
    """ Identify images containing errors in green and purple """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    green_min = np.array([30, 100, 100], dtype="uint8")
    green_max = np.array([90, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, green_min, green_max)
    nb_nz = np.count_nonzero(mask)
    perc = float(nb_nz)/(mask.shape[0]*mask.shape[1])
    if perc > 0.5:
        return True
    return False


def identify_red(img):
    """ Identify images containing errors in red and green """
    #img = cv2.imread(fimage)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_min = np.array([5, 50, 200], dtype="uint8")
    red_max = np.array([30, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, red_min, red_max)
    mask = mask[:100] # only sky
    nb_nz = np.count_nonzero(mask)
    if nb_nz > 10000:
        return True
    return False


def identify_topline(img, size=100):
    """ Identify images containing errors in red and green """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    _, nb_c, _ = hsv.shape
    mid = int(nb_c/2)
    lo_border = mid - size
    hi_border = mid + size

    green_min = np.array([30, 100, 100], dtype="uint8")
    green_max = np.array([90, 255, 255], dtype="uint8")
    mask_green = cv2.inRange(hsv, green_min, green_max)
    vec_green = mask_green[0,lo_border:hi_border]

    red_min = np.array([161, 55, 84], dtype="uint8")
    red_max = np.array([179, 255, 255], dtype="uint8")
    mask_red = cv2.inRange(hsv, red_min, red_max)
    vec_red = mask_red[0,lo_border:hi_border]
    mask = vec_green + vec_red
    nb_nz = np.count_nonzero(mask)
    perc = float(nb_nz)/len(mask)
    if perc > .3:
        return True
    return False


def verify_errors(filein, fileout):
    fout = open(fileout, 'w')

    logger.info('Checking number of images...')
    nb_imgs = count_lines(filein)
    logger.info('Input folder containing %d images' % nb_imgs)

    count = 0
    pb = ProgressBar(nb_imgs)
    with open(filein) as fin:
        for line in fin:
            pb.update()
            line = line.strip()
            if not isfile(line):
                continue

            img = cv2.imread(line)
            if identify_green(img) or identify_red(img) or identify_topline(img):
                fout.write('%s\n' % line)
                count += 1
    logger.info('{} files containing errors'.format(count))
    logger.info('Log of error files saved at: {}'.format(fileout))
    fout.close()
            

if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('finput', metavar='input', 
                        help='Path to a file or folder containing images.')
    parser.add_argument('-o', '--output', default='./error',
                        help='Path to a folder to save images with error.')
    parser.add_argument('-m', '--move', action='store_true', help='Move files in case of output is a folder.' )
    args = parser.parse_args()
    
    input = args.finput
    if isdir(input):
        input = input+'/'
        dirin = dirname(realpath(input))
        inputfile = join(dirin, 'paths.txt')
        create_paths(input, inputfile)
        input = inputfile

    output = args.output
    if not isdir(output):
        output = join(dirname(input), output)
        os.mkdir(output)
    outputfile = join(dirname(input), 'error.txt')
    
    verify_errors(input, outputfile)

    if args.move:
        move_files(outputfile, output, copy_files=False)
    else:
        move_files(outputfile, output, copy_files=True)

