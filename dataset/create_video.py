#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
Create a video with images and the bounding box annotation.
"""
import sys
sys.path.insert(0, '..')
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import os
import argparse
import cv2
import numpy as np
from os.path import join, isdir, splitext, basename, dirname, exists
from matplotlib import colors

import progressbar as pbar
import filehandler as fh

BBOX_COLOR = [57,255,20]


def load_annotation(file_annotation):
    dic = {}
    with open(file_annotation) as fin:
        for i, line in enumerate(fin):
            if i == 0: continue
            path, xmin, ymin, xmax, ymax, _ = line.strip().split(',')
            id = int(path.replace('"', '').replace('.jpg', ''))
            if id in dic:
                dic[id].append([int(float(xmin)), int(float(ymin)), int(float(xmax)), int(float(ymax))])
            else:
                dic[id] = [[int(float(xmin)), int(float(ymin)), int(float(xmax)), int(float(ymax))]]
    return dic


COLOR = (0, 0, 255) #BGR

def create_video_from_file(inputfile, pathfile):
    img_array = []
    
    fname, _ = splitext(basename(inputfile))
    fnameout= join(dirname(inputfile), fname+'.avi')

    dic = load_annotation(inputfile)

    label = 'obstacle'
    metadata = False
    with open(pathfile) as fin:
        for line in fin:
            fname = line.strip()
            id, _ = splitext(basename(fname))
            id = int(id)
            img = cv2.imread(fname)
            if not metadata:
                height, width, layers = img.shape
                size = (width, height)
                metadata = True

            if id in dic:
                for xmin, ymin, xmax, ymax in dic[id]:
                    cv2.rectangle(img, (xmin,ymin), (xmax,ymax), COLOR, 4)
                    cv2.putText(img, label, (xmin-10,ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR, 1)
            img_array.append(img)

    print('Saving {} files '.format(img_array))
    out = cv2.VideoWriter(fnameout, cv2.VideoWriter_fourcc(*'DIVX'), 5, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    #'''

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfile', metavar='file_input', help='File containing LIS annotation.')
    parser.add_argument('inputfile2', metavar='file_input2', help='File containing LIS annotation.')
    args = parser.parse_args()
    create_video_from_file(args.inputfile, args.inputfile2)
