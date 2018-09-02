#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
This script extracts bounding boxes from name files.
"""
import sys
sys.path.insert(0, '..')
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse
import os

from os.path import realpath, dirname, splitext, basename, join, isdir
from shutil import copyfile
from PIL import Image


def index_points(pathimg):
    """ Original name as: image_19_478_280_66_123.jpg """
    im_width, im_height = Image.open(pathimg).size
    arr = basename(pathimg).split('_')
    index = int(arr[1])
    x = int(arr[2])
    y = int(arr[3])
    w = int(arr[4])
    h = int(arr[5].replace('.jpg',''))
    if w+x < 0 or h+y < 0 or w > im_width or h > im_height:
        return index, 0, 0, 0, 0
    
    if x < 0: 
        w += x
        x = 0
    if y < 0: 
        h += y
        y = 0
    return index, x, y, w, h


def rename_files(path_in, output, index):
    """ 
    Images names as: 00001.jpg 
    Original name as: image_19_478_280_66_123.jpg
        where image_19 is the name of the file and
        478: bounding box xmin
        280: bounding box ymin
        66: bounding box width
        123: bounding box height 
    """
    dirout = join(output, 'Data')
    if not isdir(dirout):
        os.makedirs(dirout)
    _, x, y, w, h = index_points(path_in)
    pathout = join(dirout, str(index)+'.jpg')
    try:
        copyfile(path_in, pathout)
    except:
        logger.error('Could not copy file!')
        logger.error('Path input: %s' % path_in)
        logger.error('Path output: %s' % pathout)
        sys.exit(0)
    return pathout, x, y, w, h



def main(inputfolder, output):
    """ Rename files and generates a pathfile.csv
        CSV file has the form:
        'filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'
    """
    if output:
        output = realpath(output)
    else:
        output = join(dirname(input), 'Renamed')
        if not isdir(output):
            os.makedirs(output)  
    dimgs = {}
    inputfolder = realpath(inputfolder)
    for root, dirs, files in os.walk(inputfolder):
        for img in files:
            imgpath = join(root, img)
            index, _, _, _, _ = index_points(imgpath)
            dimgs[index] = imgpath

    with open(join(output, 'paths.csv'), 'w') as fout:
        fout.write('filename,width,height,class,xmin,ymin,xmax,ymax\n')
        for i, index in enumerate(sorted(dimgs), start=1):
            pathout, xmin, ymin, w, h = rename_files(dimgs[index], output, i)
            xmax = xmin + w
            ymax = xmin + h
            fout.write('%s,%d,%d,obstacle,%d,%d,%d,%d\n' % (pathout, w, h, xmin, ymin, xmax, ymax))



if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('inputfolder', metavar='imgs_folder', help='Folder containing images')
    argparser.add_argument('-o', '--output', help='File to save annotations', default=None)
    args = argparser.parse_args()
    main(args.inputfolder, args.output)
    

