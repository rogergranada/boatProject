#!/usr/bin/python
#-*- coding: utf-8 -*-

import os
import argparse
import cv2
import numpy as np
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile
from scipy import spatial
from matplotlib import pyplot as plt
import math

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

from progressbar import ProgressBar

def identify_green(img):
    #img = cv2.imread(fimage)
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
    #img = cv2.imread(fimage)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_min = np.array([5, 50, 200], dtype="uint8")
    red_max = np.array([30, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, red_min, red_max)
    mask = mask[:200] # only sky
    nb_nz = np.count_nonzero(mask)
    if nb_nz > 10000:
        return True
    return False

def error_channel(fimage):
    img = cv2.imread(fimage)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #green_min = np.array([30, 100, 100], dtype="uint8")
    #green_max = np.array([90, 255, 255], dtype="uint8")
    green_min = np.array([5, 50, 200], dtype="uint8")
    green_max = np.array([30, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, green_min, green_max)
    #print mask.shape
    mask = mask[:200]
    #print mask.shape
    nb_nz = np.count_nonzero(mask)
    perc = float(nb_nz)/(mask.shape[0]*mask.shape[1])
    print perc
    print nb_nz
    plt.imshow(mask, cmap='gray')
    plt.show()

def __error_channel(fimage):
    #f = plt.figure(1)
    color = ('b','g','r')
    img = cv2.imread(fimage)
    hist_channel = []
    for channel, col in enumerate(color):
        hist = cv2.calcHist([img],[channel],None,[256],[0,256])
        #plt.xlim([0,256])
        #plt.plot(hist, color=col)
        hist = hist.reshape(1, 256)
        hist_channel.append(list(hist[0]))

    #print len(hist_channel)

    #calculate distance
    #hist_b = np.array(hist_channel[0])[250:]
    #hist_g = np.array(hist_channel[1])[250:]
    #hist_r = np.array(hist_channel[2])[250:]
    
    vb = hist_channel[0][-1]
    vg = hist_channel[1][-1]
    vr = hist_channel[2][-1]

    #dist_rg = np.linalg.norm(hist_r - hist_g)
    #dist_rb = np.linalg.norm(hist_r - hist_b)
    #dist_gb = np.linalg.norm(hist_g - hist_b)

    #dist_rg = cosine_similarity(hist_r, hist_g)
    #dist_rb = cosine_similarity(hist_r, hist_b)
    #dist_gb = cosine_similarity(hist_g, hist_b)
    
    #plt.show()
    total = vr+vg+vb

    pr = vr/float(total)
    pg = vg/float(total)
    pb = vb/float(total)

    print pr+pg
    print pr
    print pg
    print pb
    #print 'B', vb/float(total)

    if pr+pg > 0.9 or pg < 0.1:
        return True
    #if dist_rg < 0.9: # or dist_rb < 0.8 or dist_gb < 0.8:
    #    return True
    #raw_input()
    return False

def verify_errors(filein, im_true, im_error, fileout):
    fout = open(fileout, 'w')

    logger.info('Checking number of images...')
    for nb_imgs, _ in enumerate(open(filein), start=1): pass
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
            if identify_green(img) or identify_red(img):
                fout.write('%s\n' % line)
                count += 1

    print '%d files containing errors' % count
    fout.close()
            



def green_error(filein, im_true, im_error, fileout):
    color = ('b','g','r')

    fout = open(fileout, 'w')

    # histogram correct
    logger.info('Loading true image: %s' % im_true)
    img = cv2.imread(im_true)
    hist_true = []
    for channel, col in enumerate(color):
        hist = cv2.calcHist([img],[channel],None,[256],[0,256])
        hist = hist.reshape(1, 256)
        hist_true.extend(list(hist[0]))
    #print len(hist_true)

    # histogram error
    logger.info('Loading error image: %s' % im_error)
    img = cv2.imread(im_error)
    hist_error = []
    for channel, col in enumerate(color):
        hist = cv2.calcHist([img],[channel],None,[256],[0,256])
        hist = hist.reshape(1, 256)
        hist_error.extend(list(hist[0]))
    #print len(hist_error)

    logger.info('Checking number of images...')
    for nb_imgs, _ in enumerate(open(filein), start=1): pass
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

            hist_line = []
            for channel, col in enumerate(color):
                hist = cv2.calcHist([img],[channel],None,[256],[0,256])
                hist = hist.reshape(1, 256)
                hist_line.extend(list(hist[0]))
            
            #calculate distance
            hist_line = np.array(hist_line)
            hist_true = np.array(hist_true)
            hist_error = np.array(hist_error)
            dtrue = np.linalg.norm(hist_line - hist_true)
            dfalse = np.linalg.norm(hist_line - hist_error)

            if dtrue < dfalse:
                pass #print 'True:', line
            else:
                count += 1
                #print 'False:', line 
                fout.write(line+'\n')
    print '%d files containing errors' % count
    fout.close()
            





def create_file_paths(inputfolder, fileoutput):
    fout = open(fileoutput, 'w')

    path = realpath(inputfolder)
    files = os.listdir(inputfolder)
    names = []
    for img in files:
        name, ext = splitext(img)
        if ext == '.jpg':
            names.append(int(name))
    for name in sorted(names):
        fout.write('%s\n' % join(inputfolder, str(name)+'.jpg'))
    print 'Saved paths in file: %s' % fileoutput


if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('inputfile', metavar='file_input', 
                        help='file or folder containing images.')
    parser.add_argument('image_true', metavar='true_image', 
                        help='Valid image.')
    parser.add_argument('image_error', metavar='error_image', 
                        help='Image containing errors.')
    parser.add_argument('outputfile', metavar='fileout', 
                        help='Path to file to save images with error.')
    args = parser.parse_args()
    
    #error_channel(args.inputfile)
    
    input = args.inputfile
    im_true = args.image_true
    im_error = args.image_error
    if isdir(input):
        input = input+'/'
        dirin = dirname(realpath(input))
        outputfile = join(dirin, 'paths.txt')
        create_file_paths(input, outputfile)
    elif isfile(input):
        outputfile = input    
    
    verify_errors(input, im_true, im_error, args.outputfile)
    
