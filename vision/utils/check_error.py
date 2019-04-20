#!/usr/bin/python
#-*- coding: utf-8 -*-

import os
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np

def verify_errors(filein, im_true, im_error, fileout):
    color = ('b','g','r')

    fout = open(fileout, 'w')

    # histogram correct
    img = cv2.imread(im_true)
    hist_true = []
    for channel, col in enumerate(color):
        hist = cv2.calcHist([img],[channel],None,[256],[0,256])
        hist = hist.reshape(1, 256)
        hist_true.extend(list(hist[0]))
    #print len(hist_true)

    # histogram error
    img = cv2.imread(im_error)
    hist_error = []
    for channel, col in enumerate(color):
        hist = cv2.calcHist([img],[channel],None,[256],[0,256])
        hist = hist.reshape(1, 256)
        hist_error.extend(list(hist[0]))
    #print len(hist_error)

    count = 0
    with open(filein) as fin:
        for line in fin:
            line = line.strip()
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
        fout.write('%s%d.jpg\n' % (inputfolder, name))
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
    
