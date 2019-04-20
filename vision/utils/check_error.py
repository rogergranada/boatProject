#!/usr/bin/python
#-*- coding: utf-8 -*-

import os
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np

def verify_errors(filein):
    color = ('b','g','r')
    with open(filein) as fin:
        for line in fin:
            line = line.strip()
            img = cv2.imread(line)

            for channel, col in enumerate(color):
                hist = cv2.calcHist([img],[channel],None,[256],[0,256])
                plt.plot(hist, color = col)

                plt.xlim([0,256])
                plt.title('Histogram for color scale picture')
    plt.show()

    while True:
        k = cv2.waitKey(0) & 0xFF     
        if k == 27: break             # ESC key to exit 
    cv2.destroyAllWindows()


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
    args = parser.parse_args()
    
    input = args.inputfile
    if isdir(input):
        input = input+'/'
        dirin = dirname(realpath(input))
        outputfile = join(dirin, 'paths.txt')
        create_file_paths(input, outputfile)
    elif isfile(input):
        outputfile = input    
    
    verify_errors(input)
    
