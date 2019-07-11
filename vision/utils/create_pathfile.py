#/usr/bin/env python

"""
From a file containing paths of files, move them to a new folder.
"""

import os
import argparse
import sys
from os.path import exists, dirname, splitext 
from os.path import join, basename, isfile, realpath

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)


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
        fout.write('%s/%d.jpg\n' % (inputfolder, name))
    print('Saved paths in file: %s' % fileoutput)


def main(inputfolder, outputfile):
    inputfolder = realpath(inputfolder)
    if not outputfile:
        outputfile = join(dirname(inputfolder), 'paths.txt')
    create_file_paths(inputfolder, outputfile)
    

if __name__== "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfolder', metavar='folder_input', 
                        help='Folder containing images.')
    parser.add_argument('-o', '--output', default=None, 
                        help='File output to save all paths.' )
    args = parser.parse_args()
    
    main(args.inputfolder, args.output)
