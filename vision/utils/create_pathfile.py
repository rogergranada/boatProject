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

from utils import create_paths


if __name__== "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfolder', metavar='folder_input', 
                        help='Folder containing images.')
    parser.add_argument('-o', '--output', default=None, 
                        help='File output to save all paths.')
    parser.add_argument('-p', '--path', default=None, 
                        help='Change the path of the current images to this new path.')
    args = parser.parse_args()
    
    inputfolder = realpath(args.inputfolder)
    outputfile = args.output
    if not outputfile:
        outputfile = join(dirname(inputfolder), 'paths.txt')
    create_paths(inputfolder, outputfile, path=args.path)
