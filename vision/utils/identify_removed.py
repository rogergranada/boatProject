#!/usr/bin/python
#-*- coding: utf-8 -*-

import os
import shutil
import argparse
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

from progressbar import ProgressBar
from utils import count_lines
import filehandler as fh

def main(inputfile, keepfile, output=None):
    """
    Create a file containing all the images that were removed from the dataset.   
    """
    #print inputfile, start_frame, end_frame, move, output
    if not output:
        output = join(dirname(inputfile), 'path_removed.txt')

    existent = fh.PathfileHandler.load_file(keepfile)[0]

    pfi = fh.PathfileHandler(inputfile)
    nb_removed = 0
    with open(output, 'w') as fout:
        for path in pfi:
            if path not in existent:
                nb_removed += 1
                fout.write('%s\n' % path)
    logger.info('Saved %d files that were removed.' % nb_removed)



if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('inputfile', metavar='file_input', 
                        help='file containing all image paths of the dataset.')
    parser.add_argument('keepfile', metavar='file_kept', 
                        help='file containing images after removing error frames.')
    parser.add_argument('-o', '--output', default=None, 
                        help='Output folder to move deleted files')
    args = parser.parse_args()
    
    main(args.inputfile, args.keepfile, args.output)
