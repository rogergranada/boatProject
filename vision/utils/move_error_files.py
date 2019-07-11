#/usr/bin/env python

"""
From a file containing paths of files, move them to a new folder.
"""

import os
import argparse
import sys
from os.path import exists, dirname, join, basename, isfile

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import shutil


def main(input, output, copy_files):
    with open(input) as fin:
        for line in fin:
            line = line.strip()
            if not isfile(line):
                logger.warning('File {} does not exist!'.format(line))
                continue
            fname = basename(line)
            newfile = join(output, fname)
            if copy_files:
                logger.info('Copying: {} -> {}'.format(line, newfile))
                shutil.copyfile(line, newfile)                
            else:
                logger.info('Moving: {} -> {}'.format(line, newfile))
                shutil.move(line, newfile)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filepaths', metavar='path_file', help='File containing paths to images')
    parser.add_argument('movefolder', metavar='move_folder', help='Folder where images will be moved')
    parser.add_argument('-c', '--copy', action='store_true', default=False, help='Copy files instead of move them' )
    args = parser.parse_args()
    main(args.filepaths, args.movefolder, args.copy)
    
