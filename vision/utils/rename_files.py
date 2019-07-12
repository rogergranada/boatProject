#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
This script rename files based on a file containing old_path and new_path.
For example, consider the file `rename.txt` with the following lines:

10.jpg 0.jpg
11.jpg 1.jpg

The script will rename the file from `input folder` that has the name 10.jpg
to 0.jpg and the file 11.jpg to 1.jpg.
"""

import os
import shutil
import argparse
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import filehandler as fh

def main(inputfile, inputfolder):
    """ Rename files from inputfolder based on inputfile """
    pf = fh.PathfileHandler(inputfile)
    for oldname, newname in pf:
        oldpath = join(inputfolder, oldname)
        newpath = join(inputfolder, newname)
        if not isfile(oldpath):
            logger.error('File %s does not exist!' % oldpath)
            sys.exit(1)
        shutil.move(oldpath, newpath)
        logger.debug('Renaming %s -> %s' % (oldpath, newpath))
    logger.info('A total of %d renamed files' % pf.nb_lines)
    

if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('inputfile', metavar='file_input', 
                        help='file containing old names and new names.')
    parser.add_argument('inputfolder', metavar='folder_input', 
                        help='folder containing files to be renamed.')
    args = parser.parse_args()
    
    main(args.inputfile, args.inputfolder)
