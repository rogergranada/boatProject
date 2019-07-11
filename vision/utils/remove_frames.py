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

def main(inputfile, start_frame=-1, end_frame=-1, move=False, output=None):
    """
    Remove or move files from paths described in a pathfile. 
    This function will remove/delete all files in a pathfile 
    in case `start_frame` and `end_frame` equal to -1. In case
    of moving file, an output folder may be passed as argument.     
    """
    #print inputfile, start_frame, end_frame, move, output
    if move and not output:
        output = join(dirname(inputfile), 'removed')
        if not isdir(output):
            legger.info('Creating folder: {}'.format(output))
            os.makedir(output)

    dicfiles = {}
    pb = ProgressBar(count_lines(inputfile))
    with open(inputfile) as fin:
        for line in fin:
            path = line.strip().split()[0]
            fname, _ = splitext(basename(path))
            fname = int(fname)
            dicfiles[fname] = path
            pb.update()


    for id in sorted(dicfiles):
        if (start_frame <= id and id <= end_frame) \
           or (start_frame == -1 and end_frame == -1):
            if move:
                shutil.move(dicfiles[id], join(output, str(id)+'.jpg'))
            else:
                logger.info('Removing file {}'.format(dicfiles[id]))
                os.remove(dicfiles[id])


if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('inputfile', metavar='file_input', 
                        help='file or folder containing images.')
    parser.add_argument('-s', '--start', default=-1, type=int,
                        help='Id of the start frame to delete (included).')
    parser.add_argument('-e', '--end', default=-1, type=int,
                        help='Id of the end frame to delete (included).')
    parser.add_argument('-m', '--move', action='store_true', 
                        help='Move files instead of deleting them')
    parser.add_argument('-o', '--output', default=None, 
                        help='Output folder to move deleted files')
    args = parser.parse_args()
    
    main(args.inputfile, args.start, args.end, args.move, args.output)
