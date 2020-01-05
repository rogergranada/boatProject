#/usr/bin/env python

"""
Functions that are useful for many applications 
"""
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import os
from os.path import realpath, splitext, basename, isfile, join
import shutil


def move_files(inputfile, folder_out, copy_files=True):
    with open(inputfile) as fin:
        for line in fin:
            line = line.strip()
            if not isfile(line):
                logger.warning('File {} does not exist!'.format(line))
                continue
            fname = basename(line)
            newfile = join(folder_out, fname)
            if copy_files:
                logger.info('Copying: {} -> {}'.format(line, newfile))
                shutil.copyfile(line, newfile)                
            else:
                logger.info('Moving: {} -> {}'.format(line, newfile))
                shutil.move(line, newfile)


def create_paths(inputfolder, fileoutput, path=None):
    """ Create a file containing all images from input folder """
    fout = open(fileoutput, 'w')

    files = os.listdir(inputfolder)
    names = []
    for img in files:
        name, ext = splitext(basename(img))
        if ext == '.jpg':
            names.append(int(name))
    for name in sorted(names):
        if path:
            fout.write('%s/%d.jpg\n' % (path, name))
        else:
            fout.write('%s/%d.jpg\n' % (inputfolder, name))
    logger.info('Saved paths in file: %s' % fileoutput)


def count_lines(inputfile):
    """ Count the number of lines of the input file """
    with open(inputfile) as fin:
        for i, _ in enumerate(fin, start=1):
            pass
    return i
