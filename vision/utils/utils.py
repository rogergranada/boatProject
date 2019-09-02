#/usr/bin/env python

"""
Functions that are useful for many applications 
"""
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import os
from os.path import realpath, splitext, basename


def create_paths(inputfolder, fileoutput, path=None):
    fout = open(fileoutput, 'w')

    files = os.listdir(inputfolder)
    names = []
    for img in files:
        name, ext = splitext(basename(img))
        print ext
        if ext == '.jpg':
            names.append(int(name))
    for name in sorted(names):
        if path:
            fout.write('%s/%d.jpg\n' % (path, name))
        else:
            fout.write('%s/%d.jpg\n' % (inputfolder, name))
    logger.info('Saved paths in file: %s' % fileoutput)


def count_lines(inputfile):
    with open(inputfile) as fin:
        for i, _ in enumerate(fin, start=1):
            pass
    return i
