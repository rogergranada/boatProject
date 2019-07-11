#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
Based on files existent in `left_camera` remove files from `right_camera` or `depth_camera`
that do not exist in `left_camera` folder.
"""
import os
import shutil
import argparse
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

from progressbar import ProgressBar
import filehandler as fh

def main(reference, target, move=False, output=None):
    """ Remove images from `target` that do not exist in `reference` """
    if move and not output:
        output = join(dirname(target), 'removed')
        if not isdir(output):
            legger.info('Creating folder: {}'.format(output))
            os.makedir(output)
    
    if isdir(reference):
        fout = join(dirname(reference), 'paths_ref.txt')
        fh.create_paths(reference, fout)
        reference = fout
    if isdir(target):
        fout = join(dirname(target), 'paths_target.txt')
        fh.create_paths(target, fout)
        target = fout

    pf = fh.PathfileHandler(reference, display=False)
    dtrue = {}
    for _ in pf:
        name, ext = fh.filename(pf.path)
        fname = '{}{}'.format(name, ext)
        dtrue[fname] = ''

    pf = fh.PathfileHandler(target, display=False)
    for _ in pf:
        name, ext = fh.filename(pf.path)
        fname = '{}{}'.format(name, ext)
        if not dtrue.has_key(fname):
            if move:
                new_path = join(output, fname)
                logger.info('Moving file: %s -> %s' % (pf.path, new_path))
                shutil.move(pf.path, new_path)
            else:
                logger.info('Removing file: %s' % pf.path)
                os.remove(pf.path)


if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('reffile', metavar='reference_input', 
                        help='file or folder containing images.')
    parser.add_argument('targetfile', metavar='target_input', 
                        help='folder containing images that will be removed.')
    parser.add_argument('-m', '--move', action='store_true', 
                        help='Move files instead of deleting them')
    parser.add_argument('-o', '--output', default=None, 
                        help='Output folder to move deleted files')
    args = parser.parse_args()
    
    main(args.reffile, args.targetfile, args.move, args.output)
