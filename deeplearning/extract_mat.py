#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
This script extracts the labels and annotations from .mat files
"""
import sys
sys.path.insert(0, '..')
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse

import scipy.io as sio
from os.path import realpath, dirname, splitext, basename, join

def check_namefile(index):
    """ Images contain the name starting as 00001.jpg """
    name = '{0:05}'.format(index)
    return name+'.jpg'
    

def main(matfile, output):
    """
    matfile from MODD dataset
        'smallobjects', '__header__', 'masks', '__globals__',
        'largeobjects', 'horizonts', '__version__'
    """

    matfile = realpath(matfile)
    pathimg = join(dirname(matfile), 'images')
    if output:
        output = realpath(output)
    else:
        fname, _ = splitext(basename(matfile))
        output = join(dirname(matfile), fname+'.csv')
        fout = open(output, 'w')
        header = 'filename,width,height,class,xmin,ymin,xmax,ymax\n'
        fout.write(header)

    dmat = sio.loadmat(matfile)
    small = dmat['smallobjects']
    large = dmat['largeobjects']
    
    index = 1
    for smob, laob in zip(small, large):
        img_name = join(pathimg, check_namefile(index))

        smob = smob[0]
        if smob.size:
            #smob = map(int, smob)
            _x1, _y1, _x2, _y2 = smob
            for xmin, ymin, xmax, ymax in zip(_x1, _y1, _x2, _y2):
                width = xmax - xmin
                height = ymax - ymin
                fout.write('%s,%d,%d,obstacle,%d,%d,%d,%d\n' % 
                           (img_name, width, height, xmin, ymin, xmax, ymax))

        laob = laob[0]
        if laob.size:
            _x1, _y1, _x2, _y2 = laob
            for xmin, ymin, xmax, ymax in zip(_x1, _y1, _x2, _y2):
                width = xmax - xmin
                height = ymax - ymin
                fout.write('%s,%d,%d,obstacle,%d,%d,%d,%d\n' % 
                           (img_name, width, height, xmin, ymin, xmax, ymax))
        
        index += 1



if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('matfile', metavar='file_mat', help='File (.mat) containing object annotations')
    argparser.add_argument('-o', '--output', help='File to save annotations', default=None)
    args = argparser.parse_args()
    main(args.matfile, args.output)
    

