#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
This script extracts the labels and annotations from .json files.

CSV file in the output has the form:
'filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'
"""
import sys
sys.path.insert(0, '..')
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse

import json

from os.path import realpath, dirname, splitext, basename, join

def check_namefile(index):
    """ Images contain the name starting as 00001.jpg """
    name = '{0:05}'.format(index)
    return name+'.jpg'
    

def main(jsonfile, output):
    """
    json file from a dataset annotated with VoTT
        {'scd', 'framerate', 'visitedFrames', 'inputTags', 
         'frames', 'tag_colors', 'suggestiontype'}
        
    """
    with open(jsonfile) as f:
        data = json.load(f)

    dic = {}
    for id in data['frames']:
        #'filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'
        filename = join(HOME, id+'.jpg')
        x1 = data['frames'][id]['x1']
        x2 = data['frames'][id]['x2']
        y1 = data['frames'][id]['y1']
        y2 = data['frames'][id]['y2']
        w = data['frames'][id]['width']
        h = data['frames'][id]['height']
        dic[int(id)] = {}
        print id
    #print data['frames']['1']
    #frames = data['frames']    

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
    """


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('jsonfile', metavar='file_json', help='File (.json) containing object annotations')
    argparser.add_argument('-o', '--output', help='File to save annotations', default=None)
    args = argparser.parse_args()
    main(args.jsonfile, args.output)
    

