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
    
HOME='/home/roger/Downloads/05/'
def main(jsonfile, output):
    """
    json file from a dataset annotated with VoTT
        {'scd', 'framerate', 'visitedFrames', 'inputTags', 
         'frames', 'tag_colors', 'suggestiontype'}
        
    """
    if not output:
        fname, ext = splitext(basename(jsonfile))
        output = join(dirname(jsonfile), fname+'_annot.csv')
    with open(jsonfile) as f:
        data = json.load(f)
    
    #print len(data['frames']['0'])

    dic = {}
    for id in sorted(data['frames']):
        #'filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'
        idframe = int(id)+1
        filename = join(HOME, str(idframe)+'.jpg')
        for dobj in data['frames'][id]:
            x1 = int(dobj['x1'])
            x2 = int(dobj['x2'])
            _y1 = int(dobj['y1'])
            y2 = int(dobj['y2'])
            w = int(dobj['width'])
            h = int(dobj['height'])
            _id = int(dobj['id'])

            width = x2 - x1
            height = y2 - _y1
            y1 = h - _y1 - height
            dic[(int(id), _id)] = {'x1': x1, 'x2': x2, 'y1': y1, 'y2': y2, 'width': width, 'height': height, 'filename': filename}

    with open(output, 'w') as fout:
        header = 'filename,width,height,class,xmin,ymin,xmax,ymax\n'
        fout.write(header)
        for id in sorted(dic):
            fout.write('%s,%d,%d,obstacle,%d,%d,%d,%d\n' % (dic[id]['filename'],
                                                          dic[id]['width'],
                                                          dic[id]['height'],
                                                          dic[id]['x1'],
                                                          dic[id]['y1'],
                                                          dic[id]['x2'],
                                                          dic[id]['y2']
                                                         ))

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
    

