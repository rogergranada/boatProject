#!/usr/bin/python
#-*- coding: utf-8 -*-

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse
import sys
from os import walk
from os.path import join, isdir, dirname, basename, splitext
from xml.dom import minidom

import show_box

def main(input_folder):
    for root, dirs, files in walk(input_folder):
        for name in sorted(files):
            xmlfile = minidom.parse(join(root, name))
            if not xmlfile.getElementsByTagName('object'):
                continue
            itemlist = xmlfile.getElementsByTagName('filename')
            filename = itemlist[0].childNodes[0].data
            img = join('/home/roger/Downloads/Modify_Annotations/images', filename+'.jpg')

            bounding_boxes = []
            itemlist = xmlfile.getElementsByTagName('object')
            for obj in itemlist:
                # <DOM Element: object at 0x7f6446791b90> :: <object>
                for elem in obj.childNodes:
                    if elem.hasChildNodes():
                        if elem.tagName == 'bndbox':
                            for item in elem.childNodes:
                                if item.hasChildNodes():
                                    if item.tagName == 'xmin': xmin = int(item.childNodes[0].data)
                                    if item.tagName == 'ymin': ymin = int(item.childNodes[0].data)
                                    if item.tagName == 'xmax': xmax = int(item.childNodes[0].data)
                                    if item.tagName == 'ymax': ymax = int(item.childNodes[0].data)
                w = xmax - xmin
                h = ymax - ymin
                bounding_boxes.append([xmin, ymin, w, h])
            show_box.playBox(img, bounding_boxes)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfolder', metavar='file_ground', help='File containing ground truth for all images')
    args = parser.parse_args()

    main(args.inputfolder)
