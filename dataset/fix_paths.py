#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
Fix paths for files after removing all images that contain errors or are
not part of the dataset (boat is not on the water). This script loads the
`paths.txt` and `topics_original.csv` file and renames all paths in the 
path file. It generates as output a new file for `topics.csv` containing 
only topics for images that belong to the dataset, and a file `rename.txt` 
containing the previous path and the new path for all files of the dataset.

Rename.txt has the format:
old_path new_path\n

"""
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse
from os.path import join, dirname

import filehandler as fh

def main(inputfile, topics, dirout=None):
    """
    Exclude rows from topic file that do not exist in the final list of images.
    Rename files fixing their names (restart the counter from 0).       
    """
    if not dirout:
        dirout = dirname(inputfile)

    pfi = fh.PathfileHandler(inputfile)
    dnames = {}
    for _ in pfi:
        dnames[fh.filename(pfi.path, string=True)] = ''

    count = 0
    features = ''
    with open(join(dirout, 'rename.txt'), 'w') as ftxt, \
         open(join(dirout, 'topics.csv'), 'w') as ftpc:
        pft = fh.PathfileHandler(topics, sep=';', display=False)
        for arr in pft:
            if isinstance(arr, str):
                path = arr
            else:
                path = arr[0]
            if dnames.has_key(path):
                new_path = '%d.jpg' % count 
                ftxt.write('%s %s\n' % (path, new_path))
                if len(arr) == 2 and arr[1]:
                    features = ';'.join(arr[1])
                ftpc.write('%s;%s\n' % (new_path, features))
                count += 1
    

if __name__== "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filepaths', metavar='file_kept', 
                        help='file containing path for images that are kept.')
    parser.add_argument('topics', metavar='file_topics', 
                        help='file containing all data from bag.')
    parser.add_argument('-o', '--output', default=None, 
                        help='Output folder to save new files')
    args = parser.parse_args()
    
    main(args.filepaths, args.topics, args.output)
