#/usr/bin/env python

"""

"""

import os
import argparse
import sys
from os.path import exists, dirname, join

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import filehandler as fh



def main(rootfolder):
    rootfolder = fh.is_folder(rootfolder)
    old_right = join(rootfolder, 'right_image')
    old_depth = join(rootfolder, 'depth_image')
    # folder containing subfolders with selected images
    left = join(rootfolder, 'LEFT')
    right = join(rootfolder, 'RIGHT')
    depth  join(rootfolder, 'DEPTH')
    
    for root, dirs, files in os.walk(".", topdown=False):
        for name in files:
            
            print(os.path.join(root, name))

   


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('rootfolder', metavar='root_folder', help='Folder containing left, right and depth folders')
    args = parser.parse_args()
    main(args.rootfolder)
    
