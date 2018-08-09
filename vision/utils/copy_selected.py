#/usr/bin/env python

"""
Before executing this scripts, make sure that all images that will not belong to the
dataset are already excluded and each folder from 1 to N has a sequence of frames
to the LEFT camera
"""

import os
import argparse
import sys
from os.path import exists, dirname, join

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import shutil

import filehandler as fh

def move_files_new_path(rootfolder):
    """
    After selecting images in `left_image` folder, move the selected from 
    `right_image` and `depth_image` following the same structure as `LEFT`
    folder
    """
    rootfolder = fh.is_folder(rootfolder)
    old_right = join(rootfolder, 'right_image')
    old_depth = join(rootfolder, 'depth_image')
    # folder containing subfolders with selected images
    left =  join(rootfolder, 'LEFT')
    right = join(rootfolder, 'RIGHT')
    depth = join(rootfolder, 'DEPTH')
    
    for root, dirs, files in os.walk(left, topdown=False):
        # right
        righttmp = root.replace(left, right)
        dirright = fh.mkdir_from_file(righttmp)
        # depth
        depthtmp = root.replace(left, depth)
        dirdepth = fh.mkdir_from_file(depthtmp)
        
        logger.info('Moving files following the folder: %s' % root)
        for name in files:
            leftfile = join(root, name)
            old_rightfile = join(old_right, name)
            new_rightfile = leftfile.replace(left, right)
            shutil.move(old_rightfile, new_rightfile)

            old_depthfile = join(old_depth, name)
            new_depthfile = leftfile.replace(left, depth)
            shutil.move(old_depthfile, new_depthfile)


def generate_pathfile(rootfolder):
    rootfolder = fh.is_folder(rootfolder)
    left  = join(rootfolder, 'LEFT')
    right = join(rootfolder, 'RIGHT')
    depth = join(rootfolder, 'DEPTH')

    leftfile  = open(join(rootfolder, 'original_left.txt'),  'w')
    rightfile = open(join(rootfolder, 'original_right.txt'), 'w')
    depthfile = open(join(rootfolder, 'original_depth.txt'), 'w')
    
    logger.info('Saving paths from %s' % left)
    for root, dirs, files in os.walk(left, topdown=False):
        files = fh.sort_by_name(files)
        for name in files:
            path = join(root, name)
            leftfile.write('%s\n' % path)
    leftfile.close()

    logger.info('Saving paths from %s' % right)
    for root, dirs, files in os.walk(right, topdown=False):
        files = fh.sort_by_name(files)
        for name in files:
            path = join(root, name)
            rightfile.write('%s\n' % path)
    rightfile.close()

    logger.info('Saving paths from %s' % depth)
    for root, dirs, files in os.walk(depth, topdown=False):
        files = fh.sort_by_name(files)
        for name in files:
            path = join(root, name)
            depthfile.write('%s\n' % path)
    depthfile.close()


def rename_files(rootfolder):
    rootfolder = fh.is_folder(rootfolder)
    folders = ['LEFT', 'RIGHT', 'DEPTH']

    for fld in folders:
        pathfolder = join(rootfolder, fld)
        fout = open(join(rootfolder, 'paths_'+fld.lower()+'.txt'),  'w')
    
        logger.info('Renaming paths from %s' % pathfolder)
        for root, dirs, files in os.walk(pathfolder, topdown=False):
            files = fh.sort_by_name(files)
            index = 1
            for name in files:
                old_path = join(root, name)
                new_path = join(root, str(index)+'.jpg')
                os.rename(old_path, new_path)
                fout.write('%s\n' % new_path)
                index += 1
        fout.close()


def main(rootfolder):
    # Sequence 1: Move files to RIGHT and DEPTH
    #move_files_new_path(rootfolder)

    # Sequence 2: Generate the pathfile to the original files
    #generate_pathfile(rootfolder)

    # Sequence 3: Rename files starting in 1 to n in all folders
    rename_files(rootfolder)
    pass



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('rootfolder', metavar='root_folder', help='Folder containing left, right and depth folders')
    args = parser.parse_args()
    main(args.rootfolder)
    
