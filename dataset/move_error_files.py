#/usr/bin/env python		

"""		
From a file containing paths of files, move them to a new folder.		
"""		
import argparse		
import logging		
logger = logging.getLogger(__name__)		
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)	

import utils	

if __name__ == "__main__":		
    parser = argparse.ArgumentParser()		
    parser.add_argument('filepaths', metavar='path_file', help='File containing paths to images')		
    parser.add_argument('movefolder', metavar='move_folder', help='Folder where images will be moved')		
    parser.add_argument('-c', '--copy', action='store_true', default=False, help='Copy files instead of move them' )		
    args = parser.parse_args()		
    utils.move_files(args.filepaths, args.movefolder, args.copy)
