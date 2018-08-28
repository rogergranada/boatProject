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
from os.path import realpath

def main(matfile, output):
    matfile = realpath(matfile)
    output = realpath(output)

    


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('matfile', metavar='file_mat', help='File (.mat) containing object annotations')
    argparser.add_argument('-o', '--output', help='File to save annotations', default=None)
    args = argparser.parse_args()
    main(args.matfile, args.output)
    

