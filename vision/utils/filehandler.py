#!/usr/bin/python
#-*- coding: utf-8 -*-

import sys
import linecache
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)

import ast
import numpy as np
from os.path import realpath, dirname, splitext
from os.path import basename, isfile, isdir, join

import progressbar

def is_file(inputfile):
    """ Check whether the ``inputfile`` corresponds to a file """
    inputfile = realpath(inputfile)
    if not isfile(inputfile):
        logger.error('Input is not a file!')
        sys.exit(0)
    return inputfile


def is_folder(inputfolder):
    """ Check whether the ``inputfolder`` corresponds to a folder """
    inputfolder = realpath(inputfolder)
    if not isdir(inputfolder):
        logger.error('Argument %s is not a folder!' % inputfolder)
        sys.exit(0)
    return inputfolder


def filename(path, extension=True):
    fname, ext = splitext(basename(path))
    if extension:
        return fname, ext
    return fname


def add2dic(dic, key, value):
    if dic.has_key(key):
        dic[key].append(value)
    else:
        dic[key] = [value]
    return dic


class PathfileHandler(object):
    """Class to deal with files containing paths and labels/features"""

    def __init__(self, inputfile, display=True, load=False):
        """Initializes the class
        
        Parameters
        ----------
        inputfile : string
            path to the file containing images and labels/features
        display : boolean
            show the progress bar
        load : boolean
            load path, labels and feats vectors in case they exist
        """
        self.display = display
        self.inputfile = realpath(inputfile)
        self.inputdir = dirname(inputfile)
        self.nb_lines = self.count_lines(inputfile)
        self.path = None
        self.label = None
        self.feats = None

        if load:
            self.vpaths, self.vlabels, self.vfeats = load_file(inputfile)


    def __iter__(self):
        """Iterates the file yielding the path, the true label and a vector of 
        features when exists
        """
        pb = progressbar.ProgressBar(self.nb_lines)
        with open(self.inputfile) as fin:
            for line in fin:
                arr = line.strip().split()
                if len(arr) == 1:
                    self.path = arr
                    yield self.path
                elif len(arr) > 1:
                    self.path = arr[0]
                    self.label = arr[1]
                    if len(arr) > 2:
                        if len(arr[2:]) == 1:
                            self.feats = ast.literal_eval(arr[2])
                        else:
                            self.feats = map(float, arr[2:])
                        yield self.path, self.label, self.feats
                    else:
                        yield self.path, self.label
                if self.display:
                    pb.update()


    @staticmethod
    def load_file(inputfile):
        """ Load the content of pathfile into `path`, `label` and `feats`
        """
        logger.debug('Loading file: %s' % inputfile)
        vpaths, vlabels, vfeats = [], [], []
        with open(inputfile) as fin:
            for line in fin:
                arr = line.strip().split()
                if len(arr) == 2:
                    vpaths.append(arr[0])
                    vlabels.append(int(arr[1]))
                elif len(arr) == 3:
                    vpaths.append(arr[0])
                    vlabels.append(int(arr[1]))
                    vfeats.append(int(arr[2]))
                elif len(arr) > 3:
                    vpaths.append(arr[0])
                    vlabels.append(int(arr[1]))
                    vfeats.append(map(float, arr[2:]))
        return vpaths, vlabels, vfeats


    @staticmethod
    def count_lines(inputfile):
        """ Returns the total number of images in the input file 

        Parameters
        ----------
        inputfile : string
            path to the file containing images and labels/features

        Returns
        -------
        n : int
            total number of lines in the document
        """
        with open(inputfile) as fin:
            for n, _ in enumerate(fin, start=1): pass
        return n


    def get_line(self, nb_line):
        """Returns the line at number `nb_line`"""
        return linecache.getline(self.inputfile, nb_line).strip()


    def get_path(self, nb_line, label=False):
        """Returns the path of the line at number `nb_line`"""
        line = linecache.getline(self.inputfile, nb_line)
        path = None
        if line:
            arr = line.split()
            path = arr[0]
            if label:
                y = arr[1]
                return path, y
        return path


    def get_label(self, nb_line):
        """Returns the path of the line at number `nb_line`"""
        line = linecache.getline(self.inputfile, nb_line)
        label = None
        if line:
            label = line.split()[1]
        return label


    def features_line(self, nb_line, asstr=False):
        """Returns only the feature of the line number `nb_line`"""
        line = linecache.getline(self.inputfile, nb_line)
        arr = line.strip().split()
        if len(arr) > 2:
            if asstr:
                return ' '.join(arr[2:])
            else:
                return arr[2:]
        return None


    def current_features(self, N=2):
        """
        Return a list containing the `N` features with the highest scores

        Parameters:
        -----------
        N : int
            number of features to return

        Notes:
        ------
        The output has the form of a list as:
        [(1, 0.33), (3, 0.12), (5, 0.08), ...]
        """
        features = self.labels[1:]
        classes = map(int, features[0::2])
        preds = np.array(features[1::2], dtype=np.float32)
        topN = []
        for n in range(N):
            valmax = preds.max()
            imax = preds.argmax()
            topN.append((classes[imax], valmax))
            preds[imax] = -1
        return topN


    def has_features(self):
        """
        Returns
        -------
        _ : bool
            False : only path and label
            True : path, label and features
        """
        line = self.get_line(1)
        arr = line.strip().split()
        if len(arr) > 2:
            return True
        else:
            return False
#End of class PathfileHandler
