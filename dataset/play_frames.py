#!/usr/bin/python
#-*- coding: utf-8 -*-

import os
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists, isdir, isfile

import argparse
# Python 2
#from Tkinter import Tk, Label, Listbox, END, N, S, W
# Python 3
from tkinter import Tk, Label, Listbox, END, N, S, W
from PIL import Image, ImageTk
import time


class ImageManager(object):
    """
    Class to manage the frames
    """
    def __init__(self, input):
        """
        Initiates the class ImageManager
    
        Parameters:
        -----------
        input : string
            path to the input file containing the paths to the images
            as well as the true values and the predicted values
        """
        self.input = realpath(input)
        self.imgdata = []
        self.index = 0
        self.width = 0
        self.height = 0
        self._loadImages()


    def __iter__(self):
        """
        Iterates over the images yielding the path of the image,
        the name of the image, the true label and the predicted label.
        """
        for path, name in self.imgdata:
            yield path, name

    
    def _check_size(self, pathimg):
        """Check the size of an image"""
        im = Image.open(pathimg)
        self.width, self.height = im.size


    def _loadImages(self):
        """
        Extract the content from the file and stores into an array.
        """
        self.imgdata = []
        with open(self.input) as fin:
            for line in fin:
                path = line.strip()
                name, ext = splitext(basename(path))
                self.imgdata.append((path, name))
                self._check_size(path)
        return self.imgdata


    def nextImage(self):
        """
        Return the path, true label and predicted label of the next image 
        in the list of images.
        """
        data = self.imgdata[self.index]
        if self.index < len(self.imgdata)-1:
            self.index += 1
        #time.sleep(1)
        return data
#End of class ImageManager


class DemoWindow(Tk):
    """
    Class to manage the window of the demo
    """
    def __init__(self, fileinput):
        """
        Build the visual interface with images and fields to the images data
        """
        fileinput = realpath(fileinput)
        self.imgs = ImageManager(fileinput)
        Tk.__init__(self)
        self.title("Boat frames")
        # width x height + x_offset + y_offset:
        self.geometry(str(self.imgs.width+20)+"x"+str(self.imgs.height+30)+"+1+1")
        self.i = 0
        self.prev = 0

        self.filelist = self.imgs._loadImages()

        self.frame = Label(self, text="")
        self.frame.grid(row=0, column=1, padx=10, pady=2, sticky=N+S+W)

        self.image = Label(self, image=None)
        self.image.grid(row=1, column=1, padx=10, pady=2, sticky=N+S+W)

        self.update_window()


    def updateImage(self, pathimg):
        """
        Updata the Label containing the image
        """
        im = Image.open(pathimg)
        self.tkimage = ImageTk.PhotoImage(im)
        self.image.configure(image=self.tkimage)


    def updateLabelFrame(self, text):
        """
        Update the label containing the number of the frame
        """
        self.frame.configure(text='Frame: '+text)


    def update_window(self):
        """
        Update the window and its elements every second
        """
        path, name = self.imgs.nextImage()
        self.updateImage(path)
        self.updateLabelFrame(name)
        self.after(1, self.update_window)
#End of class DemoWindow


def create_file_paths(inputfolder, fileoutput):
    fout = open(fileoutput, 'w')

    path = realpath(inputfolder)
    files = os.listdir(inputfolder)
    names = []
    for img in files:
        name, ext = splitext(img)
        if ext == '.jpg':
            names.append(int(name))
    for name in sorted(names):
        fname = str(name)+'.jpg'
        fout.write('%s\n' % join(inputfolder, fname))
    print('Saved paths in file: %s' % fileoutput)


if __name__== "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('inputfile', metavar='file_input', 
                        help='file or folder containing images.')
    args = parser.parse_args()
    
    input = args.inputfile
    if isdir(input):
        input = input+'/'
        dirin = dirname(realpath(input))
        outputfile = join(dirin, 'paths.txt')
        create_file_paths(input, outputfile)
    elif isfile(input):
        outputfile = input    
    
    window = DemoWindow(outputfile)
    window.mainloop()
    
