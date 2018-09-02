#!/usr/bin/python
#-*- coding: utf-8 -*-
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

import os
import time
from os.path import join, splitext, basename, dirname
from os.path import realpath, abspath, exists
from Tkinter import Tk, Label, Listbox, END, N, S, W, Canvas, BOTH
from PIL import Image, ImageTk, ImageDraw

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
        'filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'
        """
        self.imgdata = []
        with open(self.input) as fin:
            for i, line in enumerate(fin):
                if i == 0:
                    header = line.strip()
                    continue
                arr = line.strip().split(',')
                path = arr[0]
                x = int(arr[4])
                y = int(arr[5])
                width = int(arr[1])
                height = int(arr[2])
                self.imgdata.append((path, x, y, width, height))
                self._check_size(path)
        return self.imgdata


    def nextImage(self):
        """
        Return the path, true label and predicted label of the next image 
        in the list of images.
        """
        path, x, y, width, height = self.imgdata[self.index]
        if self.index < len(self.imgdata)-1:
            self.index += 1
        fname = basename(path)
        return fname, path, x, y, width, height
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
        if not self.im:
            self.im = Image.open(pathimg)
        self.tkimage = ImageTk.PhotoImage(self.im)
        self.image.configure(image=self.tkimage)
        time.sleep(0.03)


    def updateLabelFrame(self, text):
        """
        Update the label containing the number of the frame
        """
        self.frame.configure(text='Frame: '+text)


    def drawBox(self, pathimg, x, y, width, height):
        self.im = Image.open(pathimg)
        draw = ImageDraw.Draw(self.im)
        x1 = x + width
        y1 = y + height
        #draw.rectangle(((x, y), (x1, y1)), outline=128)
        # rectangle has only 1px lines
        # use lines with width=3 to better drawing
        line = (x, y, x, y1)
        draw.line(line, fill="red", width=3)
        line = (x, y, x1, y)
        draw.line(line, fill="red", width=3)
        line = (x, y1, x1, y1)
        draw.line(line, fill="red", width=3)
        line = (x1, y, x1, y1)
        draw.line(line, fill="red", width=3)


    def update_window(self):
        """
        Update the window and its elements every second
        """
        name, path, x, y, width, height = self.imgs.nextImage()
        self.drawBox(path, x, y, width, height)
        self.updateImage(path)
        self.updateLabelFrame(name)
        self.after(1, self.update_window)
#End of class DemoWindow


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('csv_file', metavar='file_images', help='Path to the CSV file')
    args = argparser.parse_args()
    
    window = DemoWindow(args.csv_file)
    window.mainloop()




