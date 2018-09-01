#!/usr/bin/python
#-*- coding: utf-8 -*-
import argparse
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('imgfile', metavar='image', help='Path to the image')
    argparser.add_argument('x', metavar='x_value', help='Pixel top left in X', type=int)
    argparser.add_argument('y', metavar='y_value', help='Pixel top left in Y', type=int)
    argparser.add_argument('width', metavar='width_value', help='Width of the box', type=int)
    argparser.add_argument('height', metavar='height_value', help='Height of the box', type=int)
    args = argparser.parse_args()

    img = args.imgfile
    x = args.x
    y = args.y
    w = args.width
    h = args.height
    
    im = np.array(Image.open(img), dtype=np.uint8)

    fig, ax = plt.subplots(1)
    ax.imshow(im)

    rect = patches.Rectangle((x,y),w,h,linewidth=1,edgecolor='r',facecolor='none')
    ax.add_patch(rect)
    plt.show()


