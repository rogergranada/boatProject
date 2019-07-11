import cv2
import os
import argparse
from os.path import join, basename, dirname, splitext, realpath

def main(inputfile, outputfile):
    if not outputfile:
        outputfolder = join(dirname(inputfile), 'video_frames/')
        os.mkdir(outputfolder)

    print('Loading video: {}'.format(basename(inputfile)))
    video = cv2.VideoCapture(inputfile)
    success, image = video.read()
    i = 0
    success = True
    while success:
        cv2.imwrite("%s" % join(outputfolder, str(i)+'.jpg'), image)
        success, image = video.read()
        i += 1
    print('Extracted {} frames.'.format(i))


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('inputfile', metavar='input_file', help='File containing paths to images')
    argparser.add_argument('-o', '--output', help='Path to the output file', default=None)
    args = argparser.parse_args()
    main(args.inputfile, args.output)
