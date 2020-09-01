""" Script to rename files in a folder. It uses `.zfill(<number>)` to add zeros before the
    name of the file, where <number> is the max size of the name of a file. E.g., the file
    1.jpg has size 1, 25.jpg has size 2, 264.jpg has size 3, and so on. Thus, consider a 
    folder containing 800 images, the max size of a file would be 3. When renaming the file
    the first files will contain zeros before the name of the file as 001.jpg, 002.jpg and
    so on.
"""
import argparse
import shutil
from os.path import splitext, join
import os


def rename_files(folder_input, folder_output):
    """ Rename files from folder input and save the renamed files in folder output """
    if not folder_output:
        folder_output = folder_input

    path_files = []
    max_size = 0
    for root, _, files in os.walk(folder_input):
        for img_path in files:
            id, ext = splitext(img_path)
            if max_size < len(id):
                max_size = len(id)
            path_files.append(id)

    for id_file in sorted(path_files):
        old_path = join(folder_input, id_file+'.jpg')
        new_path = join(folder_output, id_file.zfill(max_size)+'.jpg')
        if folder_input != folder_output:
            shutil.copy(old_path, new_path)
        else:
            os.rename(old_path, new_path)
    print('Renamed files in folder: {}'.format(folder_output))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfolder', metavar='folder_input', help='Folder containing images.')
    parser.add_argument('-o', '--folder_output', help='Folder to save new renamed images', default=None)
    args = parser.parse_args()

    rename_files(args.inputfolder, args.folder_output)
