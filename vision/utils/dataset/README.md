# Scripts for Dataset

This folder contains scripts to clean and fix the dataset recorded with ZED Camera. ZED Camera sometimes record images with errors like green-pink images (image 1), red-blue images (image 2) and lens problem (image 3) as the illustrated in the images below.

<img src="../images/error1.jpg" width="50%" align="center" />
<img src="../images/error2.jpg" width="50%" align="center" />
<img src="../images/error3.jpg" width="50%" align="center" />

Thus, this folder contains scripts to remove such files and rename all files to keep them in order. When extracting files from a bag, we recommend to run scripts in the following order:

- Bag Extractor: This script extracts the content of a bag file saving it in three folders (`left_image`, `right_image`, and `depth_image`), as well as a `.csv` file containing the name of each image with its respective data (IMU, GPS, throttle, etc.). To extract the content of the bag, run:

```
$ python bag_extractor.py <path_to_bag_file>
```

- Check Errors: This script verify the errors above in images. As the same error occurs in the three images (left, right and depth) at the same time, we can only verify in a single folder and them remove the content of other folders. Here we consider always the `left_image` folder. We select an `error_folder` where we will copy the files that contain error to the folder to manually analyze each image. After running the command, we can check the `error_folder` and delete the files that do not contain error, keeping in the folder only images that contain errors.

```
$ python check_error.py <left_image_folder> <error_folder>
```

-  Move Error Files: This script moves all files that are stored in a file to a selected folder. Having selected in `error_folder` all files that contain error, we first generate a file (`error.txt`) containing the path of these files (referenced to the `left_image` folder using `-p` argument) to remove from `left_image`. Thus, we run:

```
$ python create_pathfile.py -p <left_image> -o error.txt <error_folder> 
$ rm <error_folder>/*
$ python move_error_files.py error.txt <error_folder>
```

After moving all files that were automatically separated due to errors, it is necessary to verify manually the `left_image` folder, moving all files that contain error to the `<error_folder>`. It may be the case that we have lots of frames (mainly in the begin or in the end of the sequence) that are not interesting (sometimes putting or removing the boat from the water). Thus, it is interesting to remove a sequence of consecutives frames. What leads us to the next script.

- Remove Frames: This script remove a sequence of frames delimited by `-s` and `-e` indices. Files are removed using the paths decribe in the file with all paths of the `left_image` folder. We run:

```
$ python create_pathfile.py <left_image> -o <path_file> 
$ python remove_frames.py -s <start_frame> -e <end_frame> <path_file>
```

- Remove Right and Depth: This script removes all images containing error from the right and depth folders. At this point, we have all images that contain error removed from `left_image` folder. Now, we have to remove the same images from `right_image` and `depth_image` folders. To do this, we use the file containing all images from the `left_image` folder (`path_file`) and run:

```
$ python remove_right_depth.py <path_file> <right_image>
$ python remove_right_depth.py <path_file> <depth_image>
```

After removing all files that contain errors or are discarded, we can fix the paths for files in `topics_original.csv` and generate a new file named `rename.txt` containing the old paths and the new paths for files to be renamed. In order to generate them, we can run:

```
$ python fix_paths.py <path_file> <topics_original_csv> 
```

Now that we have the `rename.txt` file, we can apply the new names for all folders containing images, and thus converting all names to a sorted sequence of images starting at `0.jpg`. This is performed using the commands:

```
$ python rename_files.py <rename_txt> <left_image>
$ python rename_files.py <rename_txt> <right_image>
$ python rename_files.py <rename_txt> <depth_image>
```

Having performed all these commands, the dataset is clean and ready to be annotated.
