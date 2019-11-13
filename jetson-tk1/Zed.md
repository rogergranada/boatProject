# Stereolabs ZED Camera

The [ZED](https://www.stereolabs.com/documentation/overview/getting-started/introduction.html) is a camera that reproduces the way human vision works. Using its two "eyes" and through triangulation, the ZED provides a three-dimensional understanding of the scene it observes, allowing your application to become space and motion aware. ZED perceives the world in three dimensions by using dual lenses. Using binocular vision and high-resolution sensors, the camera can tell how far objects are around you from 0.5 to 20m at 100FPS, indoors and outdoors. It captures high-definition 3D video with a wide field of view and outputs two synchronized left and right video streams in side-by-side format on USB 3.0. 

!['ZED'](https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/images/zed.jpg)

The [ZED camera](http://www.jetsonhacks.com/2016/02/03/stereolabs-zed-camera/) is a stereoscopic imaging camera which contains two high definition imagers. One imager is mounted on the left and the other on the right side of the camera enclosure. The camera provides a video stream, each frame of which consists of a composite of an image from each camera, side by side. The images are time synchronized. The video stream is sent over USB 3.0 to a host. On the host, the frame in the stream is then converted to a depth map using the host GPU. The Stereolabs SDK on the host uses the geometry of the fixed distance between the imaging elements, and using the known field of view of the imagers calculates an accurate depth map. The ZED can sense depth between 1 and 20 meters.

## Installing ZED SDK

To install the ZED SDK in Jetson TK1, we need to access the *Archive* section in *Developer* menu of the Stereolabs web site, and download the **ZED SDK 1.2** file. Although ZED SDK has newer versions, they are not compatible with Jetson TK1 board or with Ubuntu 14.04 installed in Jetson TK1. To download the ZED SDK directly without the need of navigate in the site and install it in the Jetson, run:

```
$ wget https://www.stereolabs.com/developers/downloads/archives/ZED_SDK_Linux_JTK1_v1.2.0.run
$ chmod +x ZED_SDK_Linux_JTK1_v1.2.0.run
$ ./ZED_SDK_Linux_JTK1_v1.2.0.run
```

Running these commands will start to install the SDK. After finishing the installation process, reboot the Jetson TK1 board to apply the modifications.


## Creating a package to access ZED with ROS

We create a ROS node named ``zedpub`` to publish the name of the image being saved by ZED Camera as a ROS topics. The entire folder containing the ROS package can be found in the [Github page](https://github.com/rogergranada/lutra-usv-lsa/blob/master/jetson-tk1/zedpub.zip?raw=true). Download and compile the ``zedpub`` package with:

```
# go to the source folder of the catkin_ws workspace
$ cd ~/catkin_ws/src/

# Download the ZED package and decompress in the source folder
$ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/rogergranada/lutra-usv-lsa/master/jetson-tk1/zedpub.zip
$ unzip zedpub.zip && rm zedpub.zip

# compile the workspace
$ cd ..
$ catkin_make
```

After compiling the catkin workspace, you can test the new node by running ``roscore`` in a terminal and running ``roslaunch`` in another terminal as:

```
# in Terminal 1, run the ROS core
$ roscore

# in Terminal 2, run the ROS launch
$ roslaunch zedpub camera.launch

# in Terminal 3, you can check the published topics
$ rostopic list
``` 

You should then see the following topics in the list:

```
/camera
```
