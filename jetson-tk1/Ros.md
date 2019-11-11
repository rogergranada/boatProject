# Robot Operating System (ROS) on Jetson TK1

[Robot Operating System](http://wiki.ros.org/ROS/Introduction) (ROS) is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. The ROS runtime "graph" is a peer-to-peer network of processes (potentially distributed across machines) that are loosely coupled using the ROS communication infrastructure. ROS implements several different styles of communication, including synchronous RPC-style communication over services, asynchronous streaming of data over topics, and storage of data on a Parameter Server. 

The primary goal of ROS is to support code reuse in robotics research and development so you can find a built-in package system. It is interesting to note that although ROS contains Operation System in the name, keep in mind that ROS is not an OS, a library, or an RTOS, but a framework using the concept of an OS. A good introduction is given in the freely available book named [A Gentle Introduction to ROS ](https://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf) by Jason O'Kane. The [ROS Wiki](http://wiki.ros.org/ROS/Tutorials) also contains lots of tutorials to introduce you to its main concepts.


## Installing ROS in Jetson TK1

JetsonPack for Jetson TK1 is based on Ubuntu 14.04 and thus, the ROS version to install is named *Indigo*. In order to install the ROS package, you can run the following commands:

```
# Add ros repository to sources.list
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# Install ROS and Point Cloud Library
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop
$ sudo apt-get install ros-indigo-pcl-conversions

$ echo "" >> ~/.bashrc
$ echo "# Load ROS environment" >> ~/.bashrc
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

$ source ~/.bashrc
```

These commands install ROS Indigo Desktop and the Point Cloud Library.


## Creating Catkin workspace to run ROS

Catkin packages can be built as a standalone project, in the same way that normal cmake projects can be built, but catkin also provides the concept of workspaces, where you can build multiple, interdependent packages together all at once. A [Catkin workspace](http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces) is a folder where you modify, build, and install catkin packages. In order to create a Catkin workspace, you should run the following commands:

```
# create a folder for Catkin Workspace
$ cd ~
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace

# compile the environment
$ cd ~/catkin_ws
$ catkin_make

$ echo "" >> ~/.bashrc
$ echo "# Load Catkin Workspace" >> ~/.bashrc
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
 
These commands create a structure in the home folder with the root workspace set in ``/home/ubuntu/catkin_ws``. 

## Testing ROS installation

To check if the ROS is installed correctly, run:

```
$ roscore
```

And see it starts running correctly. In case of problem, you can check the log files by running:

```
$ roscd log
```
