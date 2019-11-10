# Cloning and Restoring Image

In many cases there is a large amount of work that goes into creating a working Jetson development system. This can include such things as loading drivers, compiling source files and libraries, system configuration, and so on. During development, sometimes things can go south and leave the system in a non-working state. Wouldn't it be great if you could take a snapshot of a baseline system for just such occasions? Well, you can! This is known as "[cloning](http://www.jetsonhacks.com/2015/08/26/clone-image-nvidia-jetson-tk1/)" the system. In this section we describe how to save and restore the system image from a Jetson to a file in the local host. The host must be the same that contains the JetPack installer that flashed the Jetson. 

## Cloning the system image to localhost 

In order to clone the image to the localhost, first you navigate to the proper directory on the localhost to begin the process. Open a Terminal and go to the JetPack install directory (e.g., ``/home/roger/JetPack/``). The correct folder has the following content:

```
GameWorksOpenGLSamples
JetPack-L4T-3.0-linux-x64.run
JetPack_Uninstaller
TK1
_installer
host-x64-linux-public-3.7.224-e982b7b
jetpack_docs
manifest.json
repository.json
tmp
update.lock
```
 
From this folder, navigate to the ``bootloader`` folder with:

``` 
$ cd TK1/Linux_for_Tegra_tk1/bootloader
```

When in the correct folder, turn on the Jetson board in the *Recovery Mode* by powering down the device (in case Jetson is on), connecting the micro-USB cable in the recovery port and in the USB of the local host, pressing and holding the FORCE RECOVERY button while turning the board on. In order to check if Jetson is in *Recovery Mode*, in your localhost, run:

```
$ lsusb
```

If the board is in *Recovery Mode*, you should see the Jetson listed as NVidia (ID 0955:7140 NVidia Corp.) in the output, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_6.png)

Having the Jetson board in *Recovery Mode*, run:

```
$ sudo ./nvflash --read APP clone.img --bl ardbeg/fastboot.bin --go
```

When running this command, an image of the Jetson board starts to be recorded into ``clone.img`` file.


## Restoring the system image to Jetson 

In order to restore the image from the localhost to the Jetson, first you navigate to the proper directory on the localhost to begin the process. Open a Terminal and go to the JetPack install directory (e.g., ``/home/roger/JetPack/``). From this folder, navigate to the ``Linux_for_Tegra_tk1`` folder with:

``` 
$ cd TK1/Linux_for_Tegra_tk1/
```

When in the correct folder, turn on the Jetson board in the *Recovery Mode* by powering down the device (in case Jetson is on), connecting the micro-USB cable in the recovery port and in the USB of the local host, pressing and holding the FORCE RECOVERY button while turning the board on. In order to check if Jetson is in *Recovery Mode*, in your localhost, run:

```
$ lsusb
```

If the board is in *Recovery Mode*, you should see the Jetson listed as NVidia (ID 0955:7140 NVidia Corp.) in the output, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_6.png)

Having the Jetson board in *Recovery Mode*, run:

```
$ sudo ./flash.sh -r -S 14580MiB jetson-tk1 mmcblk0p1
```

When running this command, the image stored in ``system.img`` in the ``bootloader`` folder will be flashed in the Jetson board. The ``-r`` flag skips building and reuse the existing ``system.img`` file. The partition size ``-S 14580MiB`` is the default that JetPack uses. When the flashing process termines, reboot the Jetson. The Jetson will be restored to the state of the original at the time of cloning.

---
**NOTE**: If you already have performed a cloning before restoring the image to the Jetson, you should rename the clone image to ``system.img``. Otherwise, the image restored to the Jetson board contains the original SDK.
