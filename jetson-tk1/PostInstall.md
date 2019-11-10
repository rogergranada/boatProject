# Post Installation

## Adding a SSD Disk

In order to add a SSD disk in Jetson board, we have to connect a SATA cable and a power cable in the board as illustrated in the image below:

!['SSD'](https://platypus-boats.readthedocs.io/en/latest/_images/connect_ssd.png)

After connecting the SSD disk, we format it using ``ext4`` file system by logging into Ubuntu and accessing ``Disks`` application. Inside the application, select the SSD disk and click on the gear and select ``Format`` (or press ``Shift+Ctrl+F``). Then, add a name to the disk (e.g. ``JetsonSSD``) and click in ``Format``, as the images below:

!['Disks'](https://platypus-boats.readthedocs.io/en/latest/_images/disks.png)

!['Format'](https://platypus-boats.readthedocs.io/en/latest/_images/format.png)

After formating the SSD disk, we have to mount it at startup. In order to do it, we have to add a call in ``/etc/fstab`` with the mounting point. In order to add this line, we have to discover the uuid of the device. With the disk manually mounted, we run the ``mount`` command to discover where the SSD is mounted, obtaining:

```
$ mount
/dev/sda on /media/ubuntu/JetsonSSD type ext4 (rw,nosuid,nodev,uhelper=udisk2)
```

Knowing the mounting local of the SSD disk (``/dev/sda``), we have to discover its uuid. To discover the uuid we run:

```
$ ls -al /dev/disk/by-uuid
lrwxrwxrwx 1 root root   9 Dec 31 21:00 ac183b24-3e75-4190-bcb7-32160e9a7c55 -> ../../sda
```

Having the uuid of the disk we can add a line to the ``/etc/fstab`` with a call to the mounting point. Running the command:

```
$ sudo gedit /etc/fstab
```

We add the following line to the file:

```
/dev/disk/by-uuid/ac183b24-3e75-4190-bcb7-32160e9a7c55 /media/JetsonSSD ext4 defaults 0 2
```

Save the file and close it. Next time Ubuntu is started, the SSD disk will be mounted at startup.

## Important Packages

Here we add several packages that should be installed to work in Jetson. All packages are installed via ``apt-get``. In order to easily install all packages a script was created and can be downloaded by running:

```
$ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/additionalPackages.sh
$ chmod +x additionalPackages.sh
$ ./additionalPackages.sh
```

The script installs the following packages:

- [Informational list of build-essential packages (build-essential)](https://packages.ubuntu.com/trusty/build-essential)
- [Cross-platform, open-source make system (CMake)](https://packages.ubuntu.com/trusty/cmake)
- [Curses based user interface for CMake (cmake-curses-gui)](https://packages.ubuntu.com/trusty/cmake-curses-gui)
- [GNU C++ compiler (G++)](https://packages.ubuntu.com/trusty/g++)
- [Set of I2C tools for Linux](https://packages.ubuntu.com/trusty/i2c-tools)
- [Userspace I2C programming library development files](https://packages.ubuntu.com/trusty/libi2c-dev)
- [Distributed revision control system (Git)](https://packages.ubuntu.com/trusty/git)
- [Multiple GNOME terminals in one window](https://packages.ubuntu.com/trusty/terminator)
- [Terminal multiplexer with VT100/ANSI terminal emulation (Screen)](https://packages.ubuntu.com/trusty/screen)

