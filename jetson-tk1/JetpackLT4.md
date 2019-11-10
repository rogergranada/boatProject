# Installing JetPack L4T

[JetPack](https://developer.nvidia.com/embedded/jetpack-3_0) (the Jetson SDK) is an on-demand all-in-one package that bundles and installs all software tools required to develop for the NVIDIA® Jetson Embedded Platform (including flashing the Jetson Developer Kit). JetPack includes host and target developer tools, APIs and packages (OS images, tools, APIs, middleware, samples, documentation including compiling samples) to enable developers to jump-start their development environment for developing with the Jetson Embedded Platform. The latest release of JetPack runs on an Ubuntu 14.04 Linux 64-bit host system and supports the Jetson TK1 Developer Kit.

## Setting up the local host

In order to flash the Jetson board with the SDK, the first step consists of installing Jetpack in a local machine. The local machine can set up all the environment before flashing the board with the SDK. From an Ubuntu 14.04 PC 64 bit host computer, you simply download the [JetPack LT4 3.0](https://developer.nvidia.com/embedded/jetpack) installer with the latest OS image from NVIDIA Web site (you'll have to sign in with your developer account) and follow the instructions in the setup guide. After downloading JetPack, install it in your local machine (not in the Jetson board).

```
$ chmod +x JetPack-L4T-3.0-linux-x64.run
$ ./JetPack-L4T-3.0-linux-x64.run
```

The process starts asking the folder to install JetPack and the board you are using to install the SDK, as follows the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_1.png)

Select **JETSON TK1 Developer Kit (32-bit) and Ubuntu host** option and click ``Next``. Keep configuring according to your needs. After finishing the selections, click ``Next`` to start downloading all packages, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_2.png)

After downloading all packages, it starts to installing them in your local machine. It took about 20 minutes in my machine... When the installation in the local host is finished, the following image is presented:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_3.png)

## Flashing the Jetson board

Next step is to configure how the binaries are transmited to the Jetson board. Thus, JetPack asks what is the layout of the network to transmit the data. Select **Device accesses internet via houter/switch.** and click ``Next``, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_4.png)

JetPack then asks you to put the Jetson board in the *Recovery Mode*, by powering down the device (in case Jetson is on), connecting the micro-USB cable in the recovery port and in the USB of the local host, pressing and holding the FORCE RECOVERY button while turning the board on, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_5.png)

In order to check if the Jetson board is set in recovery mode, open the terminal in the local host and type:

```
$ lsusb
```
If the board is in revery mode, you should see the Jetson listed as NVidia (*ID 0955:7140 NVidia Corp.*) in the output, as the image:

!['Jetpack'](https://platypus-boats.readthedocs.io/en/latest/_images/jetpack_6.png)

If you don't see the Jetson using ``lsusb``, then the device will not be flashed. In case the Jetson appears in ``lsusb``, return to the screen with the instructions and press ``Enter`` to start flashing the OS image in the board.

---
# Setting Up the OS

NVIDIA Jetson TK1 exhibits a lot of promise with lots of raw performance for its form factor and intended use, with low power consumption to boot. But as is typical with most of these types of products, the "out of the box" experience needs some help. A missing part in TK1 is a support out of the box for WiFi or Bluetooth. For people coming from commodity PCs, tablets, phones and such this is a little confusing. Usually one just installs a driver and the device starts to work. In the case of the Jetson, the actual signals on the board need to be played with a little, as well as having the driver issue. In order to overcome such issues, a new Linux kernel named [**Grinch**](http://www.jetsonhacks.com/2014/10/12/installing-grinch-linuxfortegra-l4t-nvidia-jetson-tk1/) includes a lot of the features to which most desktop users are accustomed.


## Installing Grinch kernel

The Grinch Kernel for L4T provides over 60 changes and additions to the kernel, including fixes, configuration, module and firmware support to the stock kernel. The kernel is written and supported by Jetson Forum user Santyago. For further information about the Grinch Kernel, please see the [NVidia Jetson Forum](https://devtalk.nvidia.com/forums/board/162/).

In order to install the Grinch kernel, download the ``installGrinch.sh`` file and run as:

```
$ wget --no-check-certificate --content-disposition https://raw.githubusercontent.com/lsa-pucrs/platypus_doc/master/docs/source/jetson/scripts/installGrinch.sh
$ chmod +x installGrinch.sh
$ ./installGrinch.sh
```

Running these command will download the kernel image and install it in Jetson board. After the install script completes, reboot the Jetson TK1.

## Enabling USB 3.0

By default, the USB port in Jetson TK1 is not compatible to 3.0 devices. In order to enable the USB 3.0 support, you have to configure the ``extlinux.conf`` file, changing the parameter ``usb_port_owner_info=0`` to ``usb_port_owner_info=2``. In order to do so, run:

```
$ sudo gedit /boot/extlinux/extlinux.conf
```

The ``extlinux.conf`` file looks like:

```
  TIMEOUT 30
  DEFAULT primary

  MENU TITLE Jetson-TK1 eMMC boot options

  LABEL primary
    MENU LABEL primary kernel
    LINUX /boot/zImage
    FDT /boot/tegra124-jetson_tk1-pm375-000-c00-00.dtb
    APPEND console=ttyS0,115200n8 console=tty1 no_console_suspend=1 lp0_vec=2064@0xf46ff000 mem=2015M@2048M memtype=255 ddr_die=2048M@2048M section=256M pmuboard=0x0177:0x0000:0x02:0x43:0x00 tsec=32M@3913M otf_key=c75e5bb91eb3bd947560357b64422f85 usbcore.old_scheme_first=1 core_edp_mv=1150 core_edp_ma=4000 tegraid=40.1.1.0.0 debug_uartport=lsport,3 power_supply=Adapter audio_codec=rt5640 modem_id=0 android.kerneltype=normal fbcon=map:1 commchip_id=0 usb_port_owner_info=2 lane_owner_info=6 emc_max_dvfs=0 touch_id=0@0 board_info=0x0177:0x0000:0x02:0x43:0x00 net.ifnames=0 root=/dev/mmcblk0p1 rw rootwait tegraboot=sdmmc gpt
```

Look for ``usb_port_owner_info=0`` in the file and replace it by ``usb_port_owner_info=2``. Finally, save the file and exit.
