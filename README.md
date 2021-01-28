# Ricoh theta V driver node for ROS


 Ros node for get equirectangular image of Ricoh theta V camera(https://theta360.com/en/about/theta/v.html).
 

Use
----

 Before use, you need to connect theta to the computer using USB and then switch theta to live mode.
 
 For turn off the node, you have to call the service : rosservice call /turnoff "{}"
 
  - Sample image
  
Will be updated soon!

Prerequisit
----
```sh
$ sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev libjpeg-dev libavformat-dev libswscale-dev libavcodec-dev
$ git clone https://github.com/ricohapi/libuvc-theta.git
$ cd libuvc-theta
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

nvdecoder : follow this page https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200
```

Parameters
----
| name | explain |
| ------ | ------ |
| nvdec | Using Nvidia codec instead of  decodebin (https://codetricity.github.io/theta-linux/optimization/)|
| use4k | Get image as 4K(3840x1920) resolution. If false, image will be 2K(1920x960) resolution|
| /image/raw/on | Flag of publishing raw image or not |
| /image/compress/on | Flag of publishing compressed image(PNG) or not |
| /image/compress/level | Compressed level of PNG (0:no compression-9:max compression) |

Caution : cannot use raw and compress together. (raw image will be slowed by compression process)

TODOs
----

 - Timestamp in image message might not accurate. Have to fix it.
 - Add sensor list lookup node.
 - Cannot turn off the node with Ctrl-C

Refer codes/pages
----

https://codetricity.github.io/theta-linux/

https://github.com/hirorogithub/ffmpeg_sample-H264_to_cv-Mat


License
----

BSD

UVC Related license
----

Copyright (C) 2020 K.Takeo/RICOH Co., Ltd.

Copyright (C) 2010-2015 Ken Tossell
