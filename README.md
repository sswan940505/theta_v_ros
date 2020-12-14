# THETA V DRIVER
*Prerequisit
$ sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev libjpeg-dev libavformat-dev libswscale-dev libavcodec-dev
$ git clone https://github.com/ricohapi/libuvc-theta.git
$ cd libuvc-theta
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

*Refer codes
https://codetricity.github.io/theta-linux/
https://github.com/hirorogithub/ffmpeg_sample-H264_to_cv-Mat


License of UVC-related codes
Copyright (C) 2020 K.Takeo/RICOH Co., Ltd.
Copyright (C) 2010-2015 Ken Tossell
