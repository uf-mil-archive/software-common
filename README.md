This repository contains ROS packages used in MIL that are
not specific to any robotic platform and run purely on a
host computer.

This repository depends on uf-mil's rawgps-tools repository.

This repository also depends on the
[uEye drivers/IDS Software Suite](http://en.ids-imaging.com/download-ueye.html),
installable with:

    # for 32 bit OS:
    mkdir ~/ueye && cd ~/ueye && wget http://en.ids-imaging.com/tl_files/downloads/uEye_SDK/driver/uEye_Linux_4.30_32_Bit.zip && unzip * && chmod +x *.run && sudo ./ueyesdk-setup-4.30-usb-i686.gz.run
    # for 64 bit OS:
    mkdir ~/ueye && cd ~/ueye && wget http://en.ids-imaging.com/tl_files/downloads/uEye_SDK/driver/uEye_Linux_4.30_64_Bit.zip && unzip * && chmod +x *.run && sudo ./ueyesdk-setup-4.30-usb-amd64.gz.run

This repository also depends on [PyODE](http://pyode.sourceforge.net/), 
installable with:

    sudo apt-get install python-pyode
