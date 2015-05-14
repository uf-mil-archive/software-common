This repository contains ROS packages used in MIL that are
not specific to any robotic platform and run purely on a
host computer.

This repository depends on uf-mil's rawgps-tools repository.

This repository also depends on the
[uEye drivers/IDS Software Suite](http://en.ids-imaging.com/download-ueye.html),
installable with:

    # for 32 bit OS:
    mkdir ~/ueye && cd ~/ueye && wget http://en.ids-imaging.com/tl_files/downloads/uEye_SDK/driver/uEye_Linux_4.61_32_Bit.zip && unzip * && chmod +x *.run && sudo ./ueyesdk-setup-4.61-usb-i686.gz.run
    # for 64 bit OS:
    mkdir ~/ueye && cd ~/ueye && wget http://en.ids-imaging.com/tl_files/downloads/uEye_SDK/driver/uEye_Linux_4.61_64_Bit.zip && unzip * && chmod +x *.run && sudo ./ueyesdk-setup-4.61-usb-amd64.gz.run

This repository also depends on [PyODE](http://pyode.sourceforge.net/),
but the version in Ubuntu 14.04 seems to be broken. Oddly, building it
from source works. Run this to do so:

    rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb

This repository also depends on [Pygame](http://www.pygame.org/)
and libfftw3, installable with:

    sudo apt-get install python-pygame libfftw3-dev
