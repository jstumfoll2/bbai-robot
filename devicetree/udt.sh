#! /bin/bash
cd ~/src/BeagleBoard-DeviceTrees
make src/arm/am5729-beagleboneai-roboticscape.dtb
echo "Installing device tree overlay"
sudo cp src/arm/am5729-beagleboneai-roboticscape.dtb /boot/dtbs/4.14.108-ti-r136/
echo "Complete"
