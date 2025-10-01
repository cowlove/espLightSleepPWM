#!/bin/bash -x 
ssh ls 'rm simplePost/firmware.bin simplePost/firmware.ver'
git describe --abbrev=6 --dirty --always > firmware.ver
scp ./build/esp32c3/espLightSleepPWM.ino.bin ls:simplePost/firmware.bin
scp firmware.ver ls:simplePost/firmware.ver
cp ./build/esp32c3/espLightSleepPWM.ino.bin ~/src/simplePost/firmware.bin
cp firmware.ver ~/src/simplePost/firmware.ver

    
