#!/bin/bash 
ssh ls 'rm simplePost/firmware.bin simplePost/firmware.ver'
git describe --abbrev=8 --dirty --always --tags > firmware.ver
scp /tmp/arduino/espLightSleepPWM/esp32c3/espLightSleepPWM.ino.bin ls:simplePost/firmware.bin
scp firmware.ver ls:simplePost/firmware.ver
cp /tmp/arduino/espLightSleepPWM/esp32c3/espLightSleepPWM.ino.bin ~/src/simplePost/firmware.bin
cp firmware.ver ~/src/simplePost/firmware.ver

    
