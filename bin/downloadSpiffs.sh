#!/bin/bash
PORT=${PORT:=/dev/ttyUSB0}
python3 "/home/jim/.arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool.py" --chip auto --port "${PORT}" --baud 921600  --before default_reset --after hard_reset read_flash  0x3d0000 0x20000 fs.bin
rm -rf fs
/home/jim/.arduino15/packages/esp32/tools/mkspiffs/0.2.3/mkspiffs -u fs fs.bin

