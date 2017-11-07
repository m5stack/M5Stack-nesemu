#!/bin/bash
# . ${IDF_PATH}/add_path.sh
 ~/Documents/Arduino/hardware/espressif/esp32/tools/esptool --chip esp32 --port "/dev/cu.SLAB_USBtoUART" --baud $((115200)) write_flash -fs 4MB 0x100000 "$1"
