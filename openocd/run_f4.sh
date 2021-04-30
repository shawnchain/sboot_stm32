#!/bin/sh

#openocd "-c" "gdb_port 50000" "-s" "/Users/shawn/works/HAM/MMDVM/MMDVM-Firmware/MMDVM-Firmware-X" "-f" "./openocd/openocd_f4_nucleo.cfg"

BOARD=
if [ -n "$1" ]; then
    arg="$1"
    case "$arg" in
        nucleo)
            BOARD="_nucleo"
            ;;
        jlink)
            BOARD="_jlink"
            ;;
        *)
            ;;
    esac
fi

echo "---------------------------"
echo "Using openocd_f4$BOARD.cfg"
echo "---------------------------"
openocd "-c" "gdb_port 50000" "-s" "/Users/shawn/works/HAM/MMDVM/MMDVM-Firmware/MMDVM-Firmware-X" "-f" "./openocd/openocd_f4$BOARD.cfg"
