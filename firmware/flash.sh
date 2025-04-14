#!/bin/bash
wait_for_rpi_rp2() {
    echo "Waiting for RPI-RP2 device..."
    while true; do
        # Check for RPI-RP2 by USB device (vendor:product ID for Pico is 2e8a:0003)
        if lsblk -d -o NAME,LABEL | grep -q "RPI-RP2"; then
            echo "RPI-RP2 detected!"
            break
        elif ls /dev/disk/by-id/ | grep -q "usb-RPI_RP2"; then
            echo "RPI-RP2 detected by disk ID!"
            break
        fi
        sleep 0.5
    done
}

stty -F /dev/ttyACM* 1200
wait_for_rpi_rp2
picotool load -t elf "$@"
picotool reboot
