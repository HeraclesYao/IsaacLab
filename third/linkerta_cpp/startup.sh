#!/bin/bash

PASSWORD="amitamit001"

echo "$PASSWORD" | sudo -S chmod 0666 /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo "$PASSWORD" | sudo -S echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo "$PASSWORD" | sudo -S chmod 777 /dev/ttyUSB0
sleep 3

cd build;./linkerta
