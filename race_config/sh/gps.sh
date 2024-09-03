#!/bin/bash

echo "Stop GPSD!"
sudo systemctl stop gpsd.socket
sleep 2

echo "Setup USB0 baud rate!"
sudo stty -F /dev/ttyUSB0 9600
sleep 2

echo "Restart GPSD!"
sudo systemctl restart gpsd.socket

echo "All Done!"
echo " Check gpsd: cgps -s"
echo " Check chrony: watch -n -0.1 chronyc sources -v"
echo "Exit now!"
exit

