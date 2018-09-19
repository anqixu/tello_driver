#!/bin/bash

# Connect to access point
echo '- configuring device '$WIFI_DEV
ifconfig $WIFI_DEV down
iwconfig $WIFI_DEV mode managed essid $DRONE_AP
ifconfig $WIFI_DEV up
echo '- requesting for DHCP'
dhclient $WIFI_DEV

# Verify if device is connected to wifi (a.k.a. is allocated an IP)
WIFI_IP=$(ip a | grep $WIFI_DEV | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
if [ -z '$WIFI_IP' ]
then
    echo '! proxy container did not get IP from drone wifi (is drone turned on?)'
    iwconfig $WIFI_DEV
    ifconfig $WIFI_DEV
    exit 1
else
    echo '- proxy container connected to drone wifi via '$WIFI_IP
    CONTAINER_IP=$(ip a | grep eth0 | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
    echo '- SUCCESS! you can connect to wifi proxy container via:'
    echo ''
    echo $CONTAINER_IP
    exit 0
fi
