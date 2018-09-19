#!/bin/bash

# Parse arguments
if [ "$#" -lt 2 ]; then
    echo 'Usage:' $0 'CONTAINER_NAME WIFI_DEV'
    exit 1
fi
CONTAINER_NAME=$1
WIFI_DEV=$2

# Inspired by: https://github.com/fgg89/docker-ap/wiki/Container-access-to-wireless-network-interface
PID=$(docker inspect -f '{{.State.Pid}}' $CONTAINER_NAME)

# Create network namespace dir and prune previous outdated simlinks
sudo mkdir -p /var/run/netns
for path in /var/run/netns/*; do if [ -L "$path" ] && ! [ -e "$path" ]; then sudo rm "$path"; fi; done

# Setup network namespace for container
sudo ln -s /proc/$PID/ns/net /var/run/netns/$PID

# Assign physical device for wifi dongle to container's network namespace
PHYID=phy$(iw dev | grep -B 1 $WIFI_DEV | pcregrep -o1 'phy\#([0-9]+)')
sudo iw phy $PHYID set netns $PID

# From now on, assuming the container is '--privileged', it will be able to interact with the wifi dongle