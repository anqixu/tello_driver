#!/bin/bash

# Terminate existing isntances of socat
pkill socat

# Connect to access point
echo '- configuring device '$WIFI_DEV
ifconfig $WIFI_DEV down
iwconfig $WIFI_DEV mode managed essid $DRONE_AP
ifconfig $WIFI_DEV up
echo '- requesting for DHCP'
dhclient $WIFI_DEV
pkill dhclient

# Verify if device is connected to wifi (a.k.a. is allocated an IP)
WIFI_IP=$(ip a | grep $WIFI_DEV | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
if [ -z '$WIFI_IP' ]
then
    # Warn user
    echo '! proxy container did not get IP from drone wifi (is drone turned on?)'
    iwconfig $WIFI_DEV
    ifconfig $WIFI_DEV
    exit 1
else
    # Setup socat UDP port forwarding
    # CMD   SERVER: drone (192.168.10.1:8889) <-> docker (192.168.10.2:8890 / 172.17.0.XYZ:8889) <-> host  (  172.17.0.1:port)
    # VIDEO SERVER: host  (  172.17.0.1:6038) <-> docker (172.17.0.XYZ:6039 / 192.168.10.2:6038) <-> drone (192.168.10.1:port)
    nohup /usr/bin/socat UDP4-LISTEN:8889,fork UDP4:192.168.10.1:8889,bind=:8890 &>/dev/null &
    nohup /usr/bin/socat UDP4-LISTEN:6038,fork UDP4:172.17.0.1:6038,bind=:6039 &>/dev/null &

    # Notify user
    echo '- proxy container connected to drone wifi via '$WIFI_IP
    CONTAINER_IP=$(ip a | grep eth0 | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
    echo '- SUCCESS! UDP forwarding setup. You can connect to wifi proxy container via:'
    echo ''
    echo $CONTAINER_IP
    exit 0
fi
