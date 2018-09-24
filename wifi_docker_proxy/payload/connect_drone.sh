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
if [ -z "$WIFI_IP" ]
then
    # Warn user
    echo '! proxy container did not get IP from drone wifi (is drone turned on?)'
    iwconfig $WIFI_DEV
    ifconfig $WIFI_DEV
    exit 1
else
    DOCKER_VETH_HOST_IP='172.17.0.1' # Hardcoded by Docker
    DRONE_IP='192.168.10.1'          # Hardcoded by drone
    DRONE_CMD_SERVER_PORT=8889       # Hardcoded by drone's UDP protocol
    PROXY_CMD_CLIENT_PORT=$LOCAL_CMD_CLIENT_PORT
    PROXY_VID_CLIENT_PORT=$((LOCAL_VID_SERVER_PORT+1))

    # Setup socat UDP port forwarding
    # CMD   SERVER: drone (192.168.10.1:8889) <-> docker (192.168.10.2:8890 / 172.17.0.XYZ:8889) <-> host  (  172.17.0.1:port)
    # VIDEO SERVER: host  (  172.17.0.1:6038) <-> docker (172.17.0.XYZ:6039 / 192.168.10.2:6038) <-> drone (192.168.10.1:port)
    nohup /usr/bin/socat UDP4-LISTEN:$DRONE_CMD_SERVER_PORT,fork UDP4:$DRONE_IP:$DRONE_CMD_SERVER_PORT,bind=:$PROXY_CMD_CLIENT_PORT &>/dev/null &
    nohup /usr/bin/socat UDP4-LISTEN:$LOCAL_VID_SERVER_PORT,fork UDP4:$DOCKER_VETH_HOST_IP:$LOCAL_VID_SERVER_PORT,bind=:$PROXY_VID_CLIENT_PORT &>/dev/null &

    # Notify user
    echo '- proxy container connected to drone wifi via '$WIFI_IP
    CONTAINER_IP=$(ip a | grep eth0 | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
    echo '- SUCCESS! UDP forwarding setup. You can connect to wifi proxy container via:'
    echo ''
    echo $CONTAINER_IP
    exit 0
fi