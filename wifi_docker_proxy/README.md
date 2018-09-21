## Connecting to multiple Tello drones

Problems:
* each Tello drone serves a separate access point (AP)
* currently there is no way to ask the drone to instead connect as client to another AP (since we cannot telnet or ssh into it)
* all Tello drones are configured to have an IP of `192.168.10.1`

Solution:
* use multiple devices (e.g. USB WiFi dongles) to connect to each drone
* to resolve subnet and IP clash, use a docker container to connect each device to a target drone, then bi-directionally forward UDP channels for telemetry and video
* e.g. before: client `wlan#` (`192.168.10.2:9000`) <-> drone (`192.168.10.1:8889`)
* e.g. after:  client `docker0` (`172.17.0.1:9000`) <-> container `eth0` (`172.17.0.2:8889`) <-`socat`-> container `wlan#` (`192.168.10.2:8889`) <-> drone (`192.168.10.1:8889`)
* NOTE: a similar UDP proxy needs to be setup for the video stream, although the server is running on the client rather than the drone

### Requirements:

* docker
* pcregrep

### Instructions:

* optional: add `alias go=./go.py` to `~/.bashrc`, or replace `go` with `./go.py` for commands below
* configure parameters: `$ cp config.ini.dist config.ini` and [modify content accordingly](config.ini.dist)
* build docker image: `$ go build`
* start docker container: `$ go setup` (see `--help` for overridding parameters)
* turn on drone
* configure docker container to connect to drone's WiFi and setup UDP relays: `$ go connect`
* connect driver to drone using IP given from output of above command
* teardown: `$ go stop`
