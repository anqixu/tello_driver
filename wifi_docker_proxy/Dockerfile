FROM ubuntu:18.04

MAINTAINER Anqi Xu <ax@elementai.com>

# Install networking tools
RUN apt-get update -y --fix-missing \
    && apt-get install -y iproute2 isc-dhcp-client iputils-ping net-tools wireless-tools socat pcregrep \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

# Inject script to connect to wifi
COPY payload/connect_drone.sh /

# Reduce dhclient's timeout
RUN sed -i 's/timeout 300;/timeout 10;/g' /etc/dhcp/dhclient.conf

# Configure environment
ENV SHELL=/bin/bash
ENV TERM=xterm
# Fix Python-3 compatibility (see http://bugs.python.org/issue19846)
ENV LANG C.UTF-8

CMD exec /bin/bash -c "trap : TERM INT; sleep infinity & wait"