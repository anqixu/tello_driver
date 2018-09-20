#!/usr/bin/env python2

import socket
import struct
import threading

import time


class TelloDriver:
    def __init__(self, tello_ip='192.168.10.1', tello_port=8889):
        # Initialize state
        self.alive = False  # Set to False to terminate listeners
        self.state = 0  # enum; 0=connecting, 1=connected
        self.udp = None
        self.listener_thread = None

        # Connect to UDP server
        tello_addr = (tello_ip, tello_port)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.connect(tello_addr)
        self.alive = True
        self.state = 0

        # Setup UDP listener loop
        self.listener_thread = threading.Thread(target=self._listener_loop)
        self.listener_thread.start()

        # Issue connection request
        self._send_conn_req()

        # Wait for connection ack
        while self.state == 0:
            time.sleep(0.001)
        print('Connected to Tello drone')

    def _listener_loop(self):
        while self.alive:
            data = self.udp.recv(2048)
            if not data:
                print('! Got empty from recv: %s' % str(data))
                self.alive = False
                break

            if self.state == 0:
                if data.find('conn_ack:') == 0:
                    self.state = 1
                continue

            self.parse_packet(data)

    def parse_packet(self, data):
        if len(data) < 7:
            return  # raise ValueError('Packet too short')

        cmd = struct.unpack('<h', data[5:7])[0]
            if cmd == 86:  # telemetry

            elif cmd == 26:  # wifi str

            elif cmd == 53:  # light strength

            elif cmd == 26:  # wifi str

            print('R> %4d, cmd=%d' % (len(data), cmd))

            # parse state:     ! cmd == 86: parse state, starting from 9th byte
            # R > 35, cmd = 86
            # R > 13, cmd = 26
            # R > 12, cmd = 53
            # R > 270, cmd = 4176

    def _send_conn_req(self):
        packet = bytearray('conn_req:\x96\x17')
        self.udp.send(packet)

    def disconnect(self):
        self.alive = False
        if self.listener_thread is not None:
            self.listener_thread.join()
            self.listener_thread = None
        if self.udp is not None:
            self.udp.close()
            self.udp = None

    def takeoff(self):
        if not self.alive:
            return
        packet = bytearray('\xcc\x58\x00\x7c\x68\x54\x00\xe4\x01\xc2\x16')
        self.udp.send(packet)

    def throw_takeoff(self):
        if not self.alive:
            return
        packet = bytearray('\xcc\x58\x00\x7c\x48\x5d\x00\xe4\x01\xc2\x16')
        self.udp.send(packet)

    def land(self):
        if not self.alive:
            return
        packet = bytearray('\xcc\x60\x00\x27\x68\x55\x00\xe5\x01\x00\xba\xc7')
        packet[9] = 0x00  # what is this flag?
        self.udp.send(packet)


if __name__ == '__main__':
    tello = None
    try:
        tello = TelloDriver()
        '''
        print('Take off')
        tello.takeoff()
        time.sleep(2.0)
        print('Land')
        tello.land()
        time.sleep(1.0)
        '''
        input('Press key to terminate')
        # tello.disconnect()
    finally:
        tello.disconnect()
