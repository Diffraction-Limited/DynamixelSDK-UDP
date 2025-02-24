#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2025 DIFFRACTION LIMITED
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Adam Robichaud

import time
import sys
import platform
import socket
import threading

LATENCY_TIMER = 16
DEFAULT_BAUDRATE = 1000000
DEFAULT_PORT = 6464

class PortHandlerUDP(object):

    # A worker class to handle the UDP communication asynchronously
    class Worker(object):
        def __init__(self, src_ip, src_port):
            self.ip_address = src_ip
            self.port = src_port
            self.rx_socket = None
            self.ready = False
            self.abort = False

            # existing initialization code
            self.queue_lock = threading.Lock()
            self.incoming_queue = []

        def setIP(self, ip):
            self.ip_address = ip
        
        def setPort(self, port):
            self.port = port

        def start(self):
            if (self.ready):
                return
            
            self.abort = False
            self.listener_thread = threading.Thread(target=self._listen_for_messages)
            self.listener_thread.daemon = True
            self.listener_thread.start()

        def isReady(self):
            with self.queue_lock:
                return self.ready

        def stop(self):
            with self.queue_lock:
                self.abort = True
            self.listener_thread.join()

        def flush(self):
            with self.queue_lock:
                self.incoming_queue = []

        def send(self, data):
            if isinstance(data, list):
                data = bytes(data)
            if self.rx_socket is not None:
                self.rx_socket.sendto(data, (self.ip_address, self.port))

        def getPacket(self, length):
            with self.queue_lock:
                if len(self.incoming_queue) > 0:
                    return self.incoming_queue.pop()
            return None
        
        def getBytesWaiting(self):
            with self.queue_lock:
                if len(self.incoming_queue) > 0:
                    return len(self.incoming_queue[0])

        def _listen_for_messages(self):
            # bind to a random Rx port on all addresses
            self.rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.rx_socket.bind(('', 0))
            self.rx_socket.settimeout(1.0)

            # let the main thread know we're ready to start listening
            with self.queue_lock:
                self.ready = True

            # Listen for incoming messages until abort is requested
            while not self.abort:
                try:
                    data, src_addr = self.rx_socket.recvfrom(1024)  # Adjust buffer size as needed
                    if src_addr[0] == self.ip_address and src_addr[1] == self.port:
                        with self.queue_lock:
                            self.incoming_queue.append(data)
                except socket.timeout:
                    continue
            
            # Close out the socket and clean up
            self.rx_socket.close()
            self.rx_socket = None

            # let the main thread know we're done
            with self.queue_lock:
                self.ready = False


    def __init__(self, port_name):
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.port_name = port_name
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0
        self.is_using = False

        # extract IP and port number from port_name
        self.setPortName(port_name)
               

    def openPort(self):
        if self.is_open:
            return True
        
        self.worker = self.Worker(self.ip_address, self.port)
        self.worker.start()
        while not self.worker.isReady():
            time.sleep(0.1)
        self.is_open = True
        return True

    def closePort(self):
        if not self.is_open:
            return
        
        self.worker.stop()
        self.is_open = False

    def clearPort(self):
        self.worker.flush()

    def setPortName(self, port_name):
        # Extract device IP and Port from port_name
        if port_name.startswith("udp://"):
            port_name = port_name[6:]
            if ':' in port_name:
                self.ip_address, self.port = port_name.split(':')
                self.port = int(self.port)
            else:
                self.ip_address = port_name
                self.port = DEFAULT_PORT  # Default port if not specified
        else:
            raise ValueError("Invalid port name format. Expected format: 'udp://<ip_address>:<port>'")
        
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudRate(self, baudrate):
        # superfluous, but required for compatibility with other port handlers
        self.baudrate = baudrate
        return True

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        return self.worker.getBytesWaiting() if self.worker.isReady() else 0

    def readPort(self, length):
        if not self.worker.isReady():
            self.worker.start()

        data = self.worker.getPacket(length)
        if data is not None:
            data = list(data)
        if data is None:
            return []
        return data

    def writePort(self, packet):
        if not self.worker.isReady():
            self.worker.start()

        self.worker.send(packet)
        return len(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (LATENCY_TIMER * 2.0) + 2.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since
