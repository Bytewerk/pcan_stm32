__author__ = 'hd'

import socket
from socketcan import CanMessage

class CanSocket:

    def __init__(self):
        self.sock = None

    def open(self, interface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def close(self):
        self.sock.close()

    def read(self, timeout=None):
        self.sock.settimeout(timeout)
        try:
            frame, addr = self.sock.recvfrom(16)
            return CanMessage.from_raw(frame)
        except socket.timeout:
            return None

    def send(self, msg):
        self.sock.send(msg.to_raw())
