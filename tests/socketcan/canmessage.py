__author__ = 'hd'

import struct

class CanMessage:
    # CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
    frame_fmt = "=IB3x8s"

    def __init__(self, id, data=[], is_extended=False, is_rtr=False):
        self.id = id & 0x1FFFFFFF
        self.is_extended = is_extended or ((id & 0x80000000) != 0)
        self.is_rtr = is_rtr or ((id & 0x40000000) != 0)
        self.data = data

    def from_raw(frame):
        id, dlc, data = struct.unpack(CanMessage.frame_fmt, frame)
        data_arr = []
        for x in data[:dlc]:
            data_arr.append(x)
        return CanMessage(id, data_arr)

    def to_raw(self):
        id = self.id
        if self.is_extended:
            id |= 0x80000000
        if self.is_rtr:
            id |= 0x40000000
        dlc = len(self.data)
        bdata = bytearray()
        for x in self.data:
            bdata.append(x)
        return struct.pack(CanMessage.frame_fmt, id, dlc, bdata.ljust(8, b'\x00'))
