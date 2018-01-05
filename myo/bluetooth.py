#
# Original work Copyright (c) 2014 Danny Zhu
# Modified work Copyright (c) 2017 Alvaro Villoslada, Fernando Cosentino
#
# Licensed under the MIT license. See the LICENSE file for details.
#

import re
from serial.tools.list_ports import comports
import struct
import binascii
import threading
import serial

BLE_RESPONSE_PKT = 0x00
BLE_EVENT_PKT = 0x80
WIFI_RESPONSE_PKT = 0x08
WIFI_EVENT_PKT = 0x88


def detect_myo_tty():
    """ Attempts to detect a Myo Bluetooth adapter among the system's serial ports. """
    for p in comports():
        if re.search(r'PID=2458:0*1', p[2]):
            print('available BT adapter:', p[0])
            return p[0]
    return None


def mac_ints_to_string(mac_ints):
    """Returns a string from a MAC address given as a list of ints."""
    return ":".join([binascii.hexlify(bytes([n])).decode('utf-8') for n in mac_ints[::-1]])


def mac_string_to_ints(mac_string):
    """Returns a list of ints from standard mac address notation"""
    split_addr = mac_string.split(':')[::-1]  # split by :, then reverse byte order
    addr_bytes = [binascii.unhexlify(n) for n in split_addr]  # change to bytes
    addr_ints = [struct.unpack("B", n)[0] for n in addr_bytes]  # change to ints
    return addr_ints


class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = bytes(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in list(self.payload)))


class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty=None, baudrate=115200):
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []
        # Setup Bluetooth Connection
        # TODO: put this elsewhere.
        # Detect available TTY Bluetooth dongle if not specified.
        if tty is None:
            tty = detect_myo_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')
        # Connect to the Bluetooth dongle via serial connection.
        self.ser = serial.Serial(port=tty, baudrate=baudrate, dsrdtr=1)

    def recv_packet(self, timeout=None):
        self.ser.timeout = None
        while True:  # note that this could block if a packet is never received.
            c = self.ser.read()
            if not c:
                return None
            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == BLE_EVENT_PKT:
                    self.handle_event(ret)
                return ret

    def proc_byte(self, c):
        if not self.buf:
            if c in [BLE_RESPONSE_PKT, BLE_EVENT_PKT, WIFI_RESPONSE_PKT, WIFI_EVENT_PKT]:  # [BLE response pkt, BLE event pkt, wifi response pkt, wifi event pkt]
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try:
            self.handlers.remove(h)
        except ValueError:
            pass

    def wait_event(self, cls, cmd):
        res = [None]

        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    def connect(self, address):
        """ Connect to a BLE device. Specify by MAC address in 'xx:xx:xx:xx:xx:xx' format. """
        addr = mac_string_to_ints(address)
        return self.send_command(6, 3, struct.pack('<6sBHHHH', bytes(addr), 0, 6, 6, 64, 0))  # Todo, figure out this command

    def get_connections(self):
        return self.send_command(0, 6)  # Todo, figure out this command

    def discover(self):
        return self.send_command(6, 2, b'\x01')  # Todo, figure out this command

    def end_scan(self):
        return self.send_command(6, 4)  # Todo, figure out this command

    def disconnect(self, h):
        return self.send_command(3, 0, struct.pack('<B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, struct.pack('<BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, struct.pack('<BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload=b'', wait_resp=True):
        s = struct.pack('<4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()
            # no timeout, so p won't be None
            if p.typ == 0:
                return p
            # not a response: must be an event
            self.handle_event(p)
