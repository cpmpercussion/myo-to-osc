#
# Original work Copyright (c) 2014 Danny Zhu
# Modified work Copyright (c) 2017 Alvaro Villoslada, Fernando Cosentino
# More modications by Charles P. Martin, 2017.
#
# Licensed under the MIT license. See the LICENSE file for details.
#

import re
import struct
import binascii
from serial.tools.list_ports import comports
from .bluetooth import BT
from .myohw import *


class MyoRaw(object):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []
        self.battery_handlers = []

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('using device:', p[0])
                return p[0]
        return None

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)

    def scan_myo(self):
        print('Scanning for a Myo...')
        self.bt.discover()
        addr = None
        while addr is None:
            p = self.bt.recv_packet()
            if p.payload.endswith(MyoServiceInfoUuid):  # This is MYO_SERVICE_INFO_UUID
                addr = list(list(p.payload[2:8]))
                mac_address_string = "%x:%x:%x:%x:%x:%x" % struct.unpack("BBBBBB", bytes(addr[::-1]))
                print("Found a Myo:", mac_address_string)  # print the Myo's mac address
                break
        self.bt.end_scan()
        return addr

    def mac_string_to_ints(self, mac_string):
        """Returns a list of ints from standard mac address notation"""
        split_addr = mac_string.split(':')[::-1]  # split by :, then reverse byte order
        addr_bytes = [binascii.unhexlify(n) for n in split_addr]  # change to bytes
        addr_ints = [struct.unpack("B", n)[0] for n in addr_bytes]  # change to ints
        return addr_ints

    def get_name(self):
        """ Get the connected Myo's name. """
        name = self.read_attr(MyoChars.DeviceName.value)
        name = name.payload[5:] # chop off the first 5 bytes? junk for some reason.
        name = name.decode("utf-8")
        return name

    def get_firmware(self):
        """ Get the connected Myo's firmware version. """
        fw = self.read_attr(MyoChars.FirmwareVersionCharacteristic.value)
        # get firmware version.
        _, _, _, _, v0, v1, v2, v3 = struct.unpack('<BHBBHHHH', fw.payload)
        return v0, v1, v2, v3

    def connect(self, address=None):
        """ Connects to a Myo specified by MAC address, or scans for a Myo if no address is given. """
        # stop scanning and disconnect bluetooth as needed.
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        # Calculate address or scan for Myo if necessary.
        if address is None:
            addr = self.scan_myo()
        else:
            addr = self.mac_string_to_ints(address)
            print("Connecting to Myo:", address)
            print("Byte Address:", addr)

        # connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = list(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)
        # Print out some Myo details.
        print('name:', self.get_name())
        print('firmware: %d.%d.%d.%d' % self.get_firmware())

        #  Subscribe to services etc.
        # enable IMU data
        # self.write_attr(MyoChars.IMUDataDescriptor.value, b'\x01\x00')
        # enable on/off arm notifications
        # self.write_attr(MyoChars.ArmDescriptor, b'\x02\x00')
        # enable EMG notifications
        # self.start_raw(filtered)
        # enable battery notifications
        # self.write_attr(MyoChars.BatteryDescriptor, b'\x01\x10')

        # Add the handler function to the bluetooth connection.
        self.bt.add_handler(self.handle_ble_data)

    def handle_ble_data(self, p):
        """ Handle Data sent from the Bluetooth Connection. """
        if (p.cls, p.cmd) != (4, 5):
            return
        c, attr, typ = struct.unpack('<BHB', p.payload[:4])
        pay = p.payload[5:]
        # TODO: is this just for old firmware? maybe delete.
        if attr == MyoChars.EMGCharacteristic.value:
            # Unpack a 17 byte array, first 16 are 8 unsigned shorts, last one an unsigned char
            vals = struct.unpack('<8HB', pay)
            # not entirely sure what the last byte is, but it's a bitmask that
            # seems to indicate which sensors think they're being moved around or
            # something
            emg1 = vals[:8]
            motion = vals[8]
            self.on_emg(emg)
        # Read notification handles corresponding to the for EMG characteristics
        elif attr in MYO_EMG_CHARACTERISTICS:
            emg1, emg2 = emg_data(pay)
            self.on_emg(emg1, 0)
            self.on_emg(emg2, 0)
        # Read IMU characteristic handle
        elif attr == MyoChars.IMUDataCharacteristic.value:
            quat, acc, gyro = imu_data(pay)
            self.on_imu(quat, acc, gyro)
        # Read classifier characteristic handle
        elif attr == MyoChars.ClassifierCharacteristic.value:
            typ, val, xdir = classifier_event(pay)
            if typ == Classifier_Event_Type.arm_synced.value:  # on arm
                self.on_arm(Arm(val), X_Direction(xdir))
            elif typ == Classifier_Event_Type.arm_unsynced.value:  # removed from arm
                self.on_arm(Arm.unknown, X_Direction.unknown)
            elif typ == Classifier_Event_Type.pose.value:  # pose
                self.on_pose(Pose(val))
        # Read battery characteristic handle
        elif attr == MyoChars.BatteryCharacteristic.value:
            battery_level = ord(pay)
            self.on_battery(battery_level)
        else:
            print('data with unknown attr: %02X %s' % (attr, p))

    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    # def start_raw(self, filtered):
    #     ''' To get raw EMG signals, we subscribe to the four EMG notification
    #     characteristics by writing a 0x0100 command to the corresponding handles.
    #     '''
    #     if not filtered:
    #         self.write_attr(MyoChars.EmgData0Descriptor.value, b'\x01\x00')  # Suscribe to EmgData0Characteristic
    #         self.write_attr(MyoChars.EmgData1Descriptor.value, b'\x01\x00')  # Suscribe to EmgData1Characteristic
    #         self.write_attr(MyoChars.EmgData2Descriptor.value, b'\x01\x00')  # Suscribe to EmgData2Characteristic
    #         self.write_attr(MyoChars.EmgData3Descriptor.value, b'\x01\x00')  # Suscribe to EmgData3Characteristic

    #     if not filtered:
    #         # self.write_attr(MyoChars.CommandCharacteristic.value, b'\x01\x03\x02\x01\x01')
    #         self.set_mode(EMG_Mode.emg_mode_send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.enabled.value)

    #     '''By writting a 0x0100 command to handle 0x28, some kind of "hidden" EMG
    #     notification characteristic is activated. This characteristic is not
    #     listed on the Myo services of the offical BLE specification from Thalmic
    #     Labs. Also, in the second line where we tell the Myo to enable EMG and
    #     IMU data streams and classifier events, the 0x01 command which corresponds
    #     to the EMG mode is not listed on the myohw_emg_mode_t struct of the Myo
    #     BLE specification.
    #     These two lines, besides enabling the IMU and the classifier, enable the
    #     transmission of a stream of low-pass filtered EMG signals from the eight
    #     sensor pods of the Myo armband (the "hidden" mode I mentioned above).
    #     Instead of getting the raw EMG signals, we get rectified and smoothed
    #     signals, a measure of the amplitude of the EMG (which is useful to have
    #     a measure of muscle strength, but are not as useful as a truly raw signal).
    #     '''
    #     if filtered:
    #         self.write_attr(MyoChars.EMGDescriptor.value, b'\x01\x00')  # Not needed for raw signals # What's this handle?
    #         self.write_attr(MyoChars.CommandCharacteristic.value, b'\x01\x03\x01\x01\x01')

    def set_mode(self, emg_mode, imu_mode, cla_mode):
        """ Set the EMG, IMU and Classifier modes as described in myohw.py """
        command = command_set_mode(emg_mode, imu_mode, cla_mode)  # Construct the command
        self.write_attr(MyoChars.CommandCharacteristic.value, command)  # Send it.

    def sleep_mode(self, mode):
        """ Set the Myo's sleep mode. (See Sleep_Mode in myohw.py for details). """
        self.write_attr(MyoChars.CommandCharacteristic.value, command_set_sleep_mode(mode))

    def deep_sleep(self):
        """ Put the Myo into deep sleep mode. (Needs power cable to wake up)."""
        self.write_attr(MyoChars.CommandCharacteristic.value, command_deep_sleep())

    def vibrate(self, type):
        """ Send a vibrate command to the Myo, see Vibration_Type for the kinds of vibrations. """
        if type in range(1, 4):
            self.write_attr(MyoChars.CommandCharacteristic.value, command_vibrate(type))

    # Remove this (private) command.
    # def set_leds(self, logo, line):
    #     self.write_attr(MyoChars.CommandCharacteristic.value, struct.pack('<8B', 6, 6, *(logo + line)))

    def add_emg_handler(self, h):
        """ Add a handler function for EMG signals. Signature: function(emg). """
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        """ Add a handler function for IMU signals. Signature: function(quat, acc, gyro). """
        self.imu_handlers.append(h)

    def add_pose_handler(self, h):
        """ Add a handler for pose signals. Signature: function(pose). """
        self.pose_handlers.append(h)

    def add_arm_handler(self, h):
        """ Add a handler for arm signals. Signature: function(arm, x_direction). """
        self.arm_handlers.append(h)

    def add_battery_handler(self, h):
        """ Add a handler for battery signals. Signature: function(battery_level). """
        self.battery_handlers.append(h)

    def on_emg(self, emg):
        for h in self.emg_handlers:
            h(emg)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_pose(self, p):
        for h in self.pose_handlers:
            h(p)

    def on_arm(self, arm, xdir):
        for h in self.arm_handlers:
            h(arm, xdir)

    def on_battery(self, battery_level):
        for h in self.battery_handlers:
            h(battery_level)
