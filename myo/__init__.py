#
# Original work Copyright (c) 2014 Danny Zhu
# Modified work Copyright (c) 2017 Alvaro Villoslada, Fernando Cosentino
# More modications by Charles P. Martin, 2017.
#
# Licensed under the MIT license. See the LICENSE file for details.
#

import struct
from .bluetooth import *
from .myohw import *
import time


def discover_myos(adapter):
    """ Scans the Bluetooth adapter for available Myos. 
    Prints out their names and MAC addresses. """
    print('Scanning for Myos...')
    adapter.discover()
    myo_details = {}
    t = time.time()
    while time.time() - t < 1.0:  # todo, put a timeout here.
        p = adapter.recv_packet()
        name = p.payload[8:]
        name = name[5:].split(b'\x00')[0]
        mac_address = list(list(p.payload[2:8]))
        uuid = p.payload[-16:]
        mac_address_string = mac_ints_to_string(mac_address)
        if uuid == MyoServiceInfoUuid:  # This is MYO_SERVICE_INFO_UUID # found a myo.
            name = name.decode('utf-8')
            if name not in myo_details.keys():
                print("Found a Myo:", name, "MAC:", mac_address_string)  # print the Myo's mac address
            myo_details[name] = mac_address_string
    adapter.end_scan()
    return myo_details


class Myo(object):
    '''Manages a connection with a Myo dongle, handles receiving messages,
    sending to handler functions, and sending configuration.'''

    def __init__(self, adapter=None, tty=None):
        """ Initialise a Myo handler.

        Keyword arguments:
        adapter -- A BT object connected to a USB Bluetooth dongle.
        tty -- A serial port connected to a USB Bluetooth dongle.
        """
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []
        self.battery_handlers = []
        self.name = None
        self.firmware = None
        if adapter is None:
            self.bt = BT(tty, baudrate=115200)
        else:
            self.bt = adapter

    def run(self):
        """ Receive BLE packets. Run this command in a loop to receive data. """
        self.bt.recv_packet()

    def connect(self, address=None):
        """ Connects to a Myo specified by MAC address, or scans for a Myo if no address is given. 

        Keyword arguments:
        address -- a Myo's MAC address string in "xx:xx:xx:xx:xx:xx" format.

        If no address is given, scans and connects to first responding Myo.
        """
        # stop scanning and disconnect bluetooth as needed.
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        # scan for Myo if necessary.
        if address is None:
            myo_details = discover_myos(self.bt)
            address = list(myo_details.values())[0]  # Could fail if no Myo is found.

        # Calculate address
        print("Connecting to Myo:", address)

        # connect and wait for status event
        conn_pkt = self.bt.connect(address)
        self.conn = list(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)  # TODO: figure out this line.
        # Print out some Myo details.
        self.name = self.get_name()
        self.firmware = self.get_firmware()
        print('Myo Connected.')
        print('name:', self.name)
        print('firmware: %d.%d.%d.%d' % self.firmware)

        # Subscribe to services
        self.write_attr(MyoChars.IMUDataDescriptor.value, b'\x01\x00')  # enable IMU data
        self.write_attr(MyoChars.ArmDescriptor.value, b'\x02\x00')  # enable on/off arm notifications
        self.write_attr(MyoChars.EmgData0Descriptor.value, b'\x01\x00')  # Suscribe to EmgData0Characteristic
        self.write_attr(MyoChars.EmgData1Descriptor.value, b'\x01\x00')  # Suscribe to EmgData1Characteristic
        self.write_attr(MyoChars.EmgData2Descriptor.value, b'\x01\x00')  # Suscribe to EmgData2Characteristic
        self.write_attr(MyoChars.EmgData3Descriptor.value, b'\x01\x00')  # Suscribe to EmgData3Characteristic
        self.write_attr(MyoChars.BatteryDescriptor.value, b'\x01\x10')  # Subscribe to battery notifications

        # Add the handler function to the bluetooth connection.
        self.bt.add_handler(self.handle_ble_data)

    def handle_ble_data(self, p):
        """ Handle Data sent from the Bluetooth Connection. """
        if (p.cls, p.cmd) != (4, 5):
            return
        c, attr, typ = struct.unpack('<BHB', p.payload[:4])
        pay = p.payload[5:] # chop off the first five bytes of the payload for some reason.
        # Read notification handles corresponding to the for EMG characteristics
        if attr in MYO_EMG_CHARACTERISTICS:
            self.on_emg(pay)
        # Read IMU characteristic handle
        elif attr == MyoChars.IMUDataCharacteristic.value:
            self.on_imu(pay)
        # Read classifier characteristic handle
        elif attr == MyoChars.ClassifierCharacteristic.value:
            self.on_gesture(pay)
        # Read battery characteristic handle
        elif attr == MyoChars.BatteryCharacteristic.value:
            self.on_battery(pay)
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

    def get_name(self):
        """ Get the connected Myo's name. """
        name = self.read_attr(MyoChars.DeviceName.value)
        name = name.payload[5:]  # chop off the first 5 bytes? junk for some reason.
        name = name.decode("utf-8")
        return name

    def get_firmware(self):
        """ Get the connected Myo's firmware version. """
        fw = self.read_attr(MyoChars.FirmwareVersionCharacteristic.value)
        fw = fw.payload[5:]  # chop off the first 5 bytes? Junk for some reason.
        return fw_version(fw)

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

    def on_emg(self, emg_input_data):
        """ Sends EMG data on to any registered handler function.
        Note that each EMG reading is an int in [-127, 127]."""
        emg1, emg2 = emg_data(emg_input_data)
        for h in self.emg_handlers:
            h(emg1)
            h(emg2)

    def on_imu(self, imu_input_data):
        """Parses and scales the IMU data according to the myohw constants and sends it on to
        any registered handler function."""
        quat, acc, gyro = imu_data(imu_input_data)
        quat = tuple(map(lambda x: x / ORIENTATION_SCALE, quat))
        acc = tuple(map(lambda x: x / ACCELEROMETER_SCALE, acc))
        gyro = tuple(map(lambda x: x / GYROSCOPE_SCALE, gyro))
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_gesture(self, gesture_input_data):
        """ Parses gesture classification data and sends to appropriate
        handler functions. """
        typ, val, xdir = classifier_event(gesture_input_data)

        # On arm state.
        if typ == Classifier_Event_Type.arm_synced.value:
            for h in self.arm_handlers:
                h(Arm(val), X_Direction(xdir))

        # Removed from arm state
        elif typ == Classifier_Event_Type.arm_unsynced.value:
            for h in self.arm_handlers:
                h(Arm.unknown, X_Direction.unknown)

        # Pose states
        elif typ == Classifier_Event_Type.pose.value:
            for h in self.pose_handlers:
                h(Pose(val))

    def on_battery(self, battery_level_data):
        """ Sends battery level on to any registered handler function. """
        level = ord(battery_level_data)
        for h in self.battery_handlers:
            h(level)
