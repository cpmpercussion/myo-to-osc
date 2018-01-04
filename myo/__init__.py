# Myo Bluetooth Library
# Charles P. Martin, 2017.
# Thanks to Danny Zhu, Alvaro Villoslada, and Fernando Cosentino
#
# Licensed under the MIT license. See the LICENSE file for details.
#

import pygatt
from binascii import hexlify
from .myohw import *

# address = 'C8:2F:84:E5:88:AF'

class Myo(object):
    '''Manages a connection with a Myo dongle.'''

    def __init__(self, adapter):
        self.adapter = adapter
        self.device = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []
        self.battery_handlers = []
        self.name = None
        self.firmware = None

    def connect(self, address):
        """ Connects to a Myo specified by MAC address,
        or scans for a Myo if no address is given."""
        # stop scanning and disconnect bluetooth as needed.
        self.device = self.adapter.connect(address)
        # Print out some Myo details.
        self.name = self.get_name()
        self.firmware = self.get_firmware()
        print('name:', self.name)
        print('firmware: %d.%d.%d.%d' % self.firmware)
        # Subscribe to services etc.
        self.device.subscribe(myo_uuid(Services.IMUDataCharacteristic.value), callback=self.accept_imu_data)
        self.device.subscribe(myo_uuid(Services.MotionEventCharacteristic.value), callback=self.accept_motion_data)
        self.device.subscribe(myo_uuid(Services.EmgData0Characteristic.value), callback=self.accept_emg_data)
        self.device.subscribe(myo_uuid(Services.EmgData1Characteristic.value), callback=self.accept_emg_data)
        self.device.subscribe(myo_uuid(Services.EmgData2Characteristic.value), callback=self.accept_emg_data)
        self.device.subscribe(myo_uuid(Services.EmgData3Characteristic.value), callback=self.accept_emg_data)

    def disconnect(self):
        """ Disconnects the device. """
        self.device.disconnect()

    def accept_imu_data(self, handle, value):
        quat, acc, gyro = imu_data(value)
        for handler in self.imu_handlers:
            handler(quat, acc, gyro)

    def accept_motion_data(self, handle, value):
        typ, val, xdir = classifier_event(value)
        if typ == Classifier_Event_Type.arm_synced.value:  # on arm
            for handler in self.arm_handlers:
                handler(Arm(val), X_Direction(xdir))
        elif typ == Classifier_Event_Type.arm_unsynced.value:  # removed from arm
            for handler in self.arm_handlers:
                handler(Arm.unknown, X_Direction.unknown)
        elif typ == Classifier_Event_Type.pose.value:  # pose
            for handler in self.pose_handlers:
                handler(Pose(val))

    def accept_emg_data(self, handle, value):
        emg1, emg2 = emg_data(value)
        for handler in self.emg_handlers:
            handler(emg1)
            handler(emg2)

    def disconnect(self):
        self.device.disconnect()

    def get_name(self):
        """ Get the connected Myo's name. """
        name = self.device.char_read(standard_uuid(Standard_Services.DeviceName.value))
        name = name.decode("utf-8")
        return name

    def get_firmware(self):
        """ Get the connected Myo's firmware version. """
        fw = self.device.char_read(myo_uuid(Services.FirmwareVersionCharacteristic.value))
        return fw_version(fw)

    def set_mode(self, emg_mode, imu_mode, cla_mode):
        """ Set the EMG, IMU and Classifier modes as described in myohw.py """
        command = command_set_mode(emg_mode, imu_mode, cla_mode)  # Construct the command
        self.device.char_write(myo_uuid(Services.CommandCharacteristic.value), command)  # Send it.

    def sleep_mode(self, mode):
        """ Set the Myo's sleep mode. (See Sleep_Mode in myohw.py for details). """
        self.device.char_write(myo_uuid(Services.CommandCharacteristic.value), command_set_sleep_mode(mode))

    def deep_sleep(self):
        """ Put the Myo into deep sleep mode. (Needs power cable to wake up)."""
        self.device.char_write(myo_uuid(Services.CommandCharacteristic.value), command_deep_sleep())

    def vibrate(self, typ):
        """ Send a vibrate command to the Myo, see Vibration_Type for the kinds of vibrations. """
        if type in range(1, 4):
            self.device.char_write(myo_uuid(Services.CommandCharacteristic.value), command_vibrate(typ))

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
