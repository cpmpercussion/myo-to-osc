import struct
import pygatt
from binascii import hexlify
from myohw import *

# turn on some stuff
def set_mode(emg_mode, imu_mode, cla_mode):
        """ Set the EMG, IMU and Classifier modes as described in myohw.py """
        command = command_set_mode(emg_mode, imu_mode, cla_mode)  # Construct the command
        device.char_write(myo_uuid(Services.CommandCharacteristic.value), command)  # Send it.

def accept_imu_data(handle, value):
    quat, acc, gyro = imu_data(value)
    print("Q:", quat, "A:", acc, "G:", gyro, end='\r')

def motion_handler(handle, value):
    typ, val, xdir = classifier_event(value)
    print("T:", typ, "V", val, "xdir", xdir, end='\r')

def accept_emg_data(handle, value):
    emg1, emg2 = emg_data(value)
    print("EMG:", emg1, end='\r')

address = 'C8:2F:84:E5:88:AF'
adapter = pygatt.BGAPIBackend()
adapter.start()
device = adapter.connect(address)


device.subscribe(myo_uuid(Services.IMUDataCharacteristic.value), callback=accept_imu_data)
device.subscribe(myo_uuid(Services.MotionEventCharacteristic.value), callback=motion_handler)
device.subscribe(myo_uuid(Services.EmgData0Characteristic.value), callback=accept_emg_data)
device.subscribe(myo_uuid(Services.EmgData1Characteristic.value), callback=accept_emg_data)
device.subscribe(myo_uuid(Services.EmgData2Characteristic.value), callback=accept_emg_data)
device.subscribe(myo_uuid(Services.EmgData3Characteristic.value), callback=accept_emg_data)




set_mode(EMG_Mode.none.value, IMU_Mode.none.value, Classifier_Mode.enabled.value)

