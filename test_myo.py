from myo_raw import *


def proc_imu(quat, acc, gyro):
    proc_quat = tuple(map(lambda x: x / ORIENTATION_SCALE, quat))
    proc_acc = tuple(map(lambda x: x / ACCELEROMETER_SCALE, acc))
    proc_gyro = tuple(map(lambda x: x / GYROSCOPE_SCALE, gyro))
    print("quat:", proc_quat, "acc:", proc_acc, "gyro:", proc_gyro, end='\r')


def proc_emg(em_data):
    print("emg:", em_data, end='\r')


m = MyoRaw(tty="/dev/tty.usbmodem1")
m.add_emg_handler(proc_emg)
m.add_imu_handler(proc_imu)
m.connect()
# m.sleep_mode(Sleep_Mode.never_sleep.value)

# m.start_raw(filtered=False)
m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.none.value, Classifier_Mode.disabled.value)
m.vibrate(1)

print("Now running...")

# command = command_set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)  # Construct the command
#             # self.write_attr(MyoChars.EMGDescriptor.value, b'\x01\x00')  # Not needed for raw signals # What's this handle?
#             # self.write_attr(MyoChars.CommandCharacteristic.value, b'\x01\x03\x01\x01\x01')

# command = command_set_mode(0x01, IMU_Mode.send_data.value, Classifier_Mode.enabled.value)  # Construct the command

try:
    while True:
        m.run(1)
except KeyboardInterrupt:
    pass
finally:
    m.disconnect()
    print("Disconnected")
