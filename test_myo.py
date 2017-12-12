"""Myo-to-OSC application.
Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000
"""
from myo_raw import *
from pythonosc import osc_message_builder
from pythonosc import udp_client


osc_client = udp_client.SimpleUDPClient("localhost", 3000)


def proc_imu(quat, acc, gyro):
    proc_quat = tuple(map(lambda x: x / ORIENTATION_SCALE, quat))
    proc_acc = tuple(map(lambda x: x / ACCELEROMETER_SCALE, acc))
    proc_gyro = tuple(map(lambda x: x / GYROSCOPE_SCALE, gyro))
    # print("quat:", proc_quat, "acc:", proc_acc, "gyro:", proc_gyro, end='\r')
    osc_client.send_message("/quat", proc_quat)
    osc_client.send_message("/acc", proc_acc)
    osc_client.send_message("/gyro", proc_gyro)

def proc_emg(emg_data):
    #print("emg:", em_data, end='\r')
    osc_client.send_message("/emg", emg_data)


def proc_battery(battery_level):
    #print("Battery", battery_level, end='\r')
    osc_client.send_message("/battery", battery_level)


m = MyoRaw(tty="/dev/tty.usbmodem1")
m.add_emg_handler(proc_emg)
m.add_imu_handler(proc_imu)
m.add_battery_handler(proc_battery)
m.connect()

# Setup Myo.
m.sleep_mode(Sleep_Mode.never_sleep.value)
m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
m.vibrate(1)

print("Now running...")
try:
    while True:
        m.run(1)
except KeyboardInterrupt:
    pass
finally:
    m.disconnect()
    print("Disconnected")


# TODO:
#   - direct connection to a specific myo.
#   - test out OSC sending
#   - move classification if then to myohw.py
#   - experiment connecting to multiple myos.