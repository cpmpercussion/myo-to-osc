"""Myo-to-OSC application.
Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000
"""
from myo import *
import math
from pythonosc import osc_message_builder
from pythonosc import udp_client


osc_client = udp_client.SimpleUDPClient("localhost", 3000)  # OSC Client for sending messages.


def vector_3d_magnitude(x, y, z):
    """Calculate the magnitude of a 3d vector"""
    return math.sqrt((x * x) + (y * y) + (z * z))


def toEulerAngle(w, x, y, z):
    """ Quaternion to Euler angle conversion borrowed from wikipedia.
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles """
    # roll (x-axis rotation)
    sinr = +2.0 * (w * x + y * z)
    cosr = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y-axis rotation)
    sinp = +2.0 * (w * y - z * x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny = +2.0 * (w * z + x * y)
    cosy = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def proc_imu(quat, acc, gyro):
    proc_quat = tuple(map(lambda x: x / ORIENTATION_SCALE, quat))
    proc_acc = tuple(map(lambda x: x / ACCELEROMETER_SCALE, acc))
    proc_gyro = tuple(map(lambda x: x / GYROSCOPE_SCALE, gyro))
    # print("quat:", proc_quat, "acc:", proc_acc, "gyro:", proc_gyro, end='\r')
    osc_client.send_message("/ori", proc_quat)
    osc_client.send_message("/acc", proc_acc)
    osc_client.send_message("/gyr", proc_gyro)
    roll, pitch, yaw = toEulerAngle(proc_quat[0], proc_quat[1], proc_quat[2], proc_quat[3])
    osc_client.send_message("/euler", (roll / math.pi, pitch / math.pi, yaw / math.pi))  # vals sent in [-1,1] (not [-pi,pi])
    osc_client.send_message("/accmag", vector_3d_magnitude(proc_acc[0], proc_acc[1], proc_acc[2]))  # magnitude of accelerometer vector
    osc_client.send_message("/gyrmag", vector_3d_magnitude(proc_gyro[0], proc_gyro[1], proc_gyro[2]))  # magnitude of gyroscope vector


def proc_emg(emg_data):
    proc_emg = tuple(map(lambda x: x / 127.0, emg_data))  # scale EMG to be in [-1, 1]
    # print("emg:", em_data, end='\r')
    osc_client.send_message("/emg", proc_emg)


def proc_battery(battery_level):
    # print("Battery", battery_level, end='\r')
    osc_client.send_message("/battery", battery_level)


# Setup Myo Connection
m = Myo(tty="/dev/tty.usbmodem1")  # MacOS
# m = Myo(tty="/dev/ttyACM0")  # Linux
m.add_emg_handler(proc_emg)
m.add_imu_handler(proc_imu)
m.add_battery_handler(proc_battery)
m.connect()  # connects to first Myo seen.

# Setup Myo mode, buzzes when ready.
m.sleep_mode(Sleep_Mode.never_sleep.value)
# EMG and IMU are enabled, classifier is disabled (thus, no sync gestures required, less annoying buzzing).
m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
# Buzz to show Myo is ready.
m.vibrate(1)

def run_loop():
    m.run()

print("Now running...")
try:
    while True:
        run_loop()
except KeyboardInterrupt:
    pass
finally:
    m.disconnect()
    print("Disconnected")


# TODO:
#   - direct connection to a specific myo.
#   - move classification if then to myohw.py
#   - experiment connecting to multiple myos.
#   - update to pyGatt rather than lame bluetooth backend.
