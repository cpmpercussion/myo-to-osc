"""Myo-to-OSC application.
Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000
"""
from myo import *
import datetime
import math
import logging
from pythonosc import osc_message_builder
from pythonosc import udp_client

LOGGING = True  # If true, the script logs to a file.
LOG_FILE = datetime.datetime.now().isoformat().replace(":", "-")[:19] + "-myo-to-osc.log"  # Log file name.

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
    osc_client.send_message("/ori", quat)
    osc_client.send_message("/acc", acc)
    osc_client.send_message("/gyr", gyro)
    roll, pitch, yaw = toEulerAngle(quat[0], quat[1], quat[2], quat[3])
    osc_client.send_message("/euler", (roll / math.pi, pitch / math.pi, yaw / math.pi))  # vals sent in [-1,1] (not [-pi,pi])
    osc_client.send_message("/accmag", vector_3d_magnitude(acc[0], acc[1], acc[2]))  # magnitude of accelerometer vector
    osc_client.send_message("/gyrmag", vector_3d_magnitude(gyro[0], gyro[1], gyro[2]))  # magnitude of gyroscope vector
    if LOGGING:
        logging.info("{0}, imu, {1[0]}, {1[1]}, {1[2]}, {1[3]}, {2[0]}, {2[1]}, {2[2]}, {3[0]}, {3[1]}, {3[2]}".format(datetime.datetime.now().isoformat(), quat, acc, gyro))  # 1 + 4 + 3 + 3 = 11 args.


def proc_emg(emg_data):
    proc_emg = tuple(map(lambda x: x / 127.0, emg_data))  # scale EMG to be in [-1, 1]
    # print("emg:", em_data, end='\r')
    osc_client.send_message("/emg", proc_emg)
    if LOGGING:
        logging.info("{0}, emg, {1[0]}, {1[1]}, {1[2]}, {1[3]}, {1[4]}, {1[5]}, {1[6]}, {1[7]}".format(datetime.datetime.now().isoformat(), emg_data))


def proc_battery(battery_level):
    # print("Battery", battery_level, end='\r')
    osc_client.send_message("/battery", battery_level)


if LOGGING:
    logging.basicConfig(filename=LOG_FILE, level=logging.INFO)
# Setup Myo Connection
m = Myo()  # scan for USB bluetooth adapter and start the serial connection automatically
# m = Myo(tty="/dev/tty.usbmodem1")  # MacOS
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
    print("\nDisconnected")


# TODO:
#   - direct connection to a specific myo.
#   - move classification if then to myohw.py
#   - experiment connecting to multiple myos.
